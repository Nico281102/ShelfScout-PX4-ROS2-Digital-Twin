"""Telemetry helpers wrapping PX4 topics and inspection events."""

from __future__ import annotations

import time
import math
import threading
from typing import Optional
from enum import Enum, auto

import rclpy


from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetEntityState, GetModelState
from std_msgs.msg import String

from px4_msgs.msg import BatteryStatus, VehicleLocalPosition, VehicleStatus

from ..core.fsm import LocalPosition
from .qos import EVENTS_QOS, TELEMETRY_QOS, GAZEBO_QOS


# ---------------------------------------------------------------------------
# Spawn sync states
# ---------------------------------------------------------------------------

class SpawnSyncState(Enum):
    UNREQUESTED = auto()
    QUERYING = auto()
    READY = auto()
    FAILED = auto()


# ---------------------------------------------------------------------------
# Telemetry
# ---------------------------------------------------------------------------

class Telemetry:
    """Collects PX4 telemetry topics and exposes convenience accessors."""

    def __init__(self, node, *, gazebo_model: str = "iris_opt_flow") -> None:
        self._node = node

        # PX4 data
        self._local_position: Optional[LocalPosition] = None
        self._vehicle_status: Optional[VehicleStatus] = None
        self._battery_status: Optional[BatteryStatus] = None

        # Inspection system
        self._pending_inspection_result: Optional[str] = None
        self._low_light_event = False

        # Spawn offset management
        self._spawn_offset_enu: Optional[tuple[float, float, float]] = None
        self._spawn_offset_from_gazebo = False

        # Gazebo sync
        self._gazebo_model = gazebo_model
        self._gazebo_sub = None
        self._gazebo_model_detected = False
        self._gazebo_pose_wait_logged = False

        # Thread & state
        self._spawn_capture_thread: Optional[threading.Thread] = None
        self._spawn_sync_state = SpawnSyncState.UNREQUESTED
        self._spawn_sync_state_since = time.monotonic()

        self._spawn_ready_event = threading.Event()

        # Subscriptions
        node.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._on_local_position,
            TELEMETRY_QOS,
        )
        node.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self._on_vehicle_status,
            TELEMETRY_QOS,
        )
        node.create_subscription(
            BatteryStatus,
            "/fmu/out/battery_status",
            self._on_battery_status,
            TELEMETRY_QOS,
        )
        node.create_subscription(
            String,
            "overrack/inspection",
            self._on_inspection_event,
            EVENTS_QOS,
        )

        # Gazebo model states subscription
        self._gazebo_sub = node.create_subscription(
            ModelStates,
            "/gazebo/model_states",
            self._on_gazebo_model_states,
            GAZEBO_QOS,
        )

    # ----------------------------------------------------------------------
    # Callbacks
    # ----------------------------------------------------------------------

    def _on_local_position(self, msg: VehicleLocalPosition) -> None:
        """Stores PX4 local position and defines spawn offset fallback."""
        stamp = self._node.get_clock().now().nanoseconds / 1e9
        new_pose = LocalPosition(
            msg.x,
            msg.y,
            msg.z,
            stamp,
            bool(getattr(msg, "xy_valid", True)),
            bool(getattr(msg, "z_valid", True)),
        )

        self._local_position = new_pose

        # If Gazebo did NOT publish the offset, fallback to PX4 origin.
        if self._spawn_offset_enu is None:
            self._spawn_offset_enu = (new_pose.y, new_pose.x, -new_pose.z)
            self._node.get_logger().info(
                "Spawn offset (fallback PX4 odom) ENU = (%.2f, %.2f, %.2f)"
                % self._spawn_offset_enu
            )

    def _on_vehicle_status(self, msg: VehicleStatus) -> None:
        previous = self._vehicle_status
        self._vehicle_status = msg

        if previous is None or previous.nav_state != msg.nav_state:
            if self.is_offboard_active():
                self._node.get_logger().info("OFFBOARD mode active (ACK)")

        if previous is None or previous.arming_state != msg.arming_state:
            if self.is_armed():
                self._node.get_logger().info("ARMED (ACK)")

    def _on_battery_status(self, msg: BatteryStatus) -> None:
        self._battery_status = msg

    def _on_inspection_event(self, msg: String) -> None:
        payload = (msg.data or "").strip().upper()
        if payload == "LOW_LIGHT":
            self._low_light_event = True
        elif payload in {"OK", "SUSPECT"}:
            self._pending_inspection_result = payload
        else:
            self._node.get_logger().warn(
                f"Unknown inspection event: {payload!r}"
            )

    def _on_gazebo_model_states(self, msg: ModelStates) -> None:
        """Detects Gazebo model and triggers spawn offset capture."""
        model_name = (self._gazebo_model or "").strip()
        if not model_name or self._spawn_offset_from_gazebo:
            self._destroy_gazebo_subscription()
            return

        try:
            index = msg.name.index(model_name)
        except ValueError:
            return

        if not self._gazebo_model_detected:
            self._node.get_logger().info(
                f"Gazebo model '{model_name}' detected — waiting for valid pose..."
            )
            self._gazebo_model_detected = True

        if index >= len(msg.pose):
            return

        pose = msg.pose[index]
        if not self._model_pose_ready(pose):
            if not self._gazebo_pose_wait_logged:
                self._node.get_logger().debug(
                    f"Pose not ready for '{model_name}', waiting..."
                )
                self._gazebo_pose_wait_logged = True
            return

        self._node.get_logger().info(
            f"Gazebo pose ready — capturing spawn offset..."
        )
        self._launch_spawn_offset_capture()

    # ----------------------------------------------------------------------
    # Public accessors
    # ----------------------------------------------------------------------

    def local_position(self) -> Optional[LocalPosition]:
        return self._local_position

    def spawn_offset_enu(self) -> Optional[tuple[float, float, float]]:
        return self._spawn_offset_enu

    def spawn_sync_state(self) -> SpawnSyncState:
        return self._spawn_sync_state

    def spawn_sync_state_since(self) -> float:
        return self._spawn_sync_state_since

    def spawn_offset_ready_event(self) -> threading.Event:
        return self._spawn_ready_event

    def vehicle_status(self) -> Optional[VehicleStatus]:
        return self._vehicle_status

    def battery_status(self) -> Optional[BatteryStatus]:
        return self._battery_status

    def is_offboard_active(self) -> bool:
        s = self._vehicle_status
        return bool(s and s.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def is_armed(self) -> bool:
        s = self._vehicle_status
        return bool(s and s.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def data_link_ok(self) -> bool:
        s = self._vehicle_status
        if s is None:
            return True
        if hasattr(s, "data_link_lost"):
            return not bool(s.data_link_lost)
        if hasattr(s, "rc_signal_lost"):
            return not bool(s.rc_signal_lost)
        return True

    def preflight_checks_pass(self) -> bool:
        s = self._vehicle_status
        return bool(s and s.pre_flight_checks_pass)

    def battery_warning(self) -> bool:
        s = self._battery_status
        return bool(s and s.warning >= BatteryStatus.BATTERY_WARNING_WARNING)

    def battery_critical(self) -> bool:
        s = self._battery_status
        return bool(s and s.warning >= BatteryStatus.BATTERY_WARNING_CRITICAL)

    def pop_inspection_result(self) -> Optional[str]:
        out = self._pending_inspection_result
        self._pending_inspection_result = None
        return out

    def consume_low_light_event(self) -> bool:
        if self._low_light_event:
            self._low_light_event = False
            return True
        return False

    # ----------------------------------------------------------------------
    # Spawn offset helpers
    # ----------------------------------------------------------------------

    def _launch_spawn_offset_capture(self) -> None:
        if self._spawn_sync_state is SpawnSyncState.READY:
            return
        if self._spawn_capture_thread and self._spawn_capture_thread.is_alive():
            return

        def worker():
            success = self._capture_spawn_offset_from_gazebo()
            self._set_spawn_sync_state(
                SpawnSyncState.READY if success else SpawnSyncState.FAILED
            )
            self._spawn_capture_thread = None

        self._set_spawn_sync_state(SpawnSyncState.QUERYING)
        self._spawn_capture_thread = threading.Thread(
            target=worker,
            name="SpawnOffsetFetcher",
            daemon=True,
        )
        self._spawn_capture_thread.start()

    def _set_spawn_sync_state(self, state: SpawnSyncState) -> None:
        self._spawn_sync_state = state

        if state in (SpawnSyncState.READY, SpawnSyncState.FAILED):
            self._spawn_ready_event.set()
            self._destroy_gazebo_subscription()
        else:
            self._spawn_ready_event.clear()

    def _capture_spawn_offset_from_gazebo(self) -> bool:
        model = (self._gazebo_model or "").strip()
        if not model:
            return False

        for _ in range(10):
            for svc_type, svc_name in [
                ("entity", "/gazebo/get_entity_state"),
                ("model", "/gazebo/get_model_state"),
            ]:
                offset = self._query_gazebo_spawn(svc_name, svc_type, model)
                if offset is not None:
                    self._spawn_offset_enu = offset
                    self._spawn_offset_from_gazebo = True
                    self._node.get_logger().info(
                        "Spawn offset from Gazebo ENU = (%.2f, %.2f, %.2f)"
                        % offset
                    )
                    return True

            if not self._node.context.ok():
                return False
            time.sleep(1.0)

        self._node.get_logger().warn(
            f"Could not fetch spawn offset for '{model}' — falling back to PX4 odom"
        )
        return False

    def _query_gazebo_spawn(
        self, service_name: str, service_type: str, model_name: str
    ) -> Optional[tuple[float, float, float]]:
        """Queries Gazebo services for model position."""
        client = temp_node = None

        try:
            temp_node = rclpy.create_node(
                "telemetry_gazebo_query", context=self._node.context
            )

            if service_type == "entity":
                client = temp_node.create_client(GetEntityState, service_name)
                request = GetEntityState.Request()
                request.name = model_name
                request.reference_frame = "world"
                pose_accessor = lambda r: r.state.pose
            else:
                client = temp_node.create_client(GetModelState, service_name)
                request = GetModelState.Request()
                request.model_name = model_name
                request.relative_entity_name = "world"
                pose_accessor = lambda r: r.pose

            if not client.wait_for_service(timeout_sec=3.0):
                return None

            fut = client.call_async(request)
            rclpy.spin_until_future_complete(temp_node, fut, timeout_sec=3.0)
            if not fut.done():
                return None

            res = fut.result()
            if not res or (hasattr(res, "success") and not res.success):
                return None

            pose = pose_accessor(res)
            return (
                pose.position.x,
                pose.position.y,
                pose.position.z,
            )

        except Exception:
            return None

        finally:
            if client and temp_node:
                try:
                    temp_node.destroy_client(client)
                except Exception:
                    pass
            if temp_node:
                try:
                    temp_node.destroy_node()
                except Exception:
                    pass

    def _destroy_gazebo_subscription(self) -> None:
        if self._gazebo_sub:
            try:
                self._node.destroy_subscription(self._gazebo_sub)
            except Exception:
                pass
            self._gazebo_sub = None

    @staticmethod
    def _model_pose_ready(pose) -> bool:
        try:
            vals = (
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        except AttributeError:
            return False

        if not all(math.isfinite(v) for v in vals):
            return False

        # Orientation must be non-zero quaternion
        qmag = math.sqrt(
            pose.orientation.x**2 +
            pose.orientation.y**2 +
            pose.orientation.z**2 +
            pose.orientation.w**2
        )
        return qmag > 0.0
