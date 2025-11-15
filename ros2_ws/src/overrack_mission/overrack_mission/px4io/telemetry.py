"""Telemetry helpers wrapping PX4 topics and inspection events."""

from __future__ import annotations

import time
import math
import threading
from typing import Optional
from enum import Enum, auto
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import rclpy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetEntityState, GetModelState
from std_msgs.msg import String

from px4_msgs.msg import BatteryStatus, VehicleLocalPosition, VehicleStatus

from ..core.fsm import LocalPosition
from .qos import EVENTS_QOS, TELEMETRY_QOS


class SpawnSyncState(Enum):
    UNREQUESTED = auto()
    QUERYING = auto()
    READY = auto()
    FAILED = auto()


class Telemetry:
    """Collects PX4 telemetry topics and exposes convenience accessors."""

    def __init__(self, node, *, gazebo_model: str = "iris_opt_flow") -> None:
        self._node = node
        self._local_position: Optional[LocalPosition] = None
        self._spawn_origin: Optional[LocalPosition] = None
        self._spawn_offset_enu: Optional[tuple[float, float, float]] = None
        self._gazebo_model = gazebo_model
        self._vehicle_status: Optional[VehicleStatus] = None
        self._battery_status: Optional[BatteryStatus] = None
        self._pending_inspection_result: Optional[str] = None
        self._low_light_event = False
        self._first_odom_logged = False
        self._last_wait_log_s: Optional[float] = None
        self._gazebo_sub = None
        self._gazebo_model_detected = False
        self._gazebo_pose_wait_logged = False
        self._spawn_capture_thread: Optional[threading.Thread] = None
        self._spawn_offset_from_gazebo = False
        self._spawn_sync_state = SpawnSyncState.UNREQUESTED
        self._spawn_sync_state_since = time.monotonic()
        self._spawn_ready_event = threading.Event()

     #   self._spawn_offset_enu = (0.0, 0.0, 0.05) Debug
      #  self._spawn_offset_from_gazebo = True 

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
        

    

        gazebo_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,  # << CAMBIA QUI
        )


        self._gazebo_sub = node.create_subscription(
            ModelStates,
            "/gazebo/model_states",
            self._on_gazebo_model_states,
            gazebo_qos,
        )



    # ------------------------------------------------------------------
    # Callbacks

    def _on_local_position(self, msg: VehicleLocalPosition) -> None:
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
        if self._spawn_origin is None:
            self._spawn_origin = new_pose
            if self._spawn_offset_enu is None:
                self._spawn_offset_enu = (new_pose.y, new_pose.x, -new_pose.z)
                self._node.get_logger().info(
                    "Spawn offset locked at ENU (x=%.2f, y=%.2f, z=%.2f)"
                    % self._spawn_offset_enu
                )
            self._node.get_logger().info(
                "Spawn origin locked at NED (x=%.2f, y=%.2f, z=%.2f)"
                % (msg.x, msg.y, msg.z)
            )
        if not self._first_odom_logged:
            self._first_odom_logged = True
            self._node.get_logger().info(
                "Odometry received: x=%.2f y=%.2f z=%.2f"
                % (msg.x, msg.y, msg.z)
            )

    def _on_vehicle_status(self, msg: VehicleStatus) -> None:
        previous = self._vehicle_status
        self._vehicle_status = msg
        if previous is None or previous.nav_state != msg.nav_state:
            if self.is_offboard_active():
                self._node.get_logger().info("OFFBOARD mode active (ACK)") # ELSE?
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
            self._node.get_logger().warn(f"Unknown inspection event payload: {payload!r}")

    def _on_gazebo_model_states(self, msg: ModelStates) -> None:
        self._node.get_logger().info(
            f"[TRACE] _on_gazebo_model_states() called, current spawn_offset_enu={self._spawn_offset_enu}"
        )

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
                f"Gazebo model '{model_name}' detected — waiting for pose before capturing spawn offset..."
            )
            self._gazebo_model_detected = True
        if index >= len(msg.pose):
            return
        pose = msg.pose[index]
        if not self._model_pose_ready(pose):
            if not self._gazebo_pose_wait_logged:
                self._node.get_logger().debug(
                    f"Gazebo model '{model_name}' pose not ready; deferring spawn offset capture"
                )
                self._gazebo_pose_wait_logged = True
            return
        self._node.get_logger().info(
            f"Gazebo model '{model_name}' pose ready — capturing spawn offset..."
        )
        self._launch_spawn_offset_capture()

    # ------------------------------------------------------------------
    # Accessors

    def local_position(self) -> Optional[LocalPosition]:
        return self._local_position

    def spawn_origin(self) -> Optional[tuple[float, float, float]]:
        origin = self._spawn_origin
        if origin is None:
            return None
        return (origin.x, origin.y, origin.z)

    def spawn_offset_enu(self) -> Optional[tuple[float, float, float]]:
        return self._spawn_offset_enu

    def spawn_offset_from_gazebo(self) -> bool:
        return self._spawn_offset_from_gazebo

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
        status = self._vehicle_status
        return bool(status and status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def is_armed(self) -> bool:
        status = self._vehicle_status
        return bool(status and status.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def data_link_ok(self) -> bool:
        status = self._vehicle_status
        if status is None:
            return True
        # Prefer data_link_lost if available, fall back to rc_signal_lost
        if hasattr(status, "data_link_lost"):
            return not bool(status.data_link_lost)
        if hasattr(status, "rc_signal_lost"):
            return not bool(status.rc_signal_lost)
        return True

    def preflight_checks_pass(self) -> bool:
        status = self._vehicle_status
        return bool(status and status.pre_flight_checks_pass)

    def battery_warning(self) -> bool:
        status = self._battery_status
        if status is None:
            return False
        return status.warning >= BatteryStatus.BATTERY_WARNING_WARNING

    def battery_critical(self) -> bool:
        status = self._battery_status
        if status is None:
            return False
        return status.warning >= BatteryStatus.BATTERY_WARNING_CRITICAL

    def pop_inspection_result(self) -> Optional[str]:
        result = self._pending_inspection_result
        self._pending_inspection_result = None
        return result

    def consume_low_light_event(self) -> bool:
        if self._low_light_event:
            self._low_light_event = False
            return True
        return False

    def log_waiting_preflight(self) -> None:
        now_s = self._node.get_clock().now().nanoseconds / 1e9
        if self._last_wait_log_s is None or (now_s - self._last_wait_log_s) > 1.0:
            self._node.get_logger().info("Waiting for preflight checks to pass before arming...")
            self._last_wait_log_s = now_s

    # ------------------------------------------------------------------
    # Spawn offset helpers

    def _capture_spawn_offset_from_gazebo(self) -> bool:
        model = (self._gazebo_model or "").strip()
        self._node.get_logger().info(
            f"Attempting to capture spawn offset from Gazebo for model '{model}'..."
        )
        self._node.get_logger().info("[DEBUG] Entered _capture_spawn_offset_from_gazebo() loop")

        if not model:
            self._node.get_logger().info("No Gazebo model name specified; cannot capture spawn offset.")
            return False
        service_specs = [
            ("entity", "/gazebo/get_entity_state"),
            ("model", "/gazebo/get_model_state"),
        ]
        for _ in range(10):
            for svc_type, svc_name in service_specs:
                self._node.get_logger().debug(
                    "Querying Gazebo service %s for model '%s' spawn offset"
                    % (svc_name, model)
                )
                offset = self._query_gazebo_spawn(svc_name, svc_type, model)
                if offset is not None:
                    self._spawn_offset_enu = offset
                    self._spawn_offset_from_gazebo = True
                    self._node.get_logger().info(
                        "Spawn offset fetched from Gazebo (%s): ENU (x=%.2f, y=%.2f, z=%.2f)"
                        % (svc_name, offset[0], offset[1], offset[2])
                    )
                    self._node.get_logger().info(f"[DEBUG] Spawn offset synced: Gazebo=({offset[0]:.2f}, {offset[1]:.2f}, {offset[2]:.2f})")

                    return True
            if not self._node.context.ok():
                return False
            time.sleep(1.0)
        self._node.get_logger().warn(
            "Unable to query Gazebo for model '%s'; awaiting PX4 odometry to lock spawn offset"
            % model
        )
        return False

    def _query_gazebo_spawn(
        self, service_name: str, service_type: str, model_name: str
    ) -> Optional[tuple[float, float, float]]:
        client = None
        temp_node = None
        try:
            temp_node = rclpy.create_node(
                "telemetry_gazebo_query",
                context=self._node.context,
            )
            if service_type == "entity":
                self._node.get_logger().info(f"[DEBUG] Creating client1 for {service_name}")
                client = temp_node.create_client(GetEntityState, service_name)
                request = GetEntityState.Request()
                request.name = model_name
                request.reference_frame = "world"
                pose_accessor = lambda response: response.state.pose
            else:
                self._node.get_logger().info(f"[DEBUG] Creating client2 for {service_name}")
                client = temp_node.create_client(GetModelState, service_name)
                request = GetModelState.Request()
                request.model_name = model_name
                request.relative_entity_name = "world"
                pose_accessor = lambda response: response.pose

            if not client.wait_for_service(timeout_sec=3.0):
                self._node.get_logger().debug(f"{service_name} not ready yet")
                return None

            future = client.call_async(request)
            rclpy.spin_until_future_complete(temp_node, future, timeout_sec=3.0)
            if not future.done():
                return None
            response = future.result()
            if response is None:
                return None
            if hasattr(response, "success") and not response.success:
                self._node.get_logger().debug(
                    "%s returned success=False for model '%s'" % (service_name, model_name)
                )
                return None
            pose = pose_accessor(response)
            return (pose.position.x, pose.position.y, pose.position.z)
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().debug(
                "Gazebo service %s query failed: %s" % (service_name, exc)
            )
            return None
        finally:
            if client is not None and temp_node is not None:
                try:
                    temp_node.destroy_client(client)
                except Exception:  # noqa: BLE001
                    pass
            if temp_node is not None:
                try:
                    temp_node.destroy_node()
                except Exception:  # noqa: BLE001
                    pass

    def _destroy_gazebo_subscription(self) -> None:
        gazebo_sub = self._gazebo_sub
        if gazebo_sub is None:
            return
        try:
            self._node.destroy_subscription(gazebo_sub)
        except Exception:  # noqa: BLE001
            pass
        self._gazebo_sub = None

    @staticmethod
    def _model_pose_ready(pose) -> bool:
        try:
            position_values = (
                pose.position.x,
                pose.position.y,
                pose.position.z,
            )
            orientation_values = (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        except AttributeError:
            return False
        if not all(math.isfinite(v) for v in position_values + orientation_values):
            return False
        orientation_mag = math.sqrt(sum(v * v for v in orientation_values))
        return orientation_mag > 0.0

    def _launch_spawn_offset_capture(self) -> None:
        thread = self._spawn_capture_thread
        if self._spawn_sync_state is SpawnSyncState.READY:
            return
        if thread is not None and thread.is_alive():
            return

        def _worker() -> None:
            try:
                success = self._capture_spawn_offset_from_gazebo()
                if success:
                    self._set_spawn_sync_state(SpawnSyncState.READY)
                else:
                    self._set_spawn_sync_state(SpawnSyncState.FAILED)
            finally:
                self._spawn_capture_thread = None

        self._node.get_logger().info("Launching async spawn offset fetch...")
        self._set_spawn_sync_state(SpawnSyncState.QUERYING)
        self._spawn_capture_thread = threading.Thread(
            target=_worker,
            name="SpawnOffsetFetcher",
            daemon=True,
        )
        self._spawn_capture_thread.start()

    def _set_spawn_sync_state(self, state: SpawnSyncState) -> None:
        self._spawn_sync_state = state
        self._spawn_sync_state_since = time.monotonic()
        if state in (SpawnSyncState.READY, SpawnSyncState.FAILED):
            self._spawn_ready_event.set()
            self._destroy_gazebo_subscription()
        else:
            self._spawn_ready_event.clear()
