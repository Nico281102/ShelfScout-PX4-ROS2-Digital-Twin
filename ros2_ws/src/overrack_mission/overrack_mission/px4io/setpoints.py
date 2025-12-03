"""Utilities to publish PX4 setpoints and commands for mission control."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Optional, Sequence, Tuple

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

from ..core.bounds import NedBounds
from .topics import namespaced
from .qos import CONTROL_QOS

if TYPE_CHECKING:
    from .telemetry import Telemetry


class SetpointPublisher:
    """Wraps the publishers used to drive PX4 in offboard mode."""

    def __init__(
        self,
        node,
        telemetry: "Telemetry",
        bounds: Optional[NedBounds] = None,
        *,
        vehicle_ns: str = "",
        vehicle_id: int = 1,
        px4_namespace: str = "",
    ) -> None:
        self._node = node
        self._telemetry = telemetry
        self._bounds = bounds
        self._vehicle_ns = vehicle_ns or ""
        self._vehicle_id = int(vehicle_id) if vehicle_id is not None else 1
        self._px4_ns = (px4_namespace or "").strip("/")
        self._offboard_pub = node.create_publisher(
            OffboardControlMode,
            namespaced("fmu/in/offboard_control_mode", namespace=self._px4_ns),
            CONTROL_QOS,
        )
        self._trajectory_pub = node.create_publisher(
            TrajectorySetpoint,
            namespaced("fmu/in/trajectory_setpoint", namespace=self._px4_ns),
            CONTROL_QOS,
        )
        self._vehicle_command_pub = node.create_publisher(
            VehicleCommand,
            namespaced("fmu/in/vehicle_command", namespace=self._px4_ns),
            CONTROL_QOS,
        )
        self._bootstrap_logs = 0
        self._bootstrap_mode = True
        self._link_loss_simulated = False
        self._fallback_sysid_warned = False
        self._logged_system_id: Optional[int] = None

    def send_offboard_control_mode(self) -> None:
        if self._link_loss_simulated:
            return
        msg = OffboardControlMode()
        msg.timestamp = self._now_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self._offboard_pub.publish(msg)

    def send_trajectory_setpoint(self, enu_position: Sequence[float], yaw_deg: float = 0.0) -> None:
        if self._link_loss_simulated:
            return
        ned_target, yaw_rad, _ = self._prepare_target(enu_position, yaw_deg)
        msg = TrajectorySetpoint()
        msg.timestamp = self._now_us()
        msg.position = list(ned_target)
        msg.yaw = yaw_rad
        self._trajectory_pub.publish(msg)
        if self._bootstrap_logs < 20:
            self._bootstrap_logs += 1
            self._node.get_logger().info(
                "Bootstrap setpoint %d/20 -> ENU(%.2f, %.2f, %.2f) NED(%.2f, %.2f, %.2f)"
                % (
                    self._bootstrap_logs,
                    enu_position[0],
                    enu_position[1],
                    enu_position[2],
                    ned_target[0],
                    ned_target[1],
                    ned_target[2],
                )
            )

    def describe_target(
        self, enu_position: Sequence[float], yaw_deg: float = 0.0
    ) -> tuple[tuple[float, float, float], tuple[float, float, float], float]:
        """Return (target_ned, origin_offset, yaw_rad) for debug logging."""

        ned_target, yaw_rad, origin = self._prepare_target(enu_position, yaw_deg)
        return ned_target, origin, yaw_rad

    def send_vehicle_command(self, command: int, *, param1: float = 0.0, param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp = self._now_us()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        sysid = self._resolve_system_id()
        msg.target_system = sysid
        msg.target_component = 1
        msg.source_system = sysid
        msg.source_component = 1
        msg.from_external = True
        self._vehicle_command_pub.publish(msg)

    def set_link_loss_simulation(self, enabled: bool) -> None:
        enabled = bool(enabled)
        if enabled == self._link_loss_simulated:
            return
        self._link_loss_simulated = enabled
        if enabled:
            self._node.get_logger().warn(
                "Link-loss simulation enabled: Offboard setpoints/heartbeats will not be published"
            )
        else:
            self._node.get_logger().info("Link-loss simulation disabled: resuming Offboard setpoints")

    @property
    def link_loss_simulated(self) -> bool:
        return self._link_loss_simulated

    # ------------------------------------------------------------------
    # Helpers

    def _prepare_target(
        self, enu_position: Sequence[float], yaw_deg: float
    ) -> tuple[tuple[float, float, float], float, tuple[float, float, float]]:
        if len(enu_position) < 3:
            raise ValueError("Setpoint requires [x, y, z] in ENU")
        enu_x, enu_y, enu_z = float(enu_position[0]), float(enu_position[1]), float(enu_position[2])
        offset = (0.0, 0.0, 0.0) if self._bootstrap_mode else self._spawn_offset_enu()
        local_enu = (
            enu_x - offset[0],
            enu_y - offset[1],
            enu_z - offset[2],
        )
        ned = (local_enu[1], local_enu[0], -local_enu[2])
        origin = self._origin_offset()
        adjusted = ned
        if self._bounds is not None:
            adjusted = self._bounds.clamp(adjusted)
        yaw_rad = math.radians(yaw_deg or 0.0)
        return adjusted, yaw_rad, origin

    def _origin_offset(self) -> tuple[float, float, float]:
        offset_enu = self._telemetry.spawn_offset_enu()
        if offset_enu is not None:
            return (offset_enu[1], offset_enu[0], -offset_enu[2])
        origin = self._telemetry.spawn_origin()
        if origin is None:
            return (0.0, 0.0, 0.0)
        return origin

    def _spawn_offset_enu(self) -> tuple[float, float, float]:
        offset = self._telemetry.spawn_offset_enu()
        if offset is None:
            return (0.0, 0.0, 0.0)
        return offset

    def _now_us(self) -> int:
        return int(self._node.get_clock().now().nanoseconds / 1000)

    def _resolve_system_id(self) -> int:
        sysid = self._telemetry.system_id()
        if sysid is not None:
            sysid_int = int(sysid)
            if sysid_int != self._logged_system_id and sysid_int != self._vehicle_id:
                self._node.get_logger().info(
                    "Using PX4 system_id=%d (configured vehicle_id=%d)" % (sysid_int, self._vehicle_id)
                )
            self._logged_system_id = sysid_int
            return sysid_int

        if not self._fallback_sysid_warned:
            self._node.get_logger().warn(
                "PX4 system_id not yet available; using configured vehicle_id=%d for commands"
                % self._vehicle_id
            )
            self._fallback_sysid_warned = True
        return self._vehicle_id

    def set_bootstrap_mode(self, enabled: bool) -> None:
        self._bootstrap_mode = bool(enabled)
