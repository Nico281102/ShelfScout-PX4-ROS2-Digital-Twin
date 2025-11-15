"""Utilities to publish PX4 setpoints and commands for mission control."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Optional, Sequence, Tuple

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

from ..core.bounds import NedBounds
from .qos import CONTROL_QOS

if TYPE_CHECKING:
    from .telemetry import Telemetry


class SetpointPublisher:
    """Wraps the publishers used to drive PX4 in offboard mode."""

    def __init__(self, node, telemetry: "Telemetry", bounds: Optional[NedBounds] = None) -> None:
        self._node = node
        self._telemetry = telemetry
        self._bounds = bounds
        self._offboard_pub = node.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", CONTROL_QOS)
        self._trajectory_pub = node.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", CONTROL_QOS)
        self._vehicle_command_pub = node.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", CONTROL_QOS)
        self._bootstrap_logs = 0
        self._bootstrap_mode = True

    def send_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp = self._now_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self._offboard_pub.publish(msg)

    def send_trajectory_setpoint(self, enu_position: Sequence[float], yaw_deg: float = 0.0) -> None:
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
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._vehicle_command_pub.publish(msg)

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
        return (0.0, 0.0, 0.0)

    def _now_us(self) -> int:
        return int(self._node.get_clock().now().nanoseconds / 1000)
    def set_bootstrap_mode(self, enabled: bool) -> None:
        self._bootstrap_mode = bool(enabled)
