#!/usr/bin/env python3
"""Mission runner that publishes PX4 offboard setpoints from a YAML plan."""

from __future__ import annotations

import math
import pathlib
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
import yaml

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

_MICROSECONDS = 1_000_000


@dataclass
class MissionPlan:
    altitude_m: float
    hover_time_s: float
    cruise_speed_mps: float
    waypoints: List[Tuple[float, float]]

    @staticmethod
    def from_yaml(path: pathlib.Path) -> "MissionPlan":
        data = yaml.safe_load(path.read_text()) or {}
        altitude = float(data.get("altitude_m", 2.5))
        hover_time = float(data.get("hover_time_s", 3.0))
        cruise_speed = float(data.get("cruise_speed_mps", 1.0))
        raw_waypoints: Sequence[Sequence[float]] = data.get("waypoints", ((0.0, 0.0),))
        waypoints = []
        for idx, wp in enumerate(raw_waypoints):
            if not isinstance(wp, Sequence) or len(wp) < 2:
                raise ValueError(f"Waypoint #{idx} must be a sequence [x, y], got: {wp!r}")
            try:
                x_val = float(wp[0])
                y_val = float(wp[1])
            except (TypeError, ValueError) as exc:
                raise ValueError(f"Waypoint #{idx} contains non numeric values: {wp!r}") from exc
            waypoints.append((x_val, y_val))
        if not waypoints:
            raise ValueError("Mission requires at least one waypoint")
        return MissionPlan(altitude, hover_time, cruise_speed, waypoints)


class MissionRunner(Node):
    """Publish position setpoints that follow a simple waypoint mission."""

    def __init__(self) -> None:
        super().__init__("mission_runner")

        mission_file_param = self.declare_parameter("mission_file", "").get_parameter_value().string_value
        mission_path = self._resolve_mission_path(mission_file_param)
        if mission_path is None:
            raise RuntimeError("Parameter 'mission_file' must point to a mission YAML file")
        self.get_logger().info(f"Loading mission from {mission_path}")
        self._mission = MissionPlan.from_yaml(mission_path)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._offboard_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos)
        self._trajectory_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos)
        self._vehicle_command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos)

        self._local_position: Optional[VehicleLocalPosition] = None
        self._vehicle_status: Optional[VehicleStatus] = None
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._on_local_position,
            qos,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self._on_vehicle_status,
            qos,
        )

        self._offboard_setpoint_counter = 0
        self._offboard_mode_requested = False
        self._armed_request_sent = False
        self._arm_request_time: Optional[Time] = None
        self._last_wait_log: Optional[Time] = None
        self._offboard_was_active = False
        self._armed_was_true = False
        self._first_odom_logged = False
        self._bootstrap_setpoint_logs = 0

        self._current_index = 0
        self._hover_deadline: Optional[Time] = None
        self._mission_complete = False

        self._timer = self.create_timer(0.05, self._on_timer)  # 20 Hz updates keep OFFBOARD happy
        self.get_logger().info(
            "Mission ready: %d waypoints, altitude %.2f m, hover %.1f s"
            % (len(self._mission.waypoints), self._mission.altitude_m, self._mission.hover_time_s)
        )
        # Initial state hint: TAKEOFF towards z=-altitude at current xy
        initial_target = (0.0, 0.0, -abs(self._mission.altitude_m))
        self.get_logger().info(
            f"FSM state -> TAKEOFF, target=({_format_tuple(initial_target)})"
        )

    def _resolve_mission_path(self, value: str) -> Optional[pathlib.Path]:
        if not value:
            return None
        path = pathlib.Path(value.replace("~", str(pathlib.Path.home()))).expanduser()
        if not path.is_absolute():
            path = (pathlib.Path.cwd() / path).resolve()
        return path if path.is_file() else None

    def _on_local_position(self, msg: VehicleLocalPosition) -> None:
        self._local_position = msg
        if not self._first_odom_logged:
            self._first_odom_logged = True
            self.get_logger().info(
                f"Odometry received: x={msg.x:.2f} y={msg.y:.2f} z={msg.z:.2f}"
            )

    def _on_vehicle_status(self, msg: VehicleStatus) -> None:
        self._vehicle_status = msg
        # Log OFFBOARD acceptance
        active = self._offboard_active()
        if active and not self._offboard_was_active:
            self.get_logger().info("OFFBOARD mode active (ACK)")
        self._offboard_was_active = active

        # Log arming acceptance or refusal
        armed_now = self._armed()
        if armed_now and not self._armed_was_true:
            self.get_logger().info("ARMED (ACK)")
        self._armed_was_true = armed_now

        # Detect preflight issues after arming request
        if self._arm_request_time is not None and not armed_now:
            if not msg.pre_flight_checks_pass:
                self.get_logger().warn("Preflight checks failing; arming blocked")
            # Provide extra context useful in SITL
            if not msg.power_input_valid:
                self.get_logger().warn("Power input invalid (system power unavailable)")
            self.get_logger().warn(
                f"Arming not accepted (arming_state={msg.arming_state}, latest_arming_reason={msg.latest_arming_reason})"
            )

    # ----------------------------------------------------------------------------
    # PX4 command helpers

    def _publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp = self._now_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self._offboard_pub.publish(msg)

    def _publish_trajectory_setpoint(self, target: Tuple[float, float, float], yaw: float = 0.0) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp = self._now_us()
        msg.position = list(target)
        msg.yaw = yaw
        self._trajectory_pub.publish(msg)
        # Debug: during bootstrap phase, confirm setpoint stream
        if self._bootstrap_setpoint_logs < 20:
            self._bootstrap_setpoint_logs += 1
            self.get_logger().info(
                f"Bootstrap setpoint {self._bootstrap_setpoint_logs}/20 -> ({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f})"
            )

    def _publish_vehicle_command(self, command: int, *, param1: float = 0.0, param2: float = 0.0) -> None:
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

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    # ----------------------------------------------------------------------------
    # Mission flow

    @property
    def _target_position(self) -> Tuple[float, float, float]:
        x, y = self._mission.waypoints[self._current_index]
        z = -abs(self._mission.altitude_m)
        return (x, y, z)

    def _offboard_active(self) -> bool:
        if self._vehicle_status is None:
            return False
        return self._vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def _armed(self) -> bool:
        if self._vehicle_status is None:
            return False
        return self._vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def _on_timer(self) -> None:
        self._publish_offboard_control_mode()
        self._publish_trajectory_setpoint(self._target_position)

        if self._offboard_setpoint_counter < 20:
            self._offboard_setpoint_counter += 1
            return

        if not self._offboard_mode_requested:
            self.get_logger().info("OFFBOARD mode command sent")
            self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self._offboard_mode_requested = True
            return

        if not self._armed_request_sent:
            # Wait until OFFBOARD is active and preflight checks pass
            if not self._offboard_active():
                return
            if self._vehicle_status is None or not self._vehicle_status.pre_flight_checks_pass:
                now = self.get_clock().now()
                if self._last_wait_log is None or (now - self._last_wait_log) > Duration(seconds=1.0):
                    self.get_logger().info("Waiting for preflight checks to pass before arming...")
                    self._last_wait_log = now
                return
            self.get_logger().info("ARM command sent")
            self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self._armed_request_sent = True
            self._arm_request_time = self.get_clock().now()
            return

        if not self._offboard_active():
            return

        if self._mission_complete:
            return

        if self._local_position is None:
            return

        # Evaluate proximity to current waypoint
        target = self._target_position
        dx = target[0] - self._local_position.x
        dy = target[1] - self._local_position.y
        dz = target[2] - self._local_position.z
        distance_xy = math.hypot(dx, dy)
        altitude_error = abs(dz)

        tolerance_xy = max(0.25, self._mission.cruise_speed_mps * 0.25)
        tolerance_z = 0.3

        now = self.get_clock().now()

        if distance_xy < tolerance_xy and altitude_error < tolerance_z:
            if self._hover_deadline is None:
                self._hover_deadline = now + Duration(seconds=self._mission.hover_time_s)
                self.get_logger().info(
                    f"FSM state -> HOVER, wp_idx={self._current_index}, target=({_format_tuple(self._target_position)})"
                )
                self.get_logger().info(
                    f"Waypoint {self._current_index + 1}/{len(self._mission.waypoints)} reached. Hovering until {self._hover_deadline.nanoseconds / 1e9:.1f}s"
                )
            elif now >= self._hover_deadline:
                self._advance_waypoint()
        else:
            self._hover_deadline = None

    def _advance_waypoint(self) -> None:
        if self._current_index < len(self._mission.waypoints) - 1:
            self._current_index += 1
            self._hover_deadline = None
            next_wp = self._mission.waypoints[self._current_index]
            self.get_logger().info(
                f"FSM state -> TRANSIT, wp_idx={self._current_index}, target=({_format_tuple(self._target_position)})"
            )
            self.get_logger().info(f"Proceeding to waypoint {self._current_index + 1}: {next_wp}")
        else:
            self._mission_complete = True
            self.get_logger().info("FSM state -> HOLD, mission complete. Holding last setpoint (hover)")


def _format_tuple(t: Tuple[float, float, float]) -> str:
    return f"{t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f}"


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)

    node: Optional[MissionRunner] = None
    try:
        node = MissionRunner()
        rclpy.spin(node)
    except Exception as exc:  # noqa: BLE001
        if node is not None:
            node.get_logger().error(f"Mission runner failed: {exc}")
        else:
            print(f"Mission runner failed: {exc}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
