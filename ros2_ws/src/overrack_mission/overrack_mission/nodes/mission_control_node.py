#!/usr/bin/env python3
"""Mission runner node that delegates execution to the core FSM."""

from __future__ import annotations

import pathlib
from typing import Optional, Sequence, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from ..core import (
    AxisBounds,
    MissionController,
    MissionPlanError,
    NedBounds,
    enu_bounds_from_ned,
)
from ..core.fsm import MissionRuntime
from ..px4io import SetpointPublisher, Telemetry
from ..px4io.qos import EVENTS_QOS
from .inspection_node import InspectionNode


class RosMissionRuntime(MissionRuntime):
    """Concrete MissionRuntime backed by rclpy."""

    def __init__(self, node: Node, telemetry: Telemetry, setpoints: SetpointPublisher) -> None:
        self._node = node
        self._telemetry = telemetry
        self._setpoints = setpoints
        self._state_pub = node.create_publisher(String, "overrack/mission_state", EVENTS_QOS)

    @property
    def logger(self):
        return self._node.get_logger()

    def now(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9

    def publish_state(self, name: str) -> None:
        msg = String()
        msg.data = name
        self._state_pub.publish(msg)

    @property
    def telemetry(self) -> Telemetry:
        return self._telemetry

    @property
    def setpoints(self) -> SetpointPublisher:
        return self._setpoints


class MissionControlNode(Node):
    """ROS2 node that loads a mission file and executes it through the core controller."""

    def __init__(self) -> None:
        super().__init__("mission_runner")
        mission_file_param = self.declare_parameter("mission_file", "").get_parameter_value().string_value
        mission_path = self._resolve_mission_path(mission_file_param)
        if mission_path is None:
            raise RuntimeError("Parameter 'mission_file' must point to a mission YAML file")
        self.get_logger().info(f"Loading mission from {mission_path}")

        ned_bounds = self._load_world_bounds()
        cruise_limits = self._load_cruise_speed_limits()
        debug_frames = bool(self.declare_parameter("debug_frames", False).value)

        gazebo_model = (
            self.declare_parameter("gazebo_model_name", "iris_opt_flow").get_parameter_value().string_value
        )
        telemetry = Telemetry(self, gazebo_model=gazebo_model)
        setpoints = SetpointPublisher(self, telemetry, ned_bounds)
        runtime = RosMissionRuntime(self, telemetry, setpoints)

        try:
            self._controller = MissionController(
                runtime,
                mission_path,
                enu_bounds=enu_bounds_from_ned(ned_bounds),
                cruise_speed_limits=cruise_limits,
                debug_frames=debug_frames,
            )
        except MissionPlanError as exc:
            raise RuntimeError(f"Failed to load mission: {exc}") from exc

        plan = self._controller.plan
        self.get_logger().info(
            "Mission ready: %d waypoints, altitude %.2f m, hover %.1f s"
            % (len(plan.waypoints), plan.altitude_m, plan.hover_time_s)
        )
        initial_target = (0.0, 0.0, plan.altitude_m)
        self.get_logger().info(
            "FSM state -> TAKEOFF (ENU target=(%.2f, %.2f, %.2f))"
            % initial_target
        )

        # 20 Hz keep-offboard timer (matches previous behaviour)
        self._timer = self.create_timer(0.025, self._on_timer) #0.05 = 20Hz

    def _resolve_mission_path(self, value: str) -> Optional[pathlib.Path]:
        if not value:
            return None
        path = pathlib.Path(value.replace("~", str(pathlib.Path.home()))).expanduser()
        if not path.is_absolute():
            path = (pathlib.Path.cwd() / path).resolve()
        return path if path.is_file() else None

    def _on_timer(self) -> None:
        self._controller.tick()

    # ------------------------------------------------------------------
    # Parameter helpers

    def _load_world_bounds(self) -> NedBounds:
        x_min, x_max = self._declare_bounds_pair("world_bounds.x", [-5.8, 5.8])
        y_min, y_max = self._declare_bounds_pair("world_bounds.y", [-4.8, 4.8])
        z_min, z_max = self._declare_bounds_pair("world_bounds.z", [-3.0, -0.1])
        return NedBounds(
            AxisBounds(x_min, x_max),
            AxisBounds(y_min, y_max),
            AxisBounds(z_min, z_max),
        )

    def _load_cruise_speed_limits(self) -> Tuple[float, float]:
        return self._declare_bounds_pair("cruise_speed_limits", [0.2, 2.0])

    def _declare_bounds_pair(self, name: str, default: list[float]) -> Tuple[float, float]:
        param = self.declare_parameter(name, default)
        values = list(param.value) if isinstance(param.value, (list, tuple)) else []
        if len(values) != 2:
            raise RuntimeError(f"Parameter '{name}' must be a sequence [min, max]")
        low = float(values[0])
        high = float(values[1])
        if low > high:
            low, high = high, low
        return (low, high)


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)

    runner: Optional[MissionControlNode] = None
    inspection: Optional[InspectionNode] = None
    executor: Optional[MultiThreadedExecutor] = None
    try:
        runner = MissionControlNode()
        executor = MultiThreadedExecutor()
        executor.add_node(runner)

        runner.get_logger().info("Starting inspection node")
        inspection = InspectionNode()
        executor.add_node(inspection)

        image_topic = inspection.get_parameter("image_topic").get_parameter_value().string_value
        runner.get_logger().info(
            "Inspection node attached (image_topic=%s)"
            % image_topic
        )

        executor.spin()
    except Exception as exc:  # noqa: BLE001
        if runner is not None:
            runner.get_logger().error(f"Mission runner failed: {exc}")
        else:
            print(f"Mission runner failed: {exc}")
        raise
    finally:
        if executor is not None:
            if runner is not None:
                try:
                    executor.remove_node(runner)
                except Exception:  # noqa: BLE001
                    pass
            if inspection is not None:
                try:
                    executor.remove_node(inspection)
                except Exception:  # noqa: BLE001
                    pass
            executor.shutdown()
        if inspection is not None:
            inspection.destroy_node()
        if runner is not None:
            runner.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
