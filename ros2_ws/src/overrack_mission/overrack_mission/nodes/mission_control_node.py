#!/usr/bin/env python3
"""Mission runner node that delegates execution to the core FSM."""

from __future__ import annotations

import pathlib
from typing import Optional, Sequence, Tuple

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
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
from ..px4io.topics import namespaced
from ..px4io.qos import EVENTS_QOS


class RosMissionRuntime(MissionRuntime):
    """Concrete MissionRuntime backed by rclpy."""

    def __init__(self, node: Node, telemetry: Telemetry, setpoints: SetpointPublisher, *, state_topic: str) -> None:
        self._node = node
        self._telemetry = telemetry
        self._setpoints = setpoints
        self._state_pub = node.create_publisher(String, state_topic, EVENTS_QOS)

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

        self._vehicle_ns = (
            self.declare_parameter("vehicle_ns", "").get_parameter_value().string_value.strip()
        )
        self._vehicle_id = int(self.declare_parameter("vehicle_id", 1).value)
        if self._vehicle_ns:
            self.get_logger().info(f"Vehicle namespace: {self._vehicle_ns} (id={self._vehicle_id})")
        else:
            self.get_logger().info(f"Vehicle id={self._vehicle_id} (no namespace)")
        self._px4_namespace = (
            self.declare_parameter("px4_namespace", "").get_parameter_value().string_value.strip().strip("/")
        )

        ned_bounds = self._load_world_bounds()
        cruise_limits = self._load_cruise_speed_limits()
        debug_frames = bool(self.declare_parameter("debug_frames", False).value)
        safe_z_param = self.declare_parameter("return_home_safe_z", None)
        return_home_safe_z = safe_z_param.value
        if return_home_safe_z is not None:
            try:
                return_home_safe_z = float(return_home_safe_z)
            except (TypeError, ValueError):
                return_home_safe_z = None

        gazebo_model = (
            self.declare_parameter("gazebo_model_name", "iris_opt_flow").get_parameter_value().string_value
        )
        self._link_loss_after_param = "sim.disable_link_after_s"
        self._link_loss_after_s = float(self.declare_parameter(self._link_loss_after_param, -1.0).value)
        self._link_loss_triggered = False
        self._sim_start_time = self.get_clock().now().nanoseconds / 1e9
        telemetry = Telemetry(
            self,
            gazebo_model=gazebo_model,
            vehicle_ns=self._vehicle_ns,
            px4_namespace=self._px4_namespace,
        )
        setpoints = SetpointPublisher(
            self,
            telemetry,
            ned_bounds,
            vehicle_ns=self._vehicle_ns,
            vehicle_id=self._vehicle_id,
            px4_namespace=self._px4_namespace,
        )
        self._setpoints = setpoints
        self._configure_link_loss_after(self._link_loss_after_s)
        runtime = RosMissionRuntime(
            self,
            telemetry,
            setpoints,
            state_topic=namespaced("overrack/mission_state", namespace=self._vehicle_ns),
        )

        try:
            self._controller = MissionController(
                runtime,
                mission_path,
                enu_bounds=enu_bounds_from_ned(ned_bounds),
                cruise_speed_limits=cruise_limits,
                debug_frames=debug_frames,
                return_home_safe_z=return_home_safe_z,
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
        self.add_on_set_parameters_callback(self._on_parameters_changed)

    def _resolve_mission_path(self, value: str) -> Optional[pathlib.Path]:
        if not value:
            return None
        path = pathlib.Path(value.replace("~", str(pathlib.Path.home()))).expanduser()
        if not path.is_absolute():
            path = (pathlib.Path.cwd() / path).resolve()
        return path if path.is_file() else None

    def _on_timer(self) -> None:
        if (
            self._link_loss_after_s is not None
            and not self._link_loss_triggered
            and not self._setpoints.link_loss_simulated
        ):
            now = self.get_clock().now().nanoseconds / 1e9
            if (now - self._sim_start_time) >= self._link_loss_after_s:
                self.get_logger().warn(
                    "Link-loss simulation auto-triggered after %.1fs"
                    % self._link_loss_after_s
                )
                self._setpoints.set_link_loss_simulation(True)
                self._link_loss_triggered = True
        self._controller.tick()

    def _on_parameters_changed(self, params: list[Parameter]) -> SetParametersResult:
        result = SetParametersResult()
        result.successful = True
        for param in params:
            if param.name == self._link_loss_after_param:
                self._configure_link_loss_after(float(param.value))
        return result

    def _configure_link_loss_after(self, value: float) -> None:
        if value < 0.0:
            self._link_loss_after_s = None
            self._link_loss_triggered = False
            self._setpoints.set_link_loss_simulation(False)
            return
        self._link_loss_after_s = value
        self._link_loss_triggered = False
        self._sim_start_time = self.get_clock().now().nanoseconds / 1e9
        if value == 0.0:
            self.get_logger().warn("Link-loss simulation auto-triggered immediately (disable_link_after_s=0)")
            self._setpoints.set_link_loss_simulation(True)
            self._link_loss_triggered = True

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
    executor: Optional[MultiThreadedExecutor] = None
    try:
        runner = MissionControlNode()
        executor = MultiThreadedExecutor()
        executor.add_node(runner)

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
            executor.shutdown()
        if runner is not None:
            runner.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
