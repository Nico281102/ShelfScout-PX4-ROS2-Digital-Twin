"""Launch mission runner, inspection, and metrics nodes with shared params."""

from __future__ import annotations

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def _load_yaml(path: Path) -> dict:
    try:
        with path.open("r", encoding="utf-8") as handle:
            return yaml.safe_load(handle) or {}
    except Exception:
        return {}


def _nested(data: dict, keys: list[str]) -> str:
    ref = data
    for key in keys:
        if not isinstance(ref, dict):
            return ""
        ref = ref.get(key)
    return ref if isinstance(ref, str) else ""


def _load_drones(config: dict) -> list[dict]:
    drones = (
        config.get("run_ros2_system", {})
        .get("ros__parameters", {})
        .get("drones_yaml")
        or ""
    )
    # Parse YAML string into list
    if isinstance(drones, str):
        try:
            drones = yaml.safe_load(drones) or []
        except Exception:
            drones = []
    if not isinstance(drones, list):
        return []
    return [d for d in drones if isinstance(d, dict)]


def _default_mission(config: dict) -> str:
    return _nested(config, ["mission_runner", "ros__parameters", "mission_file"])


def _default_model(config: dict) -> str:
    return _nested(config, ["mission_runner", "ros__parameters", "gazebo_model_name"])


def _default_mavlink_url(config: dict) -> str:
    return _nested(config, ["px4_param_setter", "ros__parameters", "mavlink_url"])


def _mission_runner_defaults(config: dict) -> dict:
    params = config.get("mission_runner", {}).get("ros__parameters", {})
    return params if isinstance(params, dict) else {}


def _section_params(config: dict, section: str) -> dict:
    params = config.get(section, {}).get("ros__parameters", {})
    return params if isinstance(params, dict) else {}


SUPPORTED_MULTI_MODELS = {"iris", "plane", "standard_vtol", "rover", "r1_rover", "typhoon_h480", "iris_opt_flow"}


def _resolve_gazebo_model_name(drone: dict, idx: int, model_default: str) -> str:
    """Mirror sitl_multiple_run.sh naming so spawn offset detection matches Gazebo."""
    explicit = str(drone.get("gazebo_model_name") or "").strip()
    if explicit:
        return explicit

    base_model = str(
        drone.get("model")
        or model_default
        or "iris"
    ).strip()
    if not base_model:
        base_model = "iris"

    resolved_model = base_model
    if base_model not in SUPPORTED_MULTI_MODELS:
        # sitl_multiple_run.sh maps unsupported models to iris
        resolved_model = "iris"

    return f"{resolved_model}_{idx}"


def generate_launch_description() -> LaunchDescription:
    pkg_share = Path(get_package_share_directory("overrack_mission"))
    default_params = str(pkg_share / "param" / "sim.yaml")

    mission_file = LaunchConfiguration("mission_file")
    params_file = LaunchConfiguration("params_file")

    declare_mission = DeclareLaunchArgument(
        "mission_file",
        default_value="",
        description="Path to the mission YAML file (overrides params file)",
    )

    declare_params = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="YAML file with mission_runner/inspection_node parameters",
    )

    px4_param_setter = Node(
        package="overrack_mission",
        executable="px4_param_setter",
        name="px4_param_setter",
        output="screen",
        parameters=[params_file],
    )

    mission_node = Node(
        package="overrack_mission",
        executable="mission_runner",
        name="mission_runner",
        output="screen",
        emulate_tty=True,
        parameters=[params_file, {"mission_file": mission_file}],
    )

    inspection_node = Node(
        package="overrack_mission",
        executable="inspection_node",
        name="inspection_node",
        output="screen",
        parameters=[params_file],
    )

    metrics_node = Node(
        package="overrack_mission",
        executable="mission_metrics",
        name="mission_metrics",
        output="screen",
        parameters=[params_file],
    )

    torch_controller = Node(
        package="overrack_mission",
        executable="torch_controller",
        name="torch_controller_default",
        output="screen",
        parameters=[params_file],
    )

    def _launch_setup(context, *args, **kwargs):
        params_path = Path(LaunchConfiguration("params_file").perform(context))
        mission_override = LaunchConfiguration("mission_file").perform(context)
        config = _load_yaml(params_path)
        drones = _load_drones(config)
        mission_default = _default_mission(config)
        model_default = _default_model(config) or "iris_opt_flow"
        mavlink_default = _default_mavlink_url(config) or "udp://:14540"
        mission_runner_defaults = _mission_runner_defaults(config)
        inspection_defaults = _section_params(config, "inspection_node")
        torch_defaults = _section_params(config, "torch_controller")

        # Multi-drone path (>=1 entry). With one entry we still namespaced to keep symmetry.
        if drones:
            actions = [declare_mission, declare_params]
            for idx, drone in enumerate(drones, start=1):
                ns = str(
                    drone.get("namespace")
                    or drone.get("name")
                    or drone.get("vehicle_ns")
                    or f"px4_{idx}"
                ).lstrip("/")
                if not ns:
                    ns = f"px4_{idx}"
                vehicle_id = int(drone.get("vehicle_id") or drone.get("mav_sys_id") or idx)
                px4_ns = str(drone.get("px4_namespace") or "").strip("/").strip()
                drone_runner_cfg = drone.get("mission_runner") if isinstance(drone.get("mission_runner"), dict) else {}
                mission_file = (
                    drone_runner_cfg.get("mission_file")
                    or drone.get("mission_file")
                    or mission_override
                    or mission_default
                )
                mission_value = str(mission_file) if mission_file else ""
                gazebo_model = _resolve_gazebo_model_name(drone, idx, model_default)
                mavlink_url = str(
                    drone.get("mavlink_url")
                    or (f"udp://:{int(drone.get('mavlink_udp_port'))}" if drone.get("mavlink_udp_port") else "")
                    or mavlink_default
                )

                runner_params = dict(mission_runner_defaults)
                runner_params.update(drone_runner_cfg)
                runner_params.update({
                    "vehicle_ns": ns,
                    "vehicle_id": vehicle_id,
                    "gazebo_model_name": gazebo_model,
                    "px4_namespace": px4_ns,
                })
                if mission_value:
                    runner_params["mission_file"] = mission_value

                inspection_params = dict(inspection_defaults)
                drone_insp_cfg = drone.get("inspection_node")
                if isinstance(drone_insp_cfg, dict):
                    inspection_params.update(drone_insp_cfg)
                inspection_params["vehicle_ns"] = ns

                metrics_params = {"vehicle_ns": ns}
                torch_params = dict(torch_defaults)
                drone_torch_cfg = drone.get("torch_controller")
                if isinstance(drone_torch_cfg, dict):
                    torch_params.update(drone_torch_cfg)
                torch_params["vehicle_ns"] = ns

                param_setter_params = {"vehicle_ns": ns, "mavlink_url": mavlink_url, "px4_namespace": px4_ns}
                if isinstance(drone.get("px4_params"), dict):
                    param_setter_params["px4_params"] = drone["px4_params"]

                group = GroupAction(
                    [
                        PushRosNamespace(ns),
                        Node(
                            package="overrack_mission",
                            executable="px4_param_setter",
                            name=f"px4_param_setter_{ns}",
                            output="screen",
                            parameters=[str(params_path), param_setter_params],
                        ),
                        Node(
                            package="overrack_mission",
                            executable="mission_runner",
                            name=f"mission_runner_{ns}",
                            output="screen",
                            emulate_tty=True,
                            parameters=[str(params_path), runner_params],
                        ),
                        Node(
                            package="overrack_mission",
                            executable="inspection_node",
                            name=f"inspection_node_{ns}",
                            output="screen",
                            parameters=[str(params_path), inspection_params],
                        ),
                        Node(
                            package="overrack_mission",
                            executable="mission_metrics",
                            name=f"mission_metrics_{ns}",
                            output="screen",
                            parameters=[str(params_path), metrics_params],
                        ),
                        Node(
                            package="overrack_mission",
                            executable="torch_controller",
                            name=f"torch_controller_{ns}",
                            output="screen",
                            parameters=[str(params_path), torch_params],
                        ),
                    ]
                )
                actions.append(group)
            return actions

        # Legacy single-drone path (no 'drones' list).
        if mission_override:
            mission_node.parameters = [params_file, {"mission_file": mission_override}]
        else:
            mission_node.parameters = [params_file]

        return [
            declare_mission,
            declare_params,
            px4_param_setter,
            mission_node,
            inspection_node,
            metrics_node,
            torch_controller,
        ]

    return LaunchDescription([OpaqueFunction(function=_launch_setup)])
