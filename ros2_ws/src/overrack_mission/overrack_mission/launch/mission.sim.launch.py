"""Launch mission runner, inspection, and metrics nodes with shared params."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    return LaunchDescription(
        [declare_mission, declare_params, mission_node, inspection_node, metrics_node]
    )
