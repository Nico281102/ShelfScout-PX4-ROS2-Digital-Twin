"""Launch RTAB-Map with default parameters for indoor mapping."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    frame_id = LaunchConfiguration("frame_id", default="base_link")
    mapping_node = Node(
        package="rtabmap_ros",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[
            {
                "frame_id": frame_id,
                "subscribe_depth": True,
                "subscribe_rgb": True,
                "subscribe_scan": False,
                "approx_sync": True,
                "queue_size": 10,
            }
        ],
    )

    return LaunchDescription(
        [DeclareLaunchArgument("frame_id", default_value="base_link"), mapping_node]
    )
