"""Start the learned driver in place of lidar_cruise_node.py.

Use together with laksa_bringup/manual_control.launch.py (joy + drive_supervisor).
Do not combine with laksa_system.launch.py enable_autonomy:=true, which starts
the rollout LiDAR Cruise node on the same command topic.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("laksa_learned_driver"))
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(share / "config" / "learned_driver.yaml")),
        DeclareLaunchArgument("model_path", default_value=str(share / "models" / "laksa_tinylidarnet_v2.npz")),
        Node(
            package="laksa_learned_driver",
            executable="learned_driver_node",
            name="learned_driver",
            output="screen",
            parameters=[LaunchConfiguration("config"), {"model_path": LaunchConfiguration("model_path")}],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
