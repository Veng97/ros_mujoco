import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=os.path.join(get_package_share_directory("ros_mujoco"), "models", "default.xml"),
                description="Path to the XML model file for MuJoCo simulation",
            ),
            ExecuteProcess(
                cmd=[os.path.join(get_package_share_directory("ros_mujoco"), "mujoco", "simulate"), LaunchConfiguration("model")],
                output="screen",
                name="mujoco",
            ),
        ]
    )
