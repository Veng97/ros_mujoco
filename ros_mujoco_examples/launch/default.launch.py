import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=os.path.join(get_package_share_directory("ros_mujoco_examples"), "models", "default.xml"),
                description="Path to the XML model file for MuJoCo simulation",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="True",
                description="Run MuJoCo in headless mode (no GUI)",
            ),
            # Launch the MuJoCo simulation
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_mujoco"), "launch", "mujoco.launch.py")),
                launch_arguments={
                    "model": LaunchConfiguration("model"),
                    "headless": LaunchConfiguration("headless"),
                }.items(),
            ),
        ]
    )
