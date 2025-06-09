import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


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
            # Launch depth image processing and point cloud generation
            ComposableNodeContainer(
                name="nodelet_manager",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="depth_image_proc",
                        plugin="depth_image_proc::PointCloudXyzrgbNode",
                        name="point_cloud_xyzrgb",
                        remappings=[
                            ("rgb/camera_info", "/image/camera_info"),
                            ("rgb/image_rect_color", "/image/color"),
                            ("depth_registered/image_rect", "/image/depth"),
                            ("points", "/image/points"),
                        ],
                    ),
                ],
                output="screen",
            ),
            # Launch RViz2 with a predefined configuration
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", os.path.join(get_package_share_directory("ros_mujoco_examples"), "config", "default.rviz")],
            ),
        ]
    )
