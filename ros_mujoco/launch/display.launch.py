import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    point_cloud_node = ComposableNodeContainer(
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
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("ros_mujoco"), "launch", "display.rviz")],
    )

    return LaunchDescription(
        [
            point_cloud_node,
            rviz_node,
        ]
    )
