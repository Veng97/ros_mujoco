import os

import launch
import launch.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription(
        [
            launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_mujoco"), "launch", "mujoco.launch.py"))),
            launch_testing.actions.ReadyToTest(),
        ]
    )
