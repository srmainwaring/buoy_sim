# Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Gazebo world with a buoy."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_buoy_gazebo = get_package_share_directory("buoy_gazebo")

    # TODO: not clear how to pass additional launch args when using
    #       PathJoinSubstitution to resolve package paths...
    #
    # world_file_path = PathJoinSubstitution(
    #     [pkg_buoy_gazebo, "worlds", LaunchConfiguration("world_file")]
    # )
    #
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py"),
    #     ),
    #     launch_arguments={"gz_args": world_file_path}.items(),
    # )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py"),
        ),
        launch_arguments={
            "gz_args": "-v4 -s -r "
            # + os.path.join(pkg_buoy_gazebo, "worlds", "buoy_playground.sdf")
            + os.path.join(pkg_buoy_gazebo, "worlds", "mbari_wec.sdf")
        }.items(),
    )

    return LaunchDescription(
        [
            gazebo,
        ]
    )
