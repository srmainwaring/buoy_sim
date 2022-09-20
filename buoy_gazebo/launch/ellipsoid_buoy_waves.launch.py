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

"""Launch Gazebo world with an ellipsoid buoy."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_buoy_gazebo = get_package_share_directory("buoy_gazebo")
    pkg_buoy_description = get_package_share_directory("buoy_description")
    model_dir = "wec_sim_ellipsoid"
    model_name = "wec_sim_ellipsoid"

    sdf_file = os.path.join(pkg_buoy_gazebo, "models", model_dir, "model.sdf")
    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()

    # assumes we have generated a urdf file
    urdf_file = os.path.join(pkg_buoy_gazebo, "models", model_dir, "model.sdf.urdf")
    with open(urdf_file, "r") as infp:
        robot_desc = infp.read()


    gazebo_world_file_launch_arg = DeclareLaunchArgument(
        "world_file",
        default_value=["ellipsoid_buoy_waves.sdf"],
        description="Gazebo world filename.sdf",
    )

    gazebo_world_name_launch_arg = DeclareLaunchArgument(
        "world_name", default_value=["world_demo"], description="Gazebo <world name>"
    )

    rviz_launch_arg = DeclareLaunchArgument(
        "rviz", default_value="false", description="Open RViz."
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py"),
        ),
        launch_arguments={"gz_args": "-v4 -s ellipsoid_buoy_waves.sdf"}.items(),
        # launch_arguments={
        #     "gz_args": PathJoinSubstitution(
        #         [pkg_buoy_gazebo, "worlds", LaunchConfiguration("world_file")]
        #     )
        # }.items(),
    )

    # Bridge to forward tf and joint states to ros2
    # LaunchConfiguration("world_name"),
    joint_state_gz_topic = (
        "/world/" + "world_demo" + "/model/" + model_name + "/joint_state"
    )
    odom_gz_topic = "/model/" + model_name + "/odometry"
    link_pose_gz_topic = "/model/" + model_name + "/pose"
    link_pose_static_gz_topic = "/model/" + model_name + "/pose_static"
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            joint_state_gz_topic + "@sensor_msgs/msg/JointState[gz.msgs.Model",
            odom_gz_topic + "@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            link_pose_gz_topic + "@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            link_pose_static_gz_topic + "_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        remappings=[
            (joint_state_gz_topic, "joint_states"),
            (link_pose_gz_topic, "/tf"),
            (link_pose_static_gz_topic, "/tf_static"),
        ],
        parameters=[
            {"qos_overrides./tf_static.publisher.durability": "transient_local"}
        ],
        output="screen",
    )

    # Get the parser plugin convert sdf to urdf using robot_description topic
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_desc},
        ],
    )

    # Launch rviz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg_buoy_gazebo, "rviz", "ellipsoid_buoy.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription(
        [
            gazebo_world_file_launch_arg,
            gazebo_world_name_launch_arg,
            rviz_launch_arg,
            gazebo,
            bridge,
            robot_state_publisher,
            rviz,
        ]
    )
