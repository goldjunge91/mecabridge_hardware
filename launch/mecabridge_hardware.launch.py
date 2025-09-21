#
# Copyright (c) 2024 MecaBridge Project
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
#

#!/usr/bin/env python3

"""
MecaBridge hardware interface master launch file.

This launch file can start any mecabridge drive configuration based on parameters.
It supports differential, mecanum, and four-wheel independent drive types.

Usage:
    # Differential drive (default)
    ros2 launch drive_arduino mecabridge_hardware.launch.py
    
    # Mecanum drive
    ros2 launch drive_arduino mecabridge_hardware.launch.py drive_type:=mecanum
    
    # Four wheel independent
    ros2 launch drive_arduino mecabridge_hardware.launch.py drive_type:=four_wheel
    
    # With custom parameters
    ros2 launch drive_arduino mecabridge_hardware.launch.py drive_type:=differential device:=/dev/ttyUSB0 has_encoders:=true
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Get launch configuration values
    drive_type = LaunchConfiguration("drive_type").perform(context)
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    device = LaunchConfiguration("device")
    baud_rate = LaunchConfiguration("baud_rate")
    has_encoders = LaunchConfiguration("has_encoders")

    # Determine controller configuration file based on drive type
    controller_configs = {
        "differential": "mecabridge_differential_enhanced.yaml",
        "mecanum": "mecabridge_mecanum_enhanced.yaml",
        "four_wheel": "mecabridge_four_wheel_controller.yaml"
    }

    # Determine controller name based on drive type
    controller_names = {
        "differential": "diff_cont",
        "mecanum": "mecanum_cont",
        "four_wheel": "four_wheel_cont"
    }

    # Determine URDF file based on drive type
    urdf_files = {
        "differential": "mecabridge_differential.urdf.xacro",
        "mecanum": "mecabridge_mecanum.urdf.xacro",
        "four_wheel": "mecabridge_four_wheel.urdf.xacro"
    }

    # Get the appropriate configuration file
    controller_config = controller_configs.get(drive_type, "mecabridge_differential_enhanced.yaml")
    controller_name = controller_names.get(drive_type, "diff_cont")
    urdf_file = urdf_files.get(drive_type, "mecabridge_differential.urdf.xacro")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("robot"), "description", "robot.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
            "device:=",
            device,
            " ",
            "baud_rate:=",
            baud_rate,
            " ",
            "has_encoders:=",
            has_encoders,
            " ",
            "drive_type:=",
            drive_type,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Controller configuration
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("robot"),
            "config",
            "my_controllers.yaml",
        ]
    )

    # Hardware interface parameters
    hardware_params = PathJoinSubstitution(
        [
            FindPackageShare("drive_arduino"),
            "controllers",
            "mecabridge_hardware_params.yaml",
        ]
    )

    # Control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers, hardware_params],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # Robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen",
    )

    # Drive controller spawner
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller_name],
        output="screen",
    )

    # Delay start of robot_controller after joint_state_broadcaster
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[robot_controller_spawner],
        )
    )

    return [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "drive_type",
            default_value="differential",
            description="Drive type: differential, mecanum, or four_wheel",
            choices=["differential", "mecanum", "four_wheel"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations. "
                        "Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "device",
            default_value="",
            description="Serial device path for mecabridge communication. Leave empty for auto-detection.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "baud_rate",
            default_value="115200",
            description="Serial communication baud rate.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "has_encoders",
            default_value="false",
            description="Whether the robot setup includes encoder feedback.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
