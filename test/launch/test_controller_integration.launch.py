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
Integration test launch file for MecaBridge hardware interface.

This launch file:
1. Loads the robot description (URDF)
2. Starts the controller manager with MecaBridge hardware interface
3. Spawns controllers (mecanum drive, joint state broadcaster, etc.)
4. Runs integration tests to verify command flow
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Get package directory
    pkg_share = get_package_share_directory('mecabridge_hardware')

    # Robot description
    urdf_file = os.path.join(pkg_share, 'test', 'urdf', 'test_robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Hardware config
    hardware_config = os.path.join(pkg_share, 'test', 'config', 'test_mecabridge_hardware.yaml')

    # Controller config  
    controller_config = os.path.join(pkg_share, 'test', 'config', 'test_controllers.yaml')

    # Nodes to launch
    nodes = [
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }]
        ),

        # Controller manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[
                hardware_config,
                controller_config,
                {'robot_description': robot_description}
            ]
        ),

        # Joint state broadcaster spawner
        Node(
            package='controller_manager',
            executable='spawner',
            name='joint_state_broadcaster_spawner',
            output='screen',
            arguments=['joint_state_broadcaster']
        ),

        # Mecanum drive controller spawner
        Node(
            package='controller_manager',
            executable='spawner',
            name='mecanum_drive_controller_spawner',
            output='screen',
            arguments=['mecanum_drive_controller']
        ),

        # Servo position controller spawner
        Node(
            package='controller_manager',
            executable='spawner',
            name='servo_position_controller_spawner',
            output='screen',
            arguments=['servo_position_controller']
        ),

        # Integration test node
        Node(
            package='mecabridge_hardware',
            executable='integration_test_node',
            name='integration_test_node',
            output='screen',
            parameters=[{
                'test_duration_seconds': 10.0,
                'expect_responses': True,
            }]
        ),
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        OpaqueFunction(function=launch_setup)
    ])
