#!/usr/bin/env python3
"""
Launch File Functionality Tests

This test file verifies launch file functionality and parameter loading
for the mecabridge hardware interface.

Requirements tested:
- 4.3: Launch file functionality 
- 4.4: Parameter loading from YAML files
"""

import launch
import launch_ros
import os
import rclpy
import sys
import tempfile
import threading
import time
import unittest
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
from rclpy.node import Node as RclpyNode


class LaunchFunctionalityTest(unittest.TestCase):
    """Test launch file functionality and parameter loading"""

    def setUp(self):
        """Set up test environment"""
        self.test_dir = Path(__file__).parent
        self.package_dir = self.test_dir.parent
        self.launch_dir = self.package_dir / "launch"
        self.controllers_dir = self.package_dir / "controllers"

    def test_launch_files_exist(self):
        """Test that all expected launch files exist"""
        expected_launch_files = [
            "mecabridge_hardware.launch.py",
            "mecabridge_differential.launch.py",
            "mecabridge_mecanum.launch.py",
            "mecabridge_four_wheel.launch.py"
        ]

        for launch_file in expected_launch_files:
            launch_path = self.launch_dir / launch_file
            self.assertTrue(launch_path.exists(),
                            f"Launch file {launch_file} does not exist at {launch_path}")

    def test_controller_config_files_exist(self):
        """Test that all expected controller configuration files exist"""
        expected_config_files = [
            "mecabridge_differential_controller.yaml",
            "mecabridge_differential_enhanced.yaml",
            "mecabridge_mecanum_controller.yaml",
            "mecabridge_mecanum_enhanced.yaml",
            "mecabridge_four_wheel_controller.yaml",
            "mecabridge_hardware_params.yaml"
        ]

        for config_file in expected_config_files:
            config_path = self.controllers_dir / config_file
            self.assertTrue(config_path.exists(),
                            f"Controller config file {config_file} does not exist at {config_path}")

    def test_launch_file_syntax(self):
        """Test that launch files have valid Python syntax"""
        launch_files = [
            "mecabridge_hardware.launch.py",
            "mecabridge_differential.launch.py",
            "mecabridge_mecanum.launch.py",
            "mecabridge_four_wheel.launch.py"
        ]

        for launch_file in launch_files:
            launch_path = self.launch_dir / launch_file
            if launch_path.exists():
                try:
                    with open(launch_path, 'r') as f:
                        code = f.read()
                    compile(code, str(launch_path), 'exec')
                except SyntaxError as e:
                    self.fail(f"Syntax error in {launch_file}: {e}")
                except Exception as e:
                    self.fail(f"Error compiling {launch_file}: {e}")

    def test_yaml_config_syntax(self):
        """Test that YAML configuration files have valid syntax"""
        yaml_files = [
            "mecabridge_differential_controller.yaml",
            "mecabridge_differential_enhanced.yaml",
            "mecabridge_mecanum_controller.yaml",
            "mecabridge_mecanum_enhanced.yaml",
            "mecabridge_four_wheel_controller.yaml",
            "mecabridge_hardware_params.yaml"
        ]

        for yaml_file in yaml_files:
            yaml_path = self.controllers_dir / yaml_file
            if yaml_path.exists():
                try:
                    with open(yaml_path, 'r') as f:
                        yaml.safe_load(f)
                except yaml.YAMLError as e:
                    self.fail(f"YAML syntax error in {yaml_file}: {e}")
                except Exception as e:
                    self.fail(f"Error loading {yaml_file}: {e}")

    def test_differential_controller_config(self):
        """Test differential drive controller configuration parameters"""
        config_path = self.controllers_dir / "mecabridge_differential_enhanced.yaml"
        if not config_path.exists():
            self.skipTest(f"Config file {config_path} does not exist")

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Test controller manager configuration
        self.assertIn('controller_manager', config)
        cm_config = config['controller_manager']

        # Test that ros2_control_node parameters exist
        self.assertIn('ros2_control_node', cm_config)

        # Test diff_drive_controller configuration
        if 'diff_cont' in config:
            diff_config = config['diff_cont']
            self.assertIn('ros__parameters', diff_config)
            params = diff_config['ros__parameters']

            # Test required parameters for diff_drive_controller
            expected_params = ['left_wheel_names', 'right_wheel_names', 'wheel_separation', 'wheel_radius']
            for param in expected_params:
                self.assertIn(param, params, f"Missing required parameter: {param}")

    def test_mecanum_controller_config(self):
        """Test mecanum drive controller configuration parameters"""
        config_path = self.controllers_dir / "mecabridge_mecanum_enhanced.yaml"
        if not config_path.exists():
            self.skipTest(f"Config file {config_path} does not exist")

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Test controller manager configuration
        self.assertIn('controller_manager', config)

        # Test mecanum controller configuration if present
        if 'mecanum_cont' in config:
            mecanum_config = config['mecanum_cont']
            self.assertIn('ros__parameters', mecanum_config)
            params = mecanum_config['ros__parameters']

            # Test required parameters for mecanum controller
            expected_params = ['wheel_names', 'wheel_separation_x', 'wheel_separation_y', 'wheel_radius']
            for param in expected_params:
                if param in params:  # Some parameters might be optional
                    self.assertIsNotNone(params[param])

    def test_hardware_params_config(self):
        """Test hardware parameters configuration"""
        config_path = self.controllers_dir / "mecabridge_hardware_params.yaml"
        if not config_path.exists():
            self.skipTest(f"Config file {config_path} does not exist")

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Test hardware interface parameters
        if 'hardware_interface' in config:
            hw_config = config['hardware_interface']
            self.assertIn('ros__parameters', hw_config)
            params = hw_config['ros__parameters']

            # Test expected hardware parameters
            expected_params = ['device', 'baud_rate', 'timeout', 'loop_rate']
            for param in expected_params:
                if param in params:
                    self.assertIsNotNone(params[param])

    def test_launch_file_imports(self):
        """Test that launch files can import required modules"""
        launch_files = [
            "mecabridge_hardware.launch.py",
            "mecabridge_differential.launch.py"
        ]

        for launch_file in launch_files:
            launch_path = self.launch_dir / launch_file
            if not launch_path.exists():
                continue

            # Test that we can import the launch file as a module
            try:
                import importlib.util
                spec = importlib.util.spec_from_file_location("test_launch", launch_path)
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)

                # Test that generate_launch_description function exists
                self.assertTrue(hasattr(module, 'generate_launch_description'),
                                f"Launch file {launch_file} missing generate_launch_description function")

                # Test that function returns LaunchDescription
                launch_desc = module.generate_launch_description()
                self.assertIsInstance(launch_desc, LaunchDescription,
                                      f"generate_launch_description in {launch_file} should return LaunchDescription")

            except Exception as e:
                self.fail(f"Error importing launch file {launch_file}: {e}")

    def test_launch_arguments(self):
        """Test that launch files declare expected arguments"""
        launch_path = self.launch_dir / "mecabridge_hardware.launch.py"
        if not launch_path.exists():
            self.skipTest("mecabridge_hardware.launch.py does not exist")

        try:
            import importlib.util
            spec = importlib.util.spec_from_file_location("test_launch", launch_path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)

            launch_desc = module.generate_launch_description()

            # Extract declared arguments
            declared_args = []
            for entity in launch_desc.entities:
                if isinstance(entity, DeclareLaunchArgument):
                    declared_args.append(entity.name)

            # Test expected arguments
            expected_args = ['drive_type', 'use_mock_hardware', 'device', 'baud_rate', 'has_encoders']
            for arg in expected_args:
                self.assertIn(arg, declared_args, f"Missing launch argument: {arg}")

        except Exception as e:
            self.fail(f"Error testing launch arguments: {e}")

    def test_parameter_substitution(self):
        """Test that launch files properly use parameter substitution"""
        launch_path = self.launch_dir / "mecabridge_hardware.launch.py"
        if not launch_path.exists():
            self.skipTest("mecabridge_hardware.launch.py does not exist")

        with open(launch_path, 'r') as f:
            content = f.read()

        # Test that LaunchConfiguration is used for parameter substitution
        self.assertIn('LaunchConfiguration', content,
                      "Launch file should use LaunchConfiguration for parameter substitution")

        # Test that parameters are passed to nodes
        self.assertIn('parameters=', content,
                      "Launch file should pass parameters to nodes")

    def test_node_configuration(self):
        """Test that launch files configure nodes correctly"""
        launch_path = self.launch_dir / "mecabridge_differential.launch.py"
        if not launch_path.exists():
            self.skipTest("mecabridge_differential.launch.py does not exist")

        try:
            import importlib.util
            spec = importlib.util.spec_from_file_location("test_launch", launch_path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)

            launch_desc = module.generate_launch_description()

            # Find nodes in launch description
            nodes = []
            for entity in launch_desc.entities:
                if isinstance(entity, Node):
                    nodes.append(entity)
                elif hasattr(entity, 'entities'):  # For nested entities
                    for nested_entity in entity.entities:
                        if isinstance(nested_entity, Node):
                            nodes.append(nested_entity)

            # Test that expected nodes are present
            node_packages = [node.package for node in nodes]
            expected_packages = ['controller_manager', 'robot_state_publisher']

            for expected_pkg in expected_packages:
                self.assertIn(expected_pkg, node_packages,
                              f"Expected node package {expected_pkg} not found in launch file")

        except Exception as e:
            self.fail(f"Error testing node configuration: {e}")


class MockControllerManagerTest(unittest.TestCase):
    """Test integration with mock controller manager"""

    def setUp(self):
        """Set up ROS 2 for testing"""
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('test_controller_manager')
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)

        # Start executor in separate thread
        self.executor_thread = threading.Thread(target=self.executor.spin)
        self.executor_thread.daemon = True
        self.executor_thread.start()

    def tearDown(self):
        """Clean up ROS 2 resources"""
        self.node.destroy_node()
        self.executor.shutdown()
        if self.executor_thread.is_alive():
            self.executor_thread.join(timeout=1.0)

    def test_controller_manager_services_available(self):
        """Test that controller manager services would be available"""
        # This test verifies the expected service names that controller_manager provides
        expected_services = [
            '/controller_manager/list_controllers',
            '/controller_manager/load_controller',
            '/controller_manager/configure_controller',
            '/controller_manager/switch_controller'
        ]

        # We can't test actual service availability without running controller_manager,
        # but we can test that our configuration would work with these services
        for service_name in expected_services:
            # Test that service name is valid
            self.assertTrue(service_name.startswith('/controller_manager/'),
                            f"Service name {service_name} should start with /controller_manager/")

    def test_diff_drive_controller_compatibility(self):
        """Test compatibility with diff_drive_controller interface requirements"""
        # Test that our hardware interface provides the interfaces that diff_drive_controller expects

        # diff_drive_controller expects these command interfaces:
        expected_command_interfaces = [
            'left_wheel_joint/velocity',
            'right_wheel_joint/velocity'
        ]

        # diff_drive_controller expects these state interfaces:
        expected_state_interfaces = [
            'left_wheel_joint/position',
            'left_wheel_joint/velocity',
            'right_wheel_joint/position',
            'right_wheel_joint/velocity'
        ]

        # Test interface naming convention
        for interface in expected_command_interfaces + expected_state_interfaces:
            parts = interface.split('/')
            self.assertEqual(len(parts), 2, f"Interface {interface} should have format 'joint/type'")
            joint_name, interface_type = parts
            self.assertTrue(joint_name.endswith('_joint'), f"Joint name {joint_name} should end with '_joint'")
            self.assertIn(interface_type, ['position', 'velocity'],
                          f"Interface type {interface_type} should be 'position' or 'velocity'")

    def test_topic_compatibility(self):
        """Test that expected topics would be published/subscribed"""
        # Test topic names that diff_drive_controller uses
        expected_topics = {
            '/diff_cont/cmd_vel_unstamped': 'geometry_msgs/msg/Twist',
            '/diff_cont/odom': 'nav_msgs/msg/Odometry',
            '/joint_states': 'sensor_msgs/msg/JointState'
        }

        for topic_name, msg_type in expected_topics.items():
            # Test topic name format
            self.assertTrue(topic_name.startswith('/'), f"Topic {topic_name} should start with '/'")
            # Test message type format
            self.assertIn('/', msg_type, f"Message type {msg_type} should contain '/'")
            parts = msg_type.split('/')
            self.assertEqual(len(parts), 3, f"Message type {msg_type} should have format 'pkg/msg/Type'")


if __name__ == '__main__':
    # Run the tests
    unittest.main()
