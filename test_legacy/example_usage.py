#!/usr/bin/env python3
"""
mecabridge Hardware Interface Example Usage

This script demonstrates how to use the mecabridge hardware interface
with different drive configurations and provides examples for
integration testing.

Requirements demonstrated:
- 4.1: Hardware interface loading and plugin registration
- 4.2: Integration with controller_manager and diff_drive_controller
- 4.3: Launch file functionality
- 4.4: Parameter loading
"""

import os
import rclpy
import subprocess
import sys
import threading
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pathlib import Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class mecabridgeExampleUsage(Node):
    """Example usage node for mecabridge hardware interface"""

    def __init__(self):
        super().__init__('mecabridge_example_usage')

        # Publishers for sending commands
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/diff_cont/cmd_vel_unstamped', 10)

        # Subscribers for monitoring feedback
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/diff_cont/odom', self.odom_callback, 10)

        # State tracking
        self.joint_states_received = False
        self.odom_received = False
        self.last_joint_state = None
        self.last_odom = None

        self.get_logger().info("mecabridge Example Usage Node initialized")

    def joint_state_callback(self, msg):
        """Callback for joint state messages"""
        self.joint_states_received = True
        self.last_joint_state = msg
        self.get_logger().debug(f"Received joint states: {len(msg.name)} joints")

    def odom_callback(self, msg):
        """Callback for odometry messages"""
        self.odom_received = True
        self.last_odom = msg
        self.get_logger().debug(f"Received odometry: x={msg.pose.pose.position.x:.3f}, "
                                f"y={msg.pose.pose.position.y:.3f}")

    def send_velocity_command(self, linear_x=0.0, angular_z=0.0):
        """Send velocity command to the robot"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"Sent velocity command: linear={linear_x}, angular={angular_z}")

    def check_system_status(self):
        """Check if the hardware interface system is working"""
        self.get_logger().info("Checking system status...")

        # Wait for initial messages
        timeout = 10.0  # seconds
        start_time = time.time()

        while (time.time() - start_time) < timeout:
            if self.joint_states_received and self.odom_received:
                self.get_logger().info("✓ System is operational - receiving joint states and odometry")
                return True
            time.sleep(0.1)

        self.get_logger().warn("⚠ System status check timed out")
        if not self.joint_states_received:
            self.get_logger().warn("  - No joint states received")
        if not self.odom_received:
            self.get_logger().warn("  - No odometry received")

        return False

    def run_movement_test(self):
        """Run a simple movement test"""
        self.get_logger().info("Running movement test...")

        # Test sequence: forward, turn left, turn right, stop
        test_commands = [
            (0.1, 0.0, "Moving forward"),
            (0.0, 0.5, "Turning left"),
            (0.0, -0.5, "Turning right"),
            (0.0, 0.0, "Stopping")
        ]

        for linear, angular, description in test_commands:
            self.get_logger().info(f"Test: {description}")
            self.send_velocity_command(linear, angular)
            time.sleep(2.0)  # Run each command for 2 seconds

        self.get_logger().info("Movement test completed")

    def print_system_info(self):
        """Print information about the current system state"""
        self.get_logger().info("=== System Information ===")

        if self.last_joint_state:
            self.get_logger().info(f"Joint States:")
            for i, name in enumerate(self.last_joint_state.name):
                pos = self.last_joint_state.position[i] if i < len(self.last_joint_state.position) else 0.0
                vel = self.last_joint_state.velocity[i] if i < len(self.last_joint_state.velocity) else 0.0
                self.get_logger().info(f"  {name}: pos={pos:.3f}, vel={vel:.3f}")

        if self.last_odom:
            pos = self.last_odom.pose.pose.position
            orient = self.last_odom.pose.pose.orientation
            twist = self.last_odom.twist.twist
            self.get_logger().info(f"Odometry:")
            self.get_logger().info(f"  Position: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}")
            self.get_logger().info(
                f"  Orientation: x={orient.x:.3f}, y={orient.y:.3f}, z={orient.z:.3f}, w={orient.w:.3f}")
            self.get_logger().info(f"  Twist: linear={twist.linear.x:.3f}, angular={twist.angular.z:.3f}")


def run_ros2_command(command, timeout=30):
    """Run a ROS 2 command and return success status"""
    try:
        result = subprocess.run(command, shell=True, capture_output=True,
                                text=True, timeout=timeout)
        return result.returncode == 0, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return False, "", "Command timed out"
    except Exception as e:
        return False, "", str(e)


def check_ros2_environment():
    """Check if ROS 2 environment is properly set up"""
    print("Checking ROS 2 environment...")

    # Check ROS_DISTRO
    ros_distro = os.environ.get('ROS_DISTRO')
    if not ros_distro:
        print("❌ ROS_DISTRO not set. Please source your ROS 2 setup.bash")
        return False
    print(f"✓ ROS_DISTRO: {ros_distro}")

    # Check if drive_arduino package is available
    success, stdout, stderr = run_ros2_command("ros2 pkg list | grep drive_arduino")
    if success:
        print("✓ drive_arduino package is available")
    else:
        print("❌ drive_arduino package not found. Please build the workspace.")
        return False

    return True


def test_plugin_loading():
    """Test that the mecabridge hardware interface plugin can be loaded"""
    print("\nTesting plugin loading...")

    # This would normally be done by controller_manager, but we can test the concept
    try:
        import pluginlib
        loader = pluginlib.ClassLoader('hardware_interface', 'hardware_interface::SystemInterface')
        classes = loader.getDeclaredClasses()

        if 'drive_arduino/MecaBridgeHardwareInterface' in classes:
            print("✓ MecaBridgeHardwareInterface plugin is discoverable")
            return True
        else:
            print("❌ MecaBridgeHardwareInterface plugin not found")
            print(f"Available classes: {classes}")
            return False
    except Exception as e:
        print(f"❌ Error testing plugin loading: {e}")
        return False


def test_launch_files():
    """Test that launch files exist and have valid syntax"""
    print("\nTesting launch files...")

    package_path = Path("src/drive_arduino")
    launch_dir = package_path / "launch"

    if not launch_dir.exists():
        print(f"❌ Launch directory not found: {launch_dir}")
        return False

    launch_files = [
        "mecabridge_hardware.launch.py",
        "mecabridge_differential.launch.py"
    ]

    for launch_file in launch_files:
        launch_path = launch_dir / launch_file
        if launch_path.exists():
            try:
                # Test syntax by compiling
                with open(launch_path, 'r') as f:
                    compile(f.read(), str(launch_path), 'exec')
                print(f"✓ {launch_file} syntax is valid")
            except SyntaxError as e:
                print(f"❌ {launch_file} has syntax error: {e}")
                return False
        else:
            print(f"⚠ {launch_file} not found")

    return True


def test_controller_configs():
    """Test that controller configuration files exist and are valid"""
    print("\nTesting controller configurations...")

    package_path = Path("src/drive_arduino")
    controllers_dir = package_path / "controllers"

    if not controllers_dir.exists():
        print(f"❌ Controllers directory not found: {controllers_dir}")
        return False

    config_files = [
        "mecabridge_differential_enhanced.yaml",
        "mecabridge_hardware_params.yaml"
    ]

    for config_file in config_files:
        config_path = controllers_dir / config_file
        if config_path.exists():
            try:
                import yaml
                with open(config_path, 'r') as f:
                    yaml.safe_load(f)
                print(f"✓ {config_file} is valid YAML")
            except yaml.YAMLError as e:
                print(f"❌ {config_file} has YAML error: {e}")
                return False
        else:
            print(f"⚠ {config_file} not found")

    return True


def main():
    """Main function demonstrating mecabridge hardware interface usage"""
    print("mecabridge Hardware Interface Example Usage")
    print("=" * 50)

    # Check environment
    if not check_ros2_environment():
        print("\n❌ Environment check failed. Please fix the issues above.")
        return 1

    # Test static components
    plugin_ok = test_plugin_loading()
    launch_ok = test_launch_files()
    config_ok = test_controller_configs()

    if not all([plugin_ok, launch_ok, config_ok]):
        print("\n❌ Static tests failed. Please fix the issues above.")
        return 1

    print("\n✓ All static tests passed!")

    # Initialize ROS 2
    print("\nInitializing ROS 2...")
    rclpy.init()

    try:
        # Create example node
        node = mecabridgeExampleUsage()
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        # Start executor in separate thread
        executor_thread = threading.Thread(target=executor.spin)
        executor_thread.daemon = True
        executor_thread.start()

        print("✓ ROS 2 node initialized")

        # Note: For full testing, you would need to launch the hardware interface first:
        # ros2 launch drive_arduino mecabridge_differential.launch.py use_mock_hardware:=true

        print("\n" + "=" * 50)
        print("INTEGRATION TEST SUMMARY")
        print("=" * 50)
        print("✓ Plugin loading test passed")
        print("✓ Launch file syntax test passed")
        print("✓ Controller configuration test passed")
        print("✓ ROS 2 node initialization passed")
        print("")
        print("To complete integration testing:")
        print("1. Launch the hardware interface:")
        print("   ros2 launch drive_arduino mecabridge_differential.launch.py use_mock_hardware:=true")
        print("")
        print("2. In another terminal, run this script again to test communication:")
        print("   python3 src/drive_arduino/test/example_usage.py --test-communication")
        print("")
        print("3. Monitor the system:")
        print("   ros2 topic list")
        print("   ros2 topic echo /joint_states")
        print("   ros2 topic echo /diff_cont/odom")

        # If --test-communication argument is provided, run communication tests
        if len(sys.argv) > 1 and sys.argv[1] == "--test-communication":
            print("\nRunning communication tests...")

            # Check system status
            if node.check_system_status():
                print("✓ Communication test passed")

                # Run movement test
                node.run_movement_test()

                # Print system info
                time.sleep(1.0)  # Allow final messages to arrive
                node.print_system_info()

                print("\n✓ All integration tests completed successfully!")
            else:
                print("❌ Communication test failed")
                print("Make sure the hardware interface is running:")
                print("ros2 launch drive_arduino mecabridge_differential.launch.py use_mock_hardware:=true")
                return 1

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
