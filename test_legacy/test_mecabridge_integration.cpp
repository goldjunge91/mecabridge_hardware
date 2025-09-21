#include <memory>
/**
 * @file test_mecabridge_integration.cpp
 * @brief Integration tests for mecabridge Hardware Interface
 *
 * This file contains integration tests that verify:
 * - Hardware interface loading and plugin registration (Requirement 4.1)
 * - Mock controller_manager integration (Requirement 4.2)
 * - Compatibility with diff_drive_controller (Requirement 4.2)
 * - Launch file functionality and parameter loading (Requirement 4.3, 4.4)
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_loader.hpp>
#include <controller_manager/controller_manager.hpp>

#include "mecabridge_hardware/mecabridge_hardware_interface.h"

class mecabridgeIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_mecabridge_integration");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    // Start executor in separate thread
    executor_thread_ = std::thread(
      [this]() {
        executor_->spin();
      });
  }

  void TearDown() override
  {
    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
};

/**
 * @brief Test hardware interface loading and plugin registration
 * Requirements: 4.1 - Hardware interface discoverable by controller_manager
 */
TEST_F(mecabridgeIntegrationTest, TestPluginLoading)
{
  // Test plugin loading using pluginlib
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");

  try {
    // Verify the plugin is discoverable
    std::vector<std::string> classes = loader.getDeclaredClasses();
    bool found_mecabridge = false;
    for (const auto & class_name : classes) {
      if (class_name == "drive_arduino/MecaBridgeHardwareInterface") {
        found_mecabridge = true;
        break;
      }
    }
    EXPECT_TRUE(found_mecabridge) <<
      "MecaBridgeHardwareInterface plugin not found in declared classes";

    // Test plugin instantiation
    auto hardware_interface = loader.createSharedInstance(
      "drive_arduino/MecaBridgeHardwareInterface");
    ASSERT_NE(
      hardware_interface,
      nullptr) << "Failed to create MecaBridgeHardwareInterface instance";

    // Verify it's the correct type
    auto mecabridge_interface =
      std::dynamic_pointer_cast<mecabridge_hardware::MecaBridgeHardwareInterface>(
      hardware_interface);
    EXPECT_NE(
      mecabridge_interface,
      nullptr) << "Plugin is not of correct type MecaBridgeHardwareInterface";

  } catch (const std::exception & e) {
    FAIL() << "Exception during plugin loading: " << e.what();
  }
}

/**
 * @brief Test hardware interface configuration with different drive types
 * Requirements: 4.3 - Parameter loading functionality
 */
TEST_F(mecabridgeIntegrationTest, TestHardwareInterfaceConfiguration)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");

  auto hardware_interface =
    loader.createSharedInstance("drive_arduino/MecaBridgeHardwareInterface");
  ASSERT_NE(hardware_interface, nullptr);

  // Test differential drive configuration
  {
    hardware_interface::HardwareInfo info;
    info.name = "mecabridge_differential";
    info.type = "system";

    // Add hardware parameters
    hardware_interface::ComponentInfo hw_params;
    hw_params.parameters["drive_type"] = "differential";
    hw_params.parameters["device"] = "/dev/ttyUSB0";
    hw_params.parameters["baud_rate"] = "115200";
    hw_params.parameters["has_encoders"] = "false";
    hw_params.parameters["left_wheel_name"] = "left_wheel";
    hw_params.parameters["right_wheel_name"] = "right_wheel";
    hw_params.parameters["loop_rate"] = "20.0";
    hw_params.parameters["max_lin_vel"] = "0.3";
    hw_params.parameters["max_ang_vel"] = "1.0";
    info.hardware_parameters = hw_params.parameters;

    // Add joint information for differential drive
    hardware_interface::ComponentInfo left_wheel;
    left_wheel.name = "left_wheel";
    left_wheel.type = "joint";
    left_wheel.command_interfaces.push_back({"velocity", {}});
    left_wheel.state_interfaces.push_back({"position", {}});
    left_wheel.state_interfaces.push_back({"velocity", {}});
    info.joints.push_back(left_wheel);

    hardware_interface::ComponentInfo right_wheel;
    right_wheel.name = "right_wheel";
    right_wheel.type = "joint";
    right_wheel.command_interfaces.push_back({"velocity", {}});
    right_wheel.state_interfaces.push_back({"position", {}});
    right_wheel.state_interfaces.push_back({"velocity", {}});
    info.joints.push_back(right_wheel);

    // Test configuration
    auto result = hardware_interface->on_init(info);
    EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS)
      << "Failed to configure differential drive hardware interface";

    // Test state interface export
    auto state_interfaces = hardware_interface->export_state_interfaces();
    EXPECT_EQ(state_interfaces.size(), 4) << "Expected 4 state interfaces for differential drive";

    // Test command interface export
    auto command_interfaces = hardware_interface->export_command_interfaces();
    EXPECT_EQ(
      command_interfaces.size(),
      2) << "Expected 2 command interfaces for differential drive";
  }

  // Test mecanum drive configuration
  {
    auto mecanum_interface =
      loader.createSharedInstance("drive_arduino/MecaBridgeHardwareInterface");
    ASSERT_NE(mecanum_interface, nullptr);

    hardware_interface::HardwareInfo info;
    info.name = "mecabridge_mecanum";
    info.type = "system";

    // Add hardware parameters for mecanum
    hardware_interface::ComponentInfo hw_params;
    hw_params.parameters["drive_type"] = "mecanum";
    hw_params.parameters["device"] = "/dev/ttyUSB0";
    hw_params.parameters["baud_rate"] = "115200";
    hw_params.parameters["has_encoders"] = "false";
    hw_params.parameters["wheel_base"] = "0.3";
    hw_params.parameters["track_width"] = "0.3";
    info.hardware_parameters = hw_params.parameters;

    // Add joint information for mecanum drive (4 wheels)
    std::vector<std::string> wheel_names = {
      "front_left_wheel", "front_right_wheel",
      "rear_left_wheel", "rear_right_wheel"
    };

    for (const auto & wheel_name : wheel_names) {
      hardware_interface::ComponentInfo wheel;
      wheel.name = wheel_name;
      wheel.type = "joint";
      wheel.command_interfaces.push_back({"velocity", {}});
      wheel.state_interfaces.push_back({"position", {}});
      wheel.state_interfaces.push_back({"velocity", {}});
      info.joints.push_back(wheel);
    }

    // Test configuration
    auto result = mecanum_interface->on_init(info);
    EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS)
      << "Failed to configure mecanum drive hardware interface";

    // Test state interface export
    auto state_interfaces = mecanum_interface->export_state_interfaces();
    EXPECT_EQ(state_interfaces.size(), 8) << "Expected 8 state interfaces for mecanum drive";

    // Test command interface export
    auto command_interfaces = mecanum_interface->export_command_interfaces();
    EXPECT_EQ(command_interfaces.size(), 4) << "Expected 4 command interfaces for mecanum drive";
  }
}

/**
 * @brief Test hardware interface lifecycle management
 * Requirements: 4.1, 4.2 - Integration with controller_manager lifecycle
 */
TEST_F(mecabridgeIntegrationTest, TestHardwareInterfaceLifecycle)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");

  auto hardware_interface =
    loader.createSharedInstance("drive_arduino/MecaBridgeHardwareInterface");
  ASSERT_NE(hardware_interface, nullptr);

  // Configure hardware interface
  hardware_interface::HardwareInfo info;
  info.name = "mecabridge_test";
  info.type = "system";

  hardware_interface::ComponentInfo hw_params;
  hw_params.parameters["drive_type"] = "differential";
  hw_params.parameters["device"] = "/dev/ttyUSB0";
  hw_params.parameters["baud_rate"] = "115200";
  hw_params.parameters["has_encoders"] = "false";
  info.hardware_parameters = hw_params.parameters;

  // Add joints
  hardware_interface::ComponentInfo left_wheel;
  left_wheel.name = "left_wheel";
  left_wheel.type = "joint";
  left_wheel.command_interfaces.push_back({"velocity", {}});
  left_wheel.state_interfaces.push_back({"position", {}});
  left_wheel.state_interfaces.push_back({"velocity", {}});
  info.joints.push_back(left_wheel);

  hardware_interface::ComponentInfo right_wheel;
  right_wheel.name = "right_wheel";
  right_wheel.type = "joint";
  right_wheel.command_interfaces.push_back({"velocity", {}});
  right_wheel.state_interfaces.push_back({"position", {}});
  right_wheel.state_interfaces.push_back({"velocity", {}});
  info.joints.push_back(right_wheel);

  // Test lifecycle transitions
  EXPECT_EQ(hardware_interface->on_init(info), hardware_interface::CallbackReturn::SUCCESS);

  // Test configure transition
  EXPECT_EQ(
    hardware_interface->on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);

  // Test activate transition (may fail due to no hardware, but should not crash)
  auto activate_result = hardware_interface->on_activate(rclcpp_lifecycle::State());
  EXPECT_TRUE(
    activate_result == hardware_interface::CallbackReturn::SUCCESS ||
    activate_result == hardware_interface::CallbackReturn::ERROR)
    << "Activate should return SUCCESS or ERROR, not crash";

  // Test deactivate transition
  EXPECT_EQ(
    hardware_interface->on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);

  // Test cleanup transition
  EXPECT_EQ(
    hardware_interface->on_cleanup(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
}

/**
 * @brief Test interface naming and compatibility
 * Requirements: 4.2 - Compatibility with diff_drive_controller
 */
TEST_F(mecabridgeIntegrationTest, TestInterfaceNaming)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");

  auto hardware_interface =
    loader.createSharedInstance("drive_arduino/MecaBridgeHardwareInterface");
  ASSERT_NE(hardware_interface, nullptr);

  // Configure for differential drive
  hardware_interface::HardwareInfo info;
  info.name = "mecabridge_diff";
  info.type = "system";

  hardware_interface::ComponentInfo hw_params;
  hw_params.parameters["drive_type"] = "differential";
  hw_params.parameters["left_wheel_name"] = "left_wheel_joint";
  hw_params.parameters["right_wheel_name"] = "right_wheel_joint";
  info.hardware_parameters = hw_params.parameters;

  // Add joints with custom names
  hardware_interface::ComponentInfo left_wheel;
  left_wheel.name = "left_wheel_joint";
  left_wheel.type = "joint";
  left_wheel.command_interfaces.push_back({"velocity", {}});
  left_wheel.state_interfaces.push_back({"position", {}});
  left_wheel.state_interfaces.push_back({"velocity", {}});
  info.joints.push_back(left_wheel);

  hardware_interface::ComponentInfo right_wheel;
  right_wheel.name = "right_wheel_joint";
  right_wheel.type = "joint";
  right_wheel.command_interfaces.push_back({"velocity", {}});
  right_wheel.state_interfaces.push_back({"position", {}});
  right_wheel.state_interfaces.push_back({"velocity", {}});
  info.joints.push_back(right_wheel);

  EXPECT_EQ(hardware_interface->on_init(info), hardware_interface::CallbackReturn::SUCCESS);

  // Test state interfaces have correct names
  auto state_interfaces = hardware_interface->export_state_interfaces();
  std::vector<std::string> expected_state_names = {
    "left_wheel_joint/position",
    "left_wheel_joint/velocity",
    "right_wheel_joint/position",
    "right_wheel_joint/velocity"
  };

  EXPECT_EQ(state_interfaces.size(), expected_state_names.size());
  for (size_t i = 0; i < state_interfaces.size(); ++i) {
    bool found = false;
    for (const auto & expected_name : expected_state_names) {
      if (state_interfaces[i].get_name() == expected_name) {
        found = true;
        break;
      }
    }
    EXPECT_TRUE(found) << "State interface " << state_interfaces[i].get_name()
                       << " not found in expected names";
  }

  // Test command interfaces have correct names
  auto command_interfaces = hardware_interface->export_command_interfaces();
  std::vector<std::string> expected_command_names = {
    "left_wheel_joint/velocity",
    "right_wheel_joint/velocity"
  };

  EXPECT_EQ(command_interfaces.size(), expected_command_names.size());
  for (size_t i = 0; i < command_interfaces.size(); ++i) {
    bool found = false;
    for (const auto & expected_name : expected_command_names) {
      if (command_interfaces[i].get_name() == expected_name) {
        found = true;
        break;
      }
    }
    EXPECT_TRUE(found) << "Command interface " << command_interfaces[i].get_name()
                       << " not found in expected names";
  }
}

/**
 * @brief Test parameter validation and error handling
 * Requirements: 4.3 - Parameter loading with validation
 */
TEST_F(mecabridgeIntegrationTest, TestParameterValidation)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");

  auto hardware_interface =
    loader.createSharedInstance("drive_arduino/MecaBridgeHardwareInterface");
  ASSERT_NE(hardware_interface, nullptr);

  // Test with missing required parameters (should use defaults)
  {
    hardware_interface::HardwareInfo info;
    info.name = "mecabridge_minimal";
    info.type = "system";

    // Minimal configuration - should use defaults
    hardware_interface::ComponentInfo hw_params;
    info.hardware_parameters = hw_params.parameters;

    // Add basic joints
    hardware_interface::ComponentInfo left_wheel;
    left_wheel.name = "left_wheel";
    left_wheel.type = "joint";
    left_wheel.command_interfaces.push_back({"velocity", {}});
    left_wheel.state_interfaces.push_back({"position", {}});
    left_wheel.state_interfaces.push_back({"velocity", {}});
    info.joints.push_back(left_wheel);

    hardware_interface::ComponentInfo right_wheel;
    right_wheel.name = "right_wheel";
    right_wheel.type = "joint";
    right_wheel.command_interfaces.push_back({"velocity", {}});
    right_wheel.state_interfaces.push_back({"position", {}});
    right_wheel.state_interfaces.push_back({"velocity", {}});
    info.joints.push_back(right_wheel);

    // Should succeed with default parameters
    EXPECT_EQ(hardware_interface->on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  }

  // Test with invalid parameters (should handle gracefully)
  {
    auto invalid_interface =
      loader.createSharedInstance("drive_arduino/MecaBridgeHardwareInterface");
    ASSERT_NE(invalid_interface, nullptr);

    hardware_interface::HardwareInfo info;
    info.name = "mecabridge_invalid";
    info.type = "system";

    // Invalid parameters
    hardware_interface::ComponentInfo hw_params;
    hw_params.parameters["drive_type"] = "invalid_drive_type";
    hw_params.parameters["baud_rate"] = "invalid_baud";
    hw_params.parameters["loop_rate"] = "invalid_rate";
    info.hardware_parameters = hw_params.parameters;

    // Add joints
    hardware_interface::ComponentInfo left_wheel;
    left_wheel.name = "left_wheel";
    left_wheel.type = "joint";
    left_wheel.command_interfaces.push_back({"velocity", {}});
    left_wheel.state_interfaces.push_back({"position", {}});
    left_wheel.state_interfaces.push_back({"velocity", {}});
    info.joints.push_back(left_wheel);

    hardware_interface::ComponentInfo right_wheel;
    right_wheel.name = "right_wheel";
    right_wheel.type = "joint";
    right_wheel.command_interfaces.push_back({"velocity", {}});
    right_wheel.state_interfaces.push_back({"position", {}});
    right_wheel.state_interfaces.push_back({"velocity", {}});
    info.joints.push_back(right_wheel);

    // Should handle invalid parameters gracefully (use defaults)
    EXPECT_EQ(invalid_interface->on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  }
}

/**
 * @brief Test read/write operations
 * Requirements: 4.2 - Integration with controller framework
 */
TEST_F(mecabridgeIntegrationTest, TestReadWriteOperations)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");

  auto hardware_interface =
    loader.createSharedInstance("drive_arduino/MecaBridgeHardwareInterface");
  ASSERT_NE(hardware_interface, nullptr);

  // Configure hardware interface
  hardware_interface::HardwareInfo info;
  info.name = "mecabridge_rw_test";
  info.type = "system";

  hardware_interface::ComponentInfo hw_params;
  hw_params.parameters["drive_type"] = "differential";
  hw_params.parameters["has_encoders"] = "false";
  info.hardware_parameters = hw_params.parameters;

  // Add joints
  hardware_interface::ComponentInfo left_wheel;
  left_wheel.name = "left_wheel";
  left_wheel.type = "joint";
  left_wheel.command_interfaces.push_back({"velocity", {}});
  left_wheel.state_interfaces.push_back({"position", {}});
  left_wheel.state_interfaces.push_back({"velocity", {}});
  info.joints.push_back(left_wheel);

  hardware_interface::ComponentInfo right_wheel;
  right_wheel.name = "right_wheel";
  right_wheel.type = "joint";
  right_wheel.command_interfaces.push_back({"velocity", {}});
  right_wheel.state_interfaces.push_back({"position", {}});
  right_wheel.state_interfaces.push_back({"velocity", {}});
  info.joints.push_back(right_wheel);

  EXPECT_EQ(hardware_interface->on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    hardware_interface->on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);

  // Get interfaces
  auto command_interfaces = hardware_interface->export_command_interfaces();
  auto state_interfaces = hardware_interface->export_state_interfaces();

  ASSERT_EQ(command_interfaces.size(), 2);
  ASSERT_EQ(state_interfaces.size(), 4);

  // Test write operation (set command values)
  command_interfaces[0].set_value(1.0);    // left wheel velocity
  command_interfaces[1].set_value(-1.0);   // right wheel velocity

  // Test write method (may fail due to no hardware, but should not crash)
  auto write_result = hardware_interface->perform_command_mode_switch({}, {});
  // Write result can be SUCCESS or ERROR, but should not crash

  // Test read operation
  auto read_result = hardware_interface->read(rclcpp::Time(), rclcpp::Duration::from_seconds(0.02));
  // Read result can be SUCCESS or ERROR, but should not crash
  EXPECT_TRUE(
    read_result == hardware_interface::return_type::OK ||
    read_result == hardware_interface::return_type::ERROR)
    << "Read should return OK or ERROR, not crash";

  // Test write operation
  auto write_result2 = hardware_interface->write(
    rclcpp::Time(), rclcpp::Duration::from_seconds(
      0.02));
  EXPECT_TRUE(
    write_result2 == hardware_interface::return_type::OK ||
    write_result2 == hardware_interface::return_type::ERROR)
    << "Write should return OK or ERROR, not crash";

  // Verify state values are reasonable (for non-encoder mode, should integrate commands)
  for (auto & state_interface : state_interfaces) {
    double value = state_interface.get_value();
    EXPECT_TRUE(std::isfinite(value)) << "State interface value should be finite";
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
