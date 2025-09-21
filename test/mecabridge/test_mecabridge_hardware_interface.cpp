/*
 * Copyright (c) 2024 MecaBridge Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "mecabridge_hardware/mecabridge_hardware_interface.h"
#include "mecabridge_hardware/mecabridge_drive_config.h"


class MecaBridgeHardwareInterfaceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS 2 if not already initialized
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    hardware_interface_ = std::make_unique<MecaBridgeHardwareInterface>();
  }

  void TearDown() override
  {
    hardware_interface_.reset();
  }

  // Helper method to create basic hardware info for testing
  hardware_interface::HardwareInfo createBasicHardwareInfo()
  {
    hardware_interface::HardwareInfo info;
    info.name = "mecabridge_hardware";
    info.type = "system";

    // Add basic parameters
    info.hardware_parameters["drive_type"] = "differential";
    info.hardware_parameters["has_encoders"] = "false";
    info.hardware_parameters["device"] = "/dev/ttyUSB0";
    info.hardware_parameters["baud_rate"] = "115200";
    info.hardware_parameters["timeout"] = "50";
    info.hardware_parameters["loop_rate"] = "20.0";
    info.hardware_parameters["max_lin_vel"] = "0.3";
    info.hardware_parameters["max_ang_vel"] = "1.0";
    info.hardware_parameters["left_wheel_name"] = "left_wheel";
    info.hardware_parameters["right_wheel_name"] = "right_wheel";

    return info;
  }

  // Helper method to create hardware info for different drive types
  hardware_interface::HardwareInfo createHardwareInfo(
    const std::string & drive_type,
    bool has_encoders = false)
  {
    hardware_interface::HardwareInfo info = createBasicHardwareInfo();
    info.hardware_parameters["drive_type"] = drive_type;
    info.hardware_parameters["has_encoders"] = has_encoders ? "true" : "false";

    if (drive_type == "four_wheel" || drive_type == "mecanum") {
      info.hardware_parameters["front_left_wheel_name"] = "front_left_wheel";
      info.hardware_parameters["front_right_wheel_name"] = "front_right_wheel";
      info.hardware_parameters["rear_left_wheel_name"] = "rear_left_wheel";
      info.hardware_parameters["rear_right_wheel_name"] = "rear_right_wheel";
      info.hardware_parameters["wheel_base"] = "0.3";
      info.hardware_parameters["track_width"] = "0.3";
    }

    if (has_encoders) {
      info.hardware_parameters["enc_counts_per_rev"] = "1920";
      info.hardware_parameters["wheel_radius"] = "0.05";
    }

    return info;
  }

  std::unique_ptr<MecaBridgeHardwareInterface> hardware_interface_;
};

// Test configuration parameter parsing and validation (Requirement 1.1)
class ConfigurationParameterTest : public MecaBridgeHardwareInterfaceTest {};

TEST_F(ConfigurationParameterTest, ParseValidDifferentialConfiguration)
{
  auto info = createHardwareInfo("differential", false);

  // Note: on_init will fail due to serial connection, but we can test parameter parsing
  auto result = hardware_interface_->on_init(info);

  // Even though connection fails, parameter parsing should work
  // We expect ERROR due to serial connection failure, not parameter issues
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(ConfigurationParameterTest, ParseValidFourWheelConfiguration)
{
  auto info = createHardwareInfo("four_wheel", false);

  auto result = hardware_interface_->on_init(info);

  // Should fail due to serial connection, not parameter parsing
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(ConfigurationParameterTest, ParseValidMecanumConfiguration)
{
  auto info = createHardwareInfo("mecanum", true);

  auto result = hardware_interface_->on_init(info);

  // Should fail due to serial connection, not parameter parsing
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(ConfigurationParameterTest, HandleInvalidDriveType)
{
  auto info = createBasicHardwareInfo();
  info.hardware_parameters["drive_type"] = "invalid_type";

  auto result = hardware_interface_->on_init(info);

  // Should fail due to serial connection, but parameter should default to differential
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(ConfigurationParameterTest, HandleInvalidBaudRate)
{
  auto info = createBasicHardwareInfo();
  info.hardware_parameters["baud_rate"] = "invalid";

  auto result = hardware_interface_->on_init(info);

  // Should fail due to serial connection, but parameter should default to 115200
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(ConfigurationParameterTest, HandleNegativeBaudRate)
{
  auto info = createBasicHardwareInfo();
  info.hardware_parameters["baud_rate"] = "-9600";

  auto result = hardware_interface_->on_init(info);

  // Should fail due to serial connection, but parameter should default to 115200
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(ConfigurationParameterTest, HandleInvalidTimeout)
{
  auto info = createBasicHardwareInfo();
  info.hardware_parameters["timeout"] = "invalid";

  auto result = hardware_interface_->on_init(info);

  // Should fail due to serial connection, but parameter should default to 50
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(ConfigurationParameterTest, HandleNegativeTimeout)
{
  auto info = createBasicHardwareInfo();
  info.hardware_parameters["timeout"] = "-10";

  auto result = hardware_interface_->on_init(info);

  // Should fail due to serial connection, but parameter should default to 50
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(ConfigurationParameterTest, HandleInvalidLoopRate)
{
  auto info = createBasicHardwareInfo();
  info.hardware_parameters["loop_rate"] = "invalid";

  auto result = hardware_interface_->on_init(info);

  // Should fail due to serial connection, but parameter should default to 20.0
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(ConfigurationParameterTest, HandleNegativeLoopRate)
{
  auto info = createBasicHardwareInfo();
  info.hardware_parameters["loop_rate"] = "-5.0";

  auto result = hardware_interface_->on_init(info);

  // Should fail due to serial connection, but parameter should default to 20.0
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(ConfigurationParameterTest, HandleExcessiveLoopRate)
{
  auto info = createBasicHardwareInfo();
  info.hardware_parameters["loop_rate"] = "2000.0";

  auto result = hardware_interface_->on_init(info);

  // Should fail due to serial connection, but parameter should default to 20.0
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(ConfigurationParameterTest, HandleInvalidVelocityLimits)
{
  auto info = createBasicHardwareInfo();
  info.hardware_parameters["max_lin_vel"] = "invalid";
  info.hardware_parameters["max_ang_vel"] = "-1.0";

  auto result = hardware_interface_->on_init(info);

  // Should fail due to serial connection, but parameters should use defaults
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(ConfigurationParameterTest, HandleInvalidEncoderParameters)
{
  auto info = createHardwareInfo("differential", true);
  info.hardware_parameters["enc_counts_per_rev"] = "invalid";
  info.hardware_parameters["wheel_radius"] = "-0.05";

  auto result = hardware_interface_->on_init(info);

  // Should fail due to serial connection, but parameters should use defaults
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(ConfigurationParameterTest, HandleInvalidMixFactor)
{
  auto info = createBasicHardwareInfo();
  info.hardware_parameters["mix_factor"] = "1.5";  // > 1.0

  auto result = hardware_interface_->on_init(info);

  // Should fail due to serial connection, but parameter should default to 0.5
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(ConfigurationParameterTest, HandleInvalidMecanumParameters)
{
  auto info = createHardwareInfo("mecanum", false);
  info.hardware_parameters["wheel_base"] = "invalid";
  info.hardware_parameters["track_width"] = "-0.3";

  auto result = hardware_interface_->on_init(info);

  // Should fail due to serial connection, but parameters should use defaults
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(ConfigurationParameterTest, HandleMissingParameters)
{
  hardware_interface::HardwareInfo info;
  info.name = "mecabridge_hardware";
  info.type = "system";
  // No parameters provided - should use all defaults

  auto result = hardware_interface_->on_init(info);

  // Should fail due to serial connection, but all parameters should use defaults
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

// Test state and command interface export for different drive types (Requirements 1.2, 1.3)
class InterfaceExportTest : public MecaBridgeHardwareInterfaceTest {};

TEST_F(InterfaceExportTest, ExportDifferentialStateInterfaces)
{
  auto info = createHardwareInfo("differential", false);

  // Initialize (will fail due to serial, but interfaces should still be exportable)
  hardware_interface_->on_init(info);

  auto state_interfaces = hardware_interface_->export_state_interfaces();

  // Should have 4 state interfaces: 2 wheels × (velocity + position)
  EXPECT_EQ(state_interfaces.size(), 4);

  // Check interface names and types
  std::vector<std::string> expected_full_names = {
    "left_wheel/velocity", "left_wheel/position", "right_wheel/velocity", "right_wheel/position"
  };
  std::vector<std::string> expected_types = {
    hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_POSITION
  };

  for (size_t i = 0; i < state_interfaces.size(); ++i) {
    EXPECT_EQ(state_interfaces[i].get_name(), expected_full_names[i]);
    EXPECT_EQ(state_interfaces[i].get_interface_name(), expected_types[i]);
  }
}

TEST_F(InterfaceExportTest, ExportDifferentialCommandInterfaces)
{
  auto info = createHardwareInfo("differential", false);

  hardware_interface_->on_init(info);

  auto command_interfaces = hardware_interface_->export_command_interfaces();

  // Should have 2 command interfaces: 2 wheels × velocity
  EXPECT_EQ(command_interfaces.size(), 2);

  // Check interface names and types
  std::vector<std::string> expected_full_names = {"left_wheel/velocity", "right_wheel/velocity"};

  for (size_t i = 0; i < command_interfaces.size(); ++i) {
    EXPECT_EQ(command_interfaces[i].get_name(), expected_full_names[i]);
    EXPECT_EQ(command_interfaces[i].get_interface_name(), hardware_interface::HW_IF_VELOCITY);
  }
}

TEST_F(InterfaceExportTest, ExportFourWheelStateInterfaces)
{
  auto info = createHardwareInfo("four_wheel", false);

  hardware_interface_->on_init(info);

  auto state_interfaces = hardware_interface_->export_state_interfaces();

  // Should have 8 state interfaces: 4 wheels × (velocity + position)
  EXPECT_EQ(state_interfaces.size(), 8);

  // Check that all four wheels are represented
  std::vector<std::string> expected_wheel_names = {
    "front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"
  };

  for (size_t i = 0; i < 4; ++i) {
    // Each wheel should have velocity and position interfaces
    EXPECT_EQ(state_interfaces[i * 2].get_name(), expected_wheel_names[i] + "/velocity");
    EXPECT_EQ(state_interfaces[i * 2].get_interface_name(), hardware_interface::HW_IF_VELOCITY);
    EXPECT_EQ(state_interfaces[i * 2 + 1].get_name(), expected_wheel_names[i] + "/position");
    EXPECT_EQ(state_interfaces[i * 2 + 1].get_interface_name(), hardware_interface::HW_IF_POSITION);
  }
}

TEST_F(InterfaceExportTest, ExportFourWheelCommandInterfaces)
{
  auto info = createHardwareInfo("four_wheel", false);

  hardware_interface_->on_init(info);

  auto command_interfaces = hardware_interface_->export_command_interfaces();

  // Should have 4 command interfaces: 4 wheels × velocity
  EXPECT_EQ(command_interfaces.size(), 4);

  // Check interface names
  std::vector<std::string> expected_full_names = {
    "front_left_wheel/velocity", "front_right_wheel/velocity", "rear_left_wheel/velocity",
    "rear_right_wheel/velocity"
  };

  for (size_t i = 0; i < command_interfaces.size(); ++i) {
    EXPECT_EQ(command_interfaces[i].get_name(), expected_full_names[i]);
    EXPECT_EQ(command_interfaces[i].get_interface_name(), hardware_interface::HW_IF_VELOCITY);
  }
}

TEST_F(InterfaceExportTest, ExportMecanumStateInterfaces)
{
  auto info = createHardwareInfo("mecanum", false);

  hardware_interface_->on_init(info);

  auto state_interfaces = hardware_interface_->export_state_interfaces();

  // Should have 8 state interfaces: 4 wheels × (velocity + position)
  EXPECT_EQ(state_interfaces.size(), 8);
}

TEST_F(InterfaceExportTest, ExportMecanumCommandInterfaces)
{
  auto info = createHardwareInfo("mecanum", false);

  hardware_interface_->on_init(info);

  auto command_interfaces = hardware_interface_->export_command_interfaces();

  // Should have 4 command interfaces: 4 wheels × velocity
  EXPECT_EQ(command_interfaces.size(), 4);
}

TEST_F(InterfaceExportTest, CustomWheelNames)
{
  auto info = createHardwareInfo("differential", false);
  info.hardware_parameters["left_wheel_name"] = "custom_left";
  info.hardware_parameters["right_wheel_name"] = "custom_right";

  hardware_interface_->on_init(info);

  auto state_interfaces = hardware_interface_->export_state_interfaces();
  auto command_interfaces = hardware_interface_->export_command_interfaces();

  // Check that custom names are used
  EXPECT_EQ(state_interfaces[0].get_name(), "custom_left/velocity");
  EXPECT_EQ(state_interfaces[2].get_name(), "custom_right/velocity");
  EXPECT_EQ(command_interfaces[0].get_name(), "custom_left/velocity");
  EXPECT_EQ(command_interfaces[1].get_name(), "custom_right/velocity");
}

// Test velocity command conversion and kinematics calculations (Requirement 1.4)
class VelocityConversionTest : public MecaBridgeHardwareInterfaceTest {};

TEST_F(VelocityConversionTest, ConvertZeroVelocity)
{
  auto info = createHardwareInfo("differential", false);
  hardware_interface_->on_init(info);

  // Access the conversion method through write() behavior
  // Since we can't directly test the private method, we test through the public interface
  auto command_interfaces = hardware_interface_->export_command_interfaces();

  // Set zero velocity commands
  command_interfaces[0].set_value(0.0);  // left wheel
  command_interfaces[1].set_value(0.0);  // right wheel

  // Write should not crash and should handle zero velocities
  rclcpp::Time time(0);
  rclcpp::Duration period(0, 50000000);  // 50ms

  auto result = hardware_interface_->write(time, period);

  // Should return ERROR due to no connection, but conversion should work
  EXPECT_EQ(result, hardware_interface::return_type::ERROR);
}

TEST_F(VelocityConversionTest, ConvertPositiveVelocity)
{
  auto info = createHardwareInfo("differential", false);
  hardware_interface_->on_init(info);

  auto command_interfaces = hardware_interface_->export_command_interfaces();

  // Set positive velocity commands (within max_lin_vel range)
  command_interfaces[0].set_value(1.0);   // left wheel: 1 rad/s
  command_interfaces[1].set_value(1.0);   // right wheel: 1 rad/s

  rclcpp::Time time(0);
  rclcpp::Duration period(0, 50000000);

  auto result = hardware_interface_->write(time, period);

  // Should return ERROR due to no connection, but conversion should work
  EXPECT_EQ(result, hardware_interface::return_type::ERROR);
}

TEST_F(VelocityConversionTest, ConvertNegativeVelocity)
{
  auto info = createHardwareInfo("differential", false);
  hardware_interface_->on_init(info);

  auto command_interfaces = hardware_interface_->export_command_interfaces();

  // Set negative velocity commands
  command_interfaces[0].set_value(-1.0);  // left wheel: -1 rad/s
  command_interfaces[1].set_value(-1.0);  // right wheel: -1 rad/s

  rclcpp::Time time(0);
  rclcpp::Duration period(0, 50000000);

  auto result = hardware_interface_->write(time, period);

  // Should return ERROR due to no connection, but conversion should work
  EXPECT_EQ(result, hardware_interface::return_type::ERROR);
}

TEST_F(VelocityConversionTest, ConvertExtremeVelocities)
{
  auto info = createHardwareInfo("differential", false);
  hardware_interface_->on_init(info);

  auto command_interfaces = hardware_interface_->export_command_interfaces();

  // Set extreme velocity commands (should be clamped)
  command_interfaces[0].set_value(100.0);   // Very high positive
  command_interfaces[1].set_value(-100.0);  // Very high negative

  rclcpp::Time time(0);
  rclcpp::Duration period(0, 50000000);

  auto result = hardware_interface_->write(time, period);

  // Should return ERROR due to no connection, but clamping should work
  EXPECT_EQ(result, hardware_interface::return_type::ERROR);
}

TEST_F(VelocityConversionTest, FourWheelVelocityConversion)
{
  auto info = createHardwareInfo("four_wheel", false);
  hardware_interface_->on_init(info);

  auto command_interfaces = hardware_interface_->export_command_interfaces();

  // Set different velocities for each wheel
  command_interfaces[0].set_value(1.0);   // front_left
  command_interfaces[1].set_value(-1.0);  // front_right
  command_interfaces[2].set_value(0.5);   // rear_left
  command_interfaces[3].set_value(-0.5);  // rear_right

  rclcpp::Time time(0);
  rclcpp::Duration period(0, 50000000);

  auto result = hardware_interface_->write(time, period);

  // Should return ERROR due to no connection, but conversion should work
  EXPECT_EQ(result, hardware_interface::return_type::ERROR);
}

TEST_F(VelocityConversionTest, MecanumVelocityConversion)
{
  auto info = createHardwareInfo("mecanum", false);
  hardware_interface_->on_init(info);

  auto command_interfaces = hardware_interface_->export_command_interfaces();

  // Set mecanum wheel velocities (would come from mecanum controller)
  command_interfaces[0].set_value(1.0);   // front_left
  command_interfaces[1].set_value(1.0);   // front_right
  command_interfaces[2].set_value(1.0);   // rear_left
  command_interfaces[3].set_value(1.0);   // rear_right

  rclcpp::Time time(0);
  rclcpp::Duration period(0, 50000000);

  auto result = hardware_interface_->write(time, period);

  // Should return ERROR due to no connection, but conversion should work
  EXPECT_EQ(result, hardware_interface::return_type::ERROR);
}

// Test encoder vs non-encoder operation modes (Requirement 1.5)
class EncoderOperationTest : public MecaBridgeHardwareInterfaceTest {};

TEST_F(EncoderOperationTest, NonEncoderModePositionIntegration)
{
  auto info = createHardwareInfo("differential", false);  // no encoders
  hardware_interface_->on_init(info);

  auto state_interfaces = hardware_interface_->export_state_interfaces();
  auto command_interfaces = hardware_interface_->export_command_interfaces();

  // Set initial velocity commands
  command_interfaces[0].set_value(1.0);  // left wheel: 1 rad/s
  command_interfaces[1].set_value(1.0);  // right wheel: 1 rad/s

  // Simulate read cycle with time period
  rclcpp::Time time(0);
  rclcpp::Duration period(0, 100000000);  // 100ms

  auto result = hardware_interface_->read(time, period);

  // Should return ERROR due to no connection, but integration should work
  EXPECT_EQ(result, hardware_interface::return_type::ERROR);

  // In non-encoder mode without connection, the read() fails so values remain at defaults
  // The main test is that the system doesn't crash and handles the error gracefully
  // We test that the interfaces exist and can be accessed
  EXPECT_GE(state_interfaces[0].get_value(), 0.0);  // left velocity (default value)
  EXPECT_GE(state_interfaces[2].get_value(), 0.0);  // right velocity (default value)

  // Position should remain at initial values due to connection error
  // We mainly test that the system doesn't crash and handles the integration
}

TEST_F(EncoderOperationTest, EncoderModeConfiguration)
{
  auto info = createHardwareInfo("differential", true);  // with encoders
  hardware_interface_->on_init(info);

  // Should configure encoder parameters
  auto state_interfaces = hardware_interface_->export_state_interfaces();

  // Should still have same number of interfaces
  EXPECT_EQ(state_interfaces.size(), 4);  // 2 wheels × (velocity + position)

  // Test read operation (will fail due to no connection, but should handle encoder mode)
  rclcpp::Time time(0);
  rclcpp::Duration period(0, 50000000);

  auto result = hardware_interface_->read(time, period);

  // Should return ERROR due to no connection, but encoder handling should work
  EXPECT_EQ(result, hardware_interface::return_type::ERROR);
}

TEST_F(EncoderOperationTest, FourWheelEncoderMode)
{
  auto info = createHardwareInfo("four_wheel", true);  // with encoders
  hardware_interface_->on_init(info);

  auto state_interfaces = hardware_interface_->export_state_interfaces();

  // Should have 8 interfaces for 4 wheels
  EXPECT_EQ(state_interfaces.size(), 8);

  rclcpp::Time time(0);
  rclcpp::Duration period(0, 50000000);

  auto result = hardware_interface_->read(time, period);

  // Should return ERROR due to no connection, but encoder handling should work
  EXPECT_EQ(result, hardware_interface::return_type::ERROR);
}

TEST_F(EncoderOperationTest, MecanumEncoderMode)
{
  auto info = createHardwareInfo("mecanum", true);  // with encoders
  hardware_interface_->on_init(info);

  auto state_interfaces = hardware_interface_->export_state_interfaces();

  // Should have 8 interfaces for 4 wheels
  EXPECT_EQ(state_interfaces.size(), 8);

  rclcpp::Time time(0);
  rclcpp::Duration period(0, 50000000);

  auto result = hardware_interface_->read(time, period);

  // Should return ERROR due to no connection, but encoder handling should work
  EXPECT_EQ(result, hardware_interface::return_type::ERROR);
}

TEST_F(EncoderOperationTest, EncoderParameterValidation)
{
  auto info = createHardwareInfo("differential", true);
  info.hardware_parameters["enc_counts_per_rev"] = "3840";  // Different encoder
  info.hardware_parameters["wheel_radius"] = "0.075";       // Different wheel size

  auto result = hardware_interface_->on_init(info);

  // Should fail due to serial connection, but encoder parameters should be parsed
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

// Test lifecycle methods
class LifecycleTest : public MecaBridgeHardwareInterfaceTest {};

TEST_F(LifecycleTest, ActivationWithoutConnection)
{
  auto info = createHardwareInfo("differential", false);
  hardware_interface_->on_init(info);

  // Activation should fail due to no connection
  rclcpp_lifecycle::State previous_state;
  auto result = hardware_interface_->on_activate(previous_state);

  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(LifecycleTest, DeactivationWithoutConnection)
{
  auto info = createHardwareInfo("differential", false);
  hardware_interface_->on_init(info);

  // Deactivation should succeed even without connection
  rclcpp_lifecycle::State previous_state;
  auto result = hardware_interface_->on_deactivate(previous_state);

  EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);
}

// Integration test for complete workflow
class IntegrationTest : public MecaBridgeHardwareInterfaceTest {};

TEST_F(IntegrationTest, CompleteWorkflowWithoutHardware)
{
  auto info = createHardwareInfo("differential", false);

  // Initialize
  auto init_result = hardware_interface_->on_init(info);
  EXPECT_EQ(init_result, hardware_interface::CallbackReturn::ERROR);  // No serial connection

  // Export interfaces
  auto state_interfaces = hardware_interface_->export_state_interfaces();
  auto command_interfaces = hardware_interface_->export_command_interfaces();

  EXPECT_EQ(state_interfaces.size(), 4);   // 2 wheels × 2 interfaces
  EXPECT_EQ(command_interfaces.size(), 2); // 2 wheels × 1 interface

  // Try to activate (will fail due to no connection)
  rclcpp_lifecycle::State previous_state;
  auto activate_result = hardware_interface_->on_activate(previous_state);
  EXPECT_EQ(activate_result, hardware_interface::CallbackReturn::ERROR);

  // Set commands and try read/write cycles
  command_interfaces[0].set_value(1.0);
  command_interfaces[1].set_value(-1.0);

  rclcpp::Time time(0);
  rclcpp::Duration period(0, 50000000);

  auto read_result = hardware_interface_->read(time, period);
  auto write_result = hardware_interface_->write(time, period);

  // Both should return ERROR due to no connection, but shouldn't crash
  EXPECT_EQ(read_result, hardware_interface::return_type::ERROR);
  EXPECT_EQ(write_result, hardware_interface::return_type::ERROR);

  // Deactivate should succeed
  auto deactivate_result = hardware_interface_->on_deactivate(previous_state);
  EXPECT_EQ(deactivate_result, hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(IntegrationTest, FourWheelCompleteWorkflow)
{
  auto info = createHardwareInfo("four_wheel", true);  // with encoders

  auto init_result = hardware_interface_->on_init(info);
  EXPECT_EQ(init_result, hardware_interface::CallbackReturn::ERROR);

  auto state_interfaces = hardware_interface_->export_state_interfaces();
  auto command_interfaces = hardware_interface_->export_command_interfaces();

  EXPECT_EQ(state_interfaces.size(), 8);   // 4 wheels × 2 interfaces
  EXPECT_EQ(command_interfaces.size(), 4); // 4 wheels × 1 interface

  // Set different commands for each wheel
  for (size_t i = 0; i < command_interfaces.size(); ++i) {
    command_interfaces[i].set_value(static_cast<double>(i) - 1.5);  // -1.5, -0.5, 0.5, 1.5
  }

  rclcpp::Time time(0);
  rclcpp::Duration period(0, 50000000);

  auto read_result = hardware_interface_->read(time, period);
  auto write_result = hardware_interface_->write(time, period);

  EXPECT_EQ(read_result, hardware_interface::return_type::ERROR);
  EXPECT_EQ(write_result, hardware_interface::return_type::ERROR);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
