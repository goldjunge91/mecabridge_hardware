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

#include <gtest/gtest.h>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

// Include the actual MecaBridgeSerialProtocol header for testing
#include "mecabridge_hardware/mecabridge_serial_protocol.h"

class MecaBridgeSerialProtocolSimpleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS 2 for logging
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    // Clean up
  }
};

// Test constructor and basic initialization
TEST_F(MecaBridgeSerialProtocolSimpleTest, TestDefaultConstructor)
{
  EXPECT_NO_THROW(
  {
    MecaBridgeSerialProtocol comms;
    // Default constructor should not throw
    EXPECT_FALSE(comms.connected());
  });
}

TEST_F(MecaBridgeSerialProtocolSimpleTest, TestParameterizedConstructorWithInvalidPort)
{
  // Test that constructor with invalid port handles errors gracefully
  EXPECT_THROW(
  {
    MecaBridgeSerialProtocol comms("/dev/nonexistent_port", 115200, 50);
  }, std::exception);
}

TEST_F(MecaBridgeSerialProtocolSimpleTest, TestSetupWithInvalidPort)
{
  MecaBridgeSerialProtocol comms;

  // Test that setup with invalid port throws appropriate error
  EXPECT_THROW(
  {
    comms.setup("/dev/nonexistent_port", 115200, 50);
  }, std::exception);

  EXPECT_FALSE(comms.connected());
}

TEST_F(MecaBridgeSerialProtocolSimpleTest, TestSetupWithEmptyPortAutoDetection)
{
  MecaBridgeSerialProtocol comms;

  // Test auto-detection when no port is specified
  // This should fail in test environment but not crash
  EXPECT_THROW(
  {
    comms.setup("", 115200, 50);
  }, std::runtime_error);

  EXPECT_FALSE(comms.connected());
}

TEST_F(MecaBridgeSerialProtocolSimpleTest, TestSetupWithInvalidParameters)
{
  MecaBridgeSerialProtocol comms;

  // Test that invalid parameters are handled (should not crash)
  EXPECT_THROW(
  {
    comms.setup("/dev/nonexistent_port", -1, -1);
  }, std::exception);

  EXPECT_FALSE(comms.connected());
}

// Test motor command methods when not connected
TEST_F(MecaBridgeSerialProtocolSimpleTest, TestMotorCommandsWhenDisconnected)
{
  MecaBridgeSerialProtocol comms;

  // Motor commands should throw when not connected
  EXPECT_THROW(
  {
    comms.setDifferentialMotors(50, -30);
  }, std::runtime_error);

  EXPECT_THROW(
  {
    comms.setFourMotors(25, -50, 75, -25);
  }, std::runtime_error);
}

// Test encoder reading methods when not connected
TEST_F(MecaBridgeSerialProtocolSimpleTest, TestEncoderReadingWhenDisconnected)
{
  MecaBridgeSerialProtocol comms;

  int left_enc, right_enc;
  // Encoder reading should throw when not connected
  EXPECT_THROW(
  {
    comms.readDifferentialEncoders(left_enc, right_enc);
  }, std::runtime_error);

  int fl, fr, rl, rr;
  EXPECT_THROW(
  {
    comms.readFourEncoders(fl, fr, rl, rr);
  }, std::runtime_error);
}

// Test ping functionality when not connected
TEST_F(MecaBridgeSerialProtocolSimpleTest, TestPingWhenDisconnected)
{
  MecaBridgeSerialProtocol comms;

  // Ping should not throw even when disconnected (it logs warning)
  EXPECT_NO_THROW(
  {
    comms.sendPing();
  });
}

// Test connection status
TEST_F(MecaBridgeSerialProtocolSimpleTest, TestConnectionStatus)
{
  MecaBridgeSerialProtocol comms;

  // Should be disconnected initially
  EXPECT_FALSE(comms.connected());
}

// Test reconnection attempt when not connected
TEST_F(MecaBridgeSerialProtocolSimpleTest, TestReconnectionWhenDisconnected)
{
  MecaBridgeSerialProtocol comms;

  // Reconnection should fail when never connected
  EXPECT_FALSE(comms.attemptReconnection());
  EXPECT_FALSE(comms.connected());
}

// Test error statistics
TEST_F(MecaBridgeSerialProtocolSimpleTest, TestErrorStatistics)
{
  MecaBridgeSerialProtocol comms;

  int write_errors, read_errors, reconnection_attempts;
  comms.getConnectionStats(write_errors, read_errors, reconnection_attempts);

  // Initially should be zero
  EXPECT_EQ(write_errors, 0);
  EXPECT_EQ(read_errors, 0);
  EXPECT_EQ(reconnection_attempts, 0);

  // Test reset functionality
  EXPECT_NO_THROW(
  {
    comms.resetErrorCounters();
  });

  // Should still be zero after reset
  comms.getConnectionStats(write_errors, read_errors, reconnection_attempts);
  EXPECT_EQ(write_errors, 0);
  EXPECT_EQ(read_errors, 0);
  EXPECT_EQ(reconnection_attempts, 0);
}

// Test motor command formatting (we can test the logic without actual serial connection)
class MecaBridgeSerialProtocolFormatTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

// Test value clamping logic by examining expected behavior
TEST_F(MecaBridgeSerialProtocolFormatTest, TestMotorValueClamping)
{
  MecaBridgeSerialProtocol comms;

  // We can't test the actual serial output, but we can test that
  // the methods handle extreme values without crashing
  EXPECT_THROW(
  {
    comms.setDifferentialMotors(200, -200);  // Should clamp to [-100, 100]
  }, std::runtime_error);  // Throws because not connected

  EXPECT_THROW(
  {
    comms.setFourMotors(150, -150, 200, -200);  // Should clamp to [-100, 100]
  }, std::runtime_error);  // Throws because not connected
}

TEST_F(MecaBridgeSerialProtocolFormatTest, TestBoundaryValues)
{
  MecaBridgeSerialProtocol comms;

  // Test boundary values
  EXPECT_THROW(
  {
    comms.setDifferentialMotors(-100, 100);  // Exact boundaries
  }, std::runtime_error);  // Throws because not connected

  EXPECT_THROW(
  {
    comms.setDifferentialMotors(0, 0);  // Zero values
  }, std::runtime_error);  // Throws because not connected
}

// Integration test class for testing with mock behavior
class MecaBridgeSerialProtocolIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

TEST_F(MecaBridgeSerialProtocolIntegrationTest, TestCompleteWorkflow)
{
  MecaBridgeSerialProtocol comms;

  // Test complete workflow from construction to usage
  EXPECT_FALSE(comms.connected());

  // Attempt setup (will fail in test environment)
  EXPECT_THROW(
  {
    comms.setup("", 115200, 50);
  }, std::runtime_error);

  // Should still be disconnected
  EXPECT_FALSE(comms.connected());

  // Commands should fail
  EXPECT_THROW(
  {
    comms.setDifferentialMotors(50, -30);
  }, std::runtime_error);

  // Reconnection should fail
  EXPECT_FALSE(comms.attemptReconnection());

  // Error stats should show reconnection attempt
  int write_errors, read_errors, reconnection_attempts;
  comms.getConnectionStats(write_errors, read_errors, reconnection_attempts);
  EXPECT_GT(reconnection_attempts, 0);
}

// Test serial port detection logic
TEST_F(MecaBridgeSerialProtocolIntegrationTest, TestSerialPortDetection)
{
  MecaBridgeSerialProtocol comms;

  // Test that auto-detection handles the case where no ports are available
  EXPECT_THROW(
  {
    comms.setup("", 115200, 50);
  }, std::runtime_error);

  // Test with specific invalid port
  EXPECT_THROW(
  {
    comms.setup("/dev/invalid_port", 115200, 50);
  }, std::exception);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
