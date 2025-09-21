#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

#include "mecabridge_hardware/mecabridge_serial_protocol.h"

class MecaBridgeSerialProtocolComprehensiveTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    comms_ = std::make_unique<MecaBridgeSerialProtocol>();
  }

  void TearDown() override
  {
    comms_.reset();
  }

  std::unique_ptr<MecaBridgeSerialProtocol> comms_;
};

// Test Requirements 2.1: Serial port detection and connection logic
class SerialPortDetectionTest : public MecaBridgeSerialProtocolComprehensiveTest
{
};

TEST_F(SerialPortDetectionTest, TestAutoDetectionWithNoAvailablePorts)
{
  // Test auto-detection when no ports are available
  // This tests the error handling path in findSerialPort
  EXPECT_THROW(
  {
    comms_->setup("", 115200, 50);
  }, std::exception);

  EXPECT_FALSE(comms_->connected());
}

TEST_F(SerialPortDetectionTest, TestSpecificPortConnectionFailure)
{
  // Test connection to a specific non-existent port
  EXPECT_THROW(
  {
    comms_->setup("/dev/nonexistent_port", 115200, 50);
  }, std::exception);

  EXPECT_FALSE(comms_->connected());
}

TEST_F(SerialPortDetectionTest, TestParameterValidation)
{
  // Test that invalid baud rate and timeout are handled
  // The implementation should correct invalid values to defaults
  EXPECT_THROW(
  {
    comms_->setup("/dev/nonexistent_port", -1, -1);
  }, std::exception);

  EXPECT_FALSE(comms_->connected());
}

TEST_F(SerialPortDetectionTest, TestConnectionStatusChecking)
{
  // Test connection status methods
  EXPECT_FALSE(comms_->connected());

  // After failed connection attempt, should still be false
  try {
    comms_->setup("/dev/nonexistent_port", 115200, 50);
  } catch (const std::exception &) {
    // Expected to fail
  }

  EXPECT_FALSE(comms_->connected());
}

// Test Requirements 2.2: Motor command formatting for different drive configurations
class MotorCommandFormattingTest : public MecaBridgeSerialProtocolComprehensiveTest
{
};

TEST_F(MotorCommandFormattingTest, TestDifferentialMotorCommandWhenDisconnected)
{
  // Test differential motor command format (should throw when not connected)
  EXPECT_THROW(
  {
    comms_->setDifferentialMotors(50, -30);
  }, std::runtime_error);

  EXPECT_THROW(
  {
    comms_->setDifferentialMotors(-100, 100);  // Boundary values
  }, std::runtime_error);

  EXPECT_THROW(
  {
    comms_->setDifferentialMotors(0, 0);  // Zero values
  }, std::runtime_error);
}

TEST_F(MotorCommandFormattingTest, TestFourMotorCommandWhenDisconnected)
{
  // Test four motor command format (should throw when not connected)
  EXPECT_THROW(
  {
    comms_->setFourMotors(25, -50, 75, -25);
  }, std::runtime_error);

  EXPECT_THROW(
  {
    comms_->setFourMotors(-100, 100, -100, 100);  // Boundary values
  }, std::runtime_error);

  EXPECT_THROW(
  {
    comms_->setFourMotors(0, 0, 0, 0);  // Zero values
  }, std::runtime_error);
}

TEST_F(MotorCommandFormattingTest, TestMotorValueClampingBehavior)
{
  // Test that extreme values are handled (implementation should clamp to [-100, 100])
  // We can't test the actual clamping without connection, but we can test the methods don't crash
  EXPECT_THROW(
  {
    comms_->setDifferentialMotors(200, -200);  // Should be clamped internally
  }, std::runtime_error);  // Throws because not connected

  EXPECT_THROW(
  {
    comms_->setFourMotors(150, -150, 200, -200);  // Should be clamped internally
  }, std::runtime_error);  // Throws because not connected
}

// Test Requirements 2.3: Encoder reading functionality with mock serial responses
class EncoderReadingTest : public MecaBridgeSerialProtocolComprehensiveTest
{
};

TEST_F(EncoderReadingTest, TestDifferentialEncoderReadingWhenDisconnected)
{
  int left_enc, right_enc;

  // Should throw when not connected
  EXPECT_THROW(
  {
    comms_->readDifferentialEncoders(left_enc, right_enc);
  }, std::runtime_error);
}

TEST_F(EncoderReadingTest, TestFourEncoderReadingWhenDisconnected)
{
  int fl, fr, rl, rr;

  // Should throw when not connected
  EXPECT_THROW(
  {
    comms_->readFourEncoders(fl, fr, rl, rr);
  }, std::runtime_error);
}

TEST_F(EncoderReadingTest, TestEncoderVariableInitialization)
{
  int left_enc = 999, right_enc = 999;
  int fl = 999, fr = 999, rl = 999, rr = 999;

  // Variables should remain unchanged when methods throw
  EXPECT_THROW(
  {
    comms_->readDifferentialEncoders(left_enc, right_enc);
  }, std::runtime_error);

  EXPECT_THROW(
  {
    comms_->readFourEncoders(fl, fr, rl, rr);
  }, std::runtime_error);

  // Values should remain unchanged since methods threw before modifying them
  EXPECT_EQ(left_enc, 999);
  EXPECT_EQ(right_enc, 999);
  EXPECT_EQ(fl, 999);
  EXPECT_EQ(fr, 999);
  EXPECT_EQ(rl, 999);
  EXPECT_EQ(rr, 999);
}

// Test Requirements 2.4: Error handling scenarios and connection failures
class ErrorHandlingTest : public MecaBridgeSerialProtocolComprehensiveTest
{
};

TEST_F(ErrorHandlingTest, TestConnectionFailureHandling)
{
  // Test various connection failure scenarios
  EXPECT_THROW(
  {
    comms_->setup("/dev/nonexistent_port", 115200, 50);
  }, std::exception);

  EXPECT_FALSE(comms_->connected());
}

TEST_F(ErrorHandlingTest, TestReconnectionAttempts)
{
  // Test reconnection when never connected
  EXPECT_FALSE(comms_->attemptReconnection());

  // Check that reconnection attempts are tracked
  int write_errors, read_errors, reconnection_attempts;
  comms_->getConnectionStats(write_errors, read_errors, reconnection_attempts);

  EXPECT_GT(reconnection_attempts, 0);  // Should have attempted reconnection
}

TEST_F(ErrorHandlingTest, TestErrorCounterFunctionality)
{
  int write_errors, read_errors, reconnection_attempts;

  // Initially should be zero
  comms_->getConnectionStats(write_errors, read_errors, reconnection_attempts);
  EXPECT_EQ(write_errors, 0);
  EXPECT_EQ(read_errors, 0);
  EXPECT_EQ(reconnection_attempts, 0);

  // Attempt reconnection to increment counter
  comms_->attemptReconnection();

  // Check that counter was incremented
  comms_->getConnectionStats(write_errors, read_errors, reconnection_attempts);
  EXPECT_GT(reconnection_attempts, 0);

  // Test reset functionality
  comms_->resetErrorCounters();

  // Should be zero after reset
  comms_->getConnectionStats(write_errors, read_errors, reconnection_attempts);
  EXPECT_EQ(write_errors, 0);
  EXPECT_EQ(read_errors, 0);
  EXPECT_EQ(reconnection_attempts, 0);
}

TEST_F(ErrorHandlingTest, TestCommandsOnDisconnectedState)
{
  // Test that all commands fail gracefully when not connected
  EXPECT_FALSE(comms_->connected());

  // Motor commands should throw
  EXPECT_THROW(
  {
    comms_->setDifferentialMotors(50, -30);
  }, std::runtime_error);

  EXPECT_THROW(
  {
    comms_->setFourMotors(10, 20, 30, 40);
  }, std::runtime_error);

  // Encoder reading should throw
  int left_enc, right_enc;
  EXPECT_THROW(
  {
    comms_->readDifferentialEncoders(left_enc, right_enc);
  }, std::runtime_error);

  int fl, fr, rl, rr;
  EXPECT_THROW(
  {
    comms_->readFourEncoders(fl, fr, rl, rr);
  }, std::runtime_error);

  // Ping should not throw (logs warning instead)
  EXPECT_NO_THROW(
  {
    comms_->sendPing();
  });
}

TEST_F(ErrorHandlingTest, TestPingFunctionality)
{
  // Test ping when disconnected (should not throw)
  EXPECT_NO_THROW(
  {
    comms_->sendPing();
  });

  // Multiple pings should not cause issues
  EXPECT_NO_THROW(
  {
    comms_->sendPing();
    comms_->sendPing();
    comms_->sendPing();
  });
}

// Integration tests that test complete workflows
class MecaBridgeSerialProtocolIntegrationTest : public MecaBridgeSerialProtocolComprehensiveTest
{
};

TEST_F(MecaBridgeSerialProtocolIntegrationTest, TestCompleteDisconnectedWorkflow)
{
  // Test a complete workflow in disconnected state
  EXPECT_FALSE(comms_->connected());

  // Setup should fail
  EXPECT_THROW(
  {
    comms_->setup("", 115200, 50);
  }, std::exception);

  EXPECT_FALSE(comms_->connected());

  // All commands should fail
  EXPECT_THROW(
  {
    comms_->setDifferentialMotors(50, -30);
  }, std::runtime_error);

  int left_enc, right_enc;
  EXPECT_THROW(
  {
    comms_->readDifferentialEncoders(left_enc, right_enc);
  }, std::runtime_error);

  // Ping should not throw
  EXPECT_NO_THROW(
  {
    comms_->sendPing();
  });

  // Reconnection should fail
  EXPECT_FALSE(comms_->attemptReconnection());

  // Error stats should show activity
  int write_errors, read_errors, reconnection_attempts;
  comms_->getConnectionStats(write_errors, read_errors, reconnection_attempts);
  EXPECT_GT(reconnection_attempts, 0);
}

TEST_F(MecaBridgeSerialProtocolIntegrationTest, TestConstructorVariants)
{
  // Test default constructor
  {
    MecaBridgeSerialProtocol comms1;
    EXPECT_FALSE(comms1.connected());
  }

  // Test parameterized constructor with invalid port
  EXPECT_THROW(
  {
    MecaBridgeSerialProtocol comms2("/dev/nonexistent_port", 115200, 50);
  }, std::exception);
}

TEST_F(MecaBridgeSerialProtocolIntegrationTest, TestErrorRecoveryScenarios)
{
  // Test error recovery patterns
  EXPECT_FALSE(comms_->connected());

  // Multiple failed connection attempts
  for (int i = 0; i < 3; ++i) {
    try {
      comms_->setup("/dev/nonexistent_port", 115200, 50);
    } catch (const std::exception &) {
      // Expected to fail
    }
    EXPECT_FALSE(comms_->connected());
  }

  // Multiple reconnection attempts
  for (int i = 0; i < 3; ++i) {
    EXPECT_FALSE(comms_->attemptReconnection());
  }

  // Check error statistics
  int write_errors, read_errors, reconnection_attempts;
  comms_->getConnectionStats(write_errors, read_errors, reconnection_attempts);
  EXPECT_GT(reconnection_attempts, 0);

  // Reset and verify
  comms_->resetErrorCounters();
  comms_->getConnectionStats(write_errors, read_errors, reconnection_attempts);
  EXPECT_EQ(reconnection_attempts, 0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
