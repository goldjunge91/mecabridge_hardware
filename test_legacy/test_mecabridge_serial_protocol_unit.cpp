#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

// Define TESTING_MODE to enable mock functionality
#define TESTING_MODE

#include "mecabridge_hardware/mecabridge_serial_protocol.h"
#include "mock_serial.h"

using ::testing::_;
using ::testing::Return;
using ::testing::Throw;
using ::testing::InSequence;
using ::testing::StrictMock;

class MecaBridgeSerialProtocolUnitTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS 2 for logging
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    mock_serial_ = std::make_shared<StrictMock<MockSerial>>();
    comms_ = std::make_unique<MecaBridgeSerialProtocol>();

    #ifdef TESTING_MODE
    comms_->setMockSerial(mock_serial_);
    #endif
  }

  void TearDown() override
  {
    comms_.reset();
    mock_serial_.reset();
  }

  std::shared_ptr<StrictMock<MockSerial>> mock_serial_;
  std::unique_ptr<MecaBridgeSerialProtocol> comms_;
};

// Test Requirements 2.1: Serial port detection and connection logic
class SerialPortDetectionUnitTest : public MecaBridgeSerialProtocolUnitTest
{
};

TEST_F(SerialPortDetectionUnitTest, TestSuccessfulConnection)
{
  // Test successful connection to a specific port
  EXPECT_CALL(*mock_serial_, setPort("/dev/ttyUSB0"));
  EXPECT_CALL(*mock_serial_, setBaudrate(115200));
  EXPECT_CALL(*mock_serial_, setTimeout(_));
  EXPECT_CALL(*mock_serial_, open());
  EXPECT_CALL(*mock_serial_, isOpen())
  .WillOnce(Return(true));

  // Expect PING to be sent after connection
  EXPECT_CALL(*mock_serial_, write("PING\n"))
  .WillOnce(Return(5));

  EXPECT_NO_THROW(
  {
    comms_->setup("/dev/ttyUSB0", 115200, 50);
  });

  EXPECT_TRUE(comms_->connected());
}

TEST_F(SerialPortDetectionUnitTest, TestConnectionFailure)
{
  // Test connection failure
  EXPECT_CALL(*mock_serial_, setPort("/dev/nonexistent"));
  EXPECT_CALL(*mock_serial_, setBaudrate(115200));
  EXPECT_CALL(*mock_serial_, setTimeout(_));
  EXPECT_CALL(*mock_serial_, open())
  .WillOnce(Throw(std::runtime_error("Port not found")));

  EXPECT_THROW(
  {
    comms_->setup("/dev/nonexistent", 115200, 50);
  }, std::exception);

  EXPECT_FALSE(comms_->connected());
}

TEST_F(SerialPortDetectionUnitTest, TestAutoDetectionSuccess)
{
  // Test auto-detection when first port fails, second succeeds
  InSequence seq;

  // First port attempt fails
  EXPECT_CALL(*mock_serial_, setPort("/dev/ttyUSB0"));
  EXPECT_CALL(*mock_serial_, setBaudrate(115200));
  EXPECT_CALL(*mock_serial_, setTimeout(_));
  EXPECT_CALL(*mock_serial_, open())
  .WillOnce(Throw(std::runtime_error("Port busy")));

  // Second port attempt succeeds
  EXPECT_CALL(*mock_serial_, setPort("/dev/ttyUSB1"));
  EXPECT_CALL(*mock_serial_, setBaudrate(115200));
  EXPECT_CALL(*mock_serial_, setTimeout(_));
  EXPECT_CALL(*mock_serial_, open());
  EXPECT_CALL(*mock_serial_, isOpen())
  .WillOnce(Return(true));

  // Expect PING after successful connection
  EXPECT_CALL(*mock_serial_, write("PING\n"))
  .WillOnce(Return(5));

  EXPECT_NO_THROW(
  {
    comms_->setup("", 115200, 50);  // Empty string triggers auto-detection
  });

  EXPECT_TRUE(comms_->connected());
}

TEST_F(SerialPortDetectionUnitTest, TestAutoDetectionFailure)
{
  // Test auto-detection when all ports fail
  for (int i = 0; i < 4; ++i) {  // Test all candidate ports
    EXPECT_CALL(*mock_serial_, setPort(_));
    EXPECT_CALL(*mock_serial_, setBaudrate(115200));
    EXPECT_CALL(*mock_serial_, setTimeout(_));
    EXPECT_CALL(*mock_serial_, open())
    .WillOnce(Throw(std::runtime_error("Port not available")));
  }

  EXPECT_THROW(
  {
    comms_->setup("", 115200, 50);
  }, std::runtime_error);

  EXPECT_FALSE(comms_->connected());
}

TEST_F(SerialPortDetectionUnitTest, TestParameterValidation)
{
  // Test parameter validation and correction
  EXPECT_CALL(*mock_serial_, setPort("/dev/ttyUSB0"));
  EXPECT_CALL(*mock_serial_, setBaudrate(115200));  // Should be corrected from -1
  EXPECT_CALL(*mock_serial_, setTimeout(_));
  EXPECT_CALL(*mock_serial_, open());
  EXPECT_CALL(*mock_serial_, isOpen())
  .WillOnce(Return(true));
  EXPECT_CALL(*mock_serial_, write("PING\n"))
  .WillOnce(Return(5));

  // Invalid parameters should be corrected to defaults
  EXPECT_NO_THROW(
  {
    comms_->setup("/dev/ttyUSB0", -1, -1);  // Invalid baud rate and timeout
  });

  EXPECT_TRUE(comms_->connected());
}

// Test Requirements 2.2: Motor command formatting for different drive configurations
class MotorCommandFormattingUnitTest : public MecaBridgeSerialProtocolUnitTest
{
protected:
  void SetUp() override
  {
    MecaBridgeSerialProtocolUnitTest::SetUp();

    // Set up a connected state
    EXPECT_CALL(*mock_serial_, setPort(_));
    EXPECT_CALL(*mock_serial_, setBaudrate(_));
    EXPECT_CALL(*mock_serial_, setTimeout(_));
    EXPECT_CALL(*mock_serial_, open());
    EXPECT_CALL(*mock_serial_, isOpen())
    .WillRepeatedly(Return(true));
    EXPECT_CALL(*mock_serial_, write("PING\n"))
    .WillOnce(Return(5));

    comms_->setup("/dev/ttyUSB0", 115200, 50);
  }
};

TEST_F(MotorCommandFormattingUnitTest, TestDifferentialMotorCommandFormat)
{
  // Test differential motor command formatting: "V {left} {right}\n"
  EXPECT_CALL(*mock_serial_, write("V 50 -30\n"))
  .WillOnce(Return(9));

  EXPECT_NO_THROW(
  {
    comms_->setDifferentialMotors(50, -30);
  });
}

TEST_F(MotorCommandFormattingUnitTest, TestDifferentialMotorBoundaryValues)
{
  // Test boundary values
  EXPECT_CALL(*mock_serial_, write("V -100 100\n"))
  .WillOnce(Return(11));

  EXPECT_NO_THROW(
  {
    comms_->setDifferentialMotors(-100, 100);
  });

  // Test zero values
  EXPECT_CALL(*mock_serial_, write("V 0 0\n"))
  .WillOnce(Return(7));

  EXPECT_NO_THROW(
  {
    comms_->setDifferentialMotors(0, 0);
  });
}

TEST_F(MotorCommandFormattingUnitTest, TestDifferentialMotorValueClamping)
{
  // Test that values outside [-100, 100] are clamped
  EXPECT_CALL(*mock_serial_, write("V 100 -100\n"))  // Should be clamped
  .WillOnce(Return(12));

  EXPECT_NO_THROW(
  {
    comms_->setDifferentialMotors(200, -200);  // Should be clamped to [100, -100]
  });
}

TEST_F(MotorCommandFormattingUnitTest, TestFourMotorCommandFormat)
{
  // Test four motor command formatting: "M {fl} {fr} {rl} {rr}\n"
  EXPECT_CALL(*mock_serial_, write("M 25 -50 75 -25\n"))
  .WillOnce(Return(16));

  EXPECT_NO_THROW(
  {
    comms_->setFourMotors(25, -50, 75, -25);
  });
}

TEST_F(MotorCommandFormattingUnitTest, TestFourMotorBoundaryValues)
{
  // Test boundary values
  EXPECT_CALL(*mock_serial_, write("M -100 100 -100 100\n"))
  .WillOnce(Return(21));

  EXPECT_NO_THROW(
  {
    comms_->setFourMotors(-100, 100, -100, 100);
  });

  // Test zero values
  EXPECT_CALL(*mock_serial_, write("M 0 0 0 0\n"))
  .WillOnce(Return(11));

  EXPECT_NO_THROW(
  {
    comms_->setFourMotors(0, 0, 0, 0);
  });
}

TEST_F(MotorCommandFormattingUnitTest, TestFourMotorValueClamping)
{
  // Test that values outside [-100, 100] are clamped
  EXPECT_CALL(*mock_serial_, write("M 100 -100 100 -100\n"))  // Should be clamped
  .WillOnce(Return(22));

  EXPECT_NO_THROW(
  {
    comms_->setFourMotors(150, -150, 200, -200);  // Should be clamped
  });
}

// Test Requirements 2.3: Encoder reading functionality with mock serial responses
class EncoderReadingUnitTest : public MecaBridgeSerialProtocolUnitTest
{
protected:
  void SetUp() override
  {
    MecaBridgeSerialProtocolUnitTest::SetUp();

    // Set up a connected state
    EXPECT_CALL(*mock_serial_, setPort(_));
    EXPECT_CALL(*mock_serial_, setBaudrate(_));
    EXPECT_CALL(*mock_serial_, setTimeout(_));
    EXPECT_CALL(*mock_serial_, open());
    EXPECT_CALL(*mock_serial_, isOpen())
    .WillRepeatedly(Return(true));
    EXPECT_CALL(*mock_serial_, write("PING\n"))
    .WillOnce(Return(5));

    comms_->setup("/dev/ttyUSB0", 115200, 50);
  }
};

TEST_F(EncoderReadingUnitTest, TestDifferentialEncoderReadingSuccess)
{
  // Test successful differential encoder reading
  EXPECT_CALL(*mock_serial_, write("E\n"))
  .WillOnce(Return(2));
  EXPECT_CALL(*mock_serial_, readline())
  .WillOnce(Return("1234 -5678\n"));

  int left_enc, right_enc;
  EXPECT_NO_THROW(
  {
    comms_->readDifferentialEncoders(left_enc, right_enc);
  });

  EXPECT_EQ(left_enc, 1234);
  EXPECT_EQ(right_enc, -5678);
}

TEST_F(EncoderReadingUnitTest, TestDifferentialEncoderReadingZeroValues)
{
  // Test zero encoder values
  EXPECT_CALL(*mock_serial_, write("E\n"))
  .WillOnce(Return(2));
  EXPECT_CALL(*mock_serial_, readline())
  .WillOnce(Return("0 0\n"));

  int left_enc, right_enc;
  EXPECT_NO_THROW(
  {
    comms_->readDifferentialEncoders(left_enc, right_enc);
  });

  EXPECT_EQ(left_enc, 0);
  EXPECT_EQ(right_enc, 0);
}

TEST_F(EncoderReadingUnitTest, TestDifferentialEncoderReadingInvalidFormat)
{
  // Test invalid response format
  EXPECT_CALL(*mock_serial_, write("E\n"))
  .WillOnce(Return(2));
  EXPECT_CALL(*mock_serial_, readline())
  .WillOnce(Return("invalid response\n"));

  int left_enc, right_enc;
  EXPECT_THROW(
  {
    comms_->readDifferentialEncoders(left_enc, right_enc);
  }, std::runtime_error);
}

TEST_F(EncoderReadingUnitTest, TestDifferentialEncoderReadingIncompleteResponse)
{
  // Test incomplete response (only one value)
  EXPECT_CALL(*mock_serial_, write("E\n"))
  .WillOnce(Return(2));
  EXPECT_CALL(*mock_serial_, readline())
  .WillOnce(Return("1234\n"));

  int left_enc, right_enc;
  EXPECT_THROW(
  {
    comms_->readDifferentialEncoders(left_enc, right_enc);
  }, std::runtime_error);
}

TEST_F(EncoderReadingUnitTest, TestFourEncoderReadingSuccess)
{
  // Test successful four encoder reading
  EXPECT_CALL(*mock_serial_, write("E\n"))
  .WillOnce(Return(2));
  EXPECT_CALL(*mock_serial_, readline())
  .WillOnce(Return("100 -200 300 -400\n"));

  int fl, fr, rl, rr;
  EXPECT_NO_THROW(
  {
    comms_->readFourEncoders(fl, fr, rl, rr);
  });

  EXPECT_EQ(fl, 100);
  EXPECT_EQ(fr, -200);
  EXPECT_EQ(rl, 300);
  EXPECT_EQ(rr, -400);
}

TEST_F(EncoderReadingUnitTest, TestFourEncoderReadingZeroValues)
{
  // Test zero encoder values
  EXPECT_CALL(*mock_serial_, write("E\n"))
  .WillOnce(Return(2));
  EXPECT_CALL(*mock_serial_, readline())
  .WillOnce(Return("0 0 0 0\n"));

  int fl, fr, rl, rr;
  EXPECT_NO_THROW(
  {
    comms_->readFourEncoders(fl, fr, rl, rr);
  });

  EXPECT_EQ(fl, 0);
  EXPECT_EQ(fr, 0);
  EXPECT_EQ(rl, 0);
  EXPECT_EQ(rr, 0);
}

TEST_F(EncoderReadingUnitTest, TestFourEncoderReadingInvalidFormat)
{
  // Test invalid response format - should set all values to 0 and not throw
  EXPECT_CALL(*mock_serial_, write("E\n"))
  .WillOnce(Return(2));
  EXPECT_CALL(*mock_serial_, readline())
  .WillOnce(Return("invalid response\n"));

  int fl, fr, rl, rr;
  EXPECT_NO_THROW(
  {
    comms_->readFourEncoders(fl, fr, rl, rr);
  });

  // Should be set to 0 on parse failure
  EXPECT_EQ(fl, 0);
  EXPECT_EQ(fr, 0);
  EXPECT_EQ(rl, 0);
  EXPECT_EQ(rr, 0);
}

TEST_F(EncoderReadingUnitTest, TestFourEncoderReadingIncompleteResponse)
{
  // Test incomplete response (only three values) - should set all to 0
  EXPECT_CALL(*mock_serial_, write("E\n"))
  .WillOnce(Return(2));
  EXPECT_CALL(*mock_serial_, readline())
  .WillOnce(Return("100 200 300\n"));

  int fl, fr, rl, rr;
  EXPECT_NO_THROW(
  {
    comms_->readFourEncoders(fl, fr, rl, rr);
  });

  // Should be set to 0 on incomplete response
  EXPECT_EQ(fl, 0);
  EXPECT_EQ(fr, 0);
  EXPECT_EQ(rl, 0);
  EXPECT_EQ(rr, 0);
}

// Test Requirements 2.4: Error handling scenarios and connection failures
class ErrorHandlingUnitTest : public MecaBridgeSerialProtocolUnitTest
{
};

TEST_F(ErrorHandlingUnitTest, TestMotorCommandsWhenDisconnected)
{
  // Test that motor commands throw when not connected
  EXPECT_CALL(*mock_serial_, isOpen())
  .WillRepeatedly(Return(false));

  EXPECT_THROW(
  {
    comms_->setDifferentialMotors(50, -30);
  }, std::runtime_error);

  EXPECT_THROW(
  {
    comms_->setFourMotors(10, 20, 30, 40);
  }, std::runtime_error);
}

TEST_F(ErrorHandlingUnitTest, TestEncoderReadingWhenDisconnected)
{
  // Test that encoder reading throws when not connected
  EXPECT_CALL(*mock_serial_, isOpen())
  .WillRepeatedly(Return(false));

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
}

TEST_F(ErrorHandlingUnitTest, TestSerialWriteFailure)
{
  // Set up connected state
  EXPECT_CALL(*mock_serial_, setPort(_));
  EXPECT_CALL(*mock_serial_, setBaudrate(_));
  EXPECT_CALL(*mock_serial_, setTimeout(_));
  EXPECT_CALL(*mock_serial_, open());
  EXPECT_CALL(*mock_serial_, isOpen())
  .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_serial_, write("PING\n"))
  .WillOnce(Return(5));

  comms_->setup("/dev/ttyUSB0", 115200, 50);

  // Test write failure
  EXPECT_CALL(*mock_serial_, write("V 50 -30\n"))
  .WillOnce(Throw(std::runtime_error("Write failed")));

  EXPECT_THROW(
  {
    comms_->setDifferentialMotors(50, -30);
  }, std::runtime_error);

  // Check that error counter was incremented
  int write_errors, read_errors, reconnection_attempts;
  comms_->getConnectionStats(write_errors, read_errors, reconnection_attempts);
  EXPECT_GT(write_errors, 0);
}

TEST_F(ErrorHandlingUnitTest, TestSerialReadFailure)
{
  // Set up connected state
  EXPECT_CALL(*mock_serial_, setPort(_));
  EXPECT_CALL(*mock_serial_, setBaudrate(_));
  EXPECT_CALL(*mock_serial_, setTimeout(_));
  EXPECT_CALL(*mock_serial_, open());
  EXPECT_CALL(*mock_serial_, isOpen())
  .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_serial_, write("PING\n"))
  .WillOnce(Return(5));

  comms_->setup("/dev/ttyUSB0", 115200, 50);

  // Test read failure
  EXPECT_CALL(*mock_serial_, write("E\n"))
  .WillOnce(Return(2));
  EXPECT_CALL(*mock_serial_, readline())
  .WillOnce(Throw(std::runtime_error("Read failed")));

  int left_enc, right_enc;
  EXPECT_THROW(
  {
    comms_->readDifferentialEncoders(left_enc, right_enc);
  }, std::runtime_error);

  // Check that error counter was incremented
  int write_errors, read_errors, reconnection_attempts;
  comms_->getConnectionStats(write_errors, read_errors, reconnection_attempts);
  EXPECT_GT(read_errors, 0);
}

TEST_F(ErrorHandlingUnitTest, TestReconnectionAttempts)
{
  // Test reconnection when never connected
  EXPECT_CALL(*mock_serial_, isOpen())
  .WillOnce(Return(false));    // Not connected initially

  // Reconnection attempt should fail
  EXPECT_FALSE(comms_->attemptReconnection());

  // Check that reconnection attempt was tracked
  int write_errors, read_errors, reconnection_attempts;
  comms_->getConnectionStats(write_errors, read_errors, reconnection_attempts);
  EXPECT_GT(reconnection_attempts, 0);
}

TEST_F(ErrorHandlingUnitTest, TestSuccessfulReconnection)
{
  // Set up initial connection
  EXPECT_CALL(*mock_serial_, setPort("/dev/ttyUSB0"));
  EXPECT_CALL(*mock_serial_, setBaudrate(115200));
  EXPECT_CALL(*mock_serial_, setTimeout(_));
  EXPECT_CALL(*mock_serial_, open());
  EXPECT_CALL(*mock_serial_, isOpen())
  .WillOnce(Return(true));
  EXPECT_CALL(*mock_serial_, write("PING\n"))
  .WillOnce(Return(5));

  comms_->setup("/dev/ttyUSB0", 115200, 50);

  // Simulate disconnection
  EXPECT_CALL(*mock_serial_, isOpen())
  .WillOnce(Return(false));    // Disconnected

  // Simulate successful reconnection
  EXPECT_CALL(*mock_serial_, close());
  EXPECT_CALL(*mock_serial_, setPort("/dev/ttyUSB0"));
  EXPECT_CALL(*mock_serial_, setBaudrate(115200));
  EXPECT_CALL(*mock_serial_, setTimeout(_));
  EXPECT_CALL(*mock_serial_, open());
  EXPECT_CALL(*mock_serial_, isOpen())
  .WillOnce(Return(true));
  EXPECT_CALL(*mock_serial_, write("PING\n"))
  .WillOnce(Return(5));

  EXPECT_TRUE(comms_->attemptReconnection());
}

TEST_F(ErrorHandlingUnitTest, TestErrorCounterReset)
{
  // Generate some errors first
  EXPECT_CALL(*mock_serial_, isOpen())
  .WillRepeatedly(Return(false));

  // Attempt operations that will fail
  try {
    comms_->setDifferentialMotors(50, -30);
  } catch (...) {
  }
  try {
    comms_->attemptReconnection();
  } catch (...) {
  }

  // Check that counters have values
  int write_errors, read_errors, reconnection_attempts;
  comms_->getConnectionStats(write_errors, read_errors, reconnection_attempts);
  EXPECT_GT(reconnection_attempts, 0);

  // Reset counters
  comms_->resetErrorCounters();

  // Check that counters are reset
  comms_->getConnectionStats(write_errors, read_errors, reconnection_attempts);
  EXPECT_EQ(write_errors, 0);
  EXPECT_EQ(read_errors, 0);
  EXPECT_EQ(reconnection_attempts, 0);
}

TEST_F(ErrorHandlingUnitTest, TestPingWhenDisconnected)
{
  // Test that ping doesn't throw when disconnected (just logs warning)
  EXPECT_CALL(*mock_serial_, isOpen())
  .WillOnce(Return(false));

  EXPECT_NO_THROW(
  {
    comms_->sendPing();
  });
}

TEST_F(ErrorHandlingUnitTest, TestPingWriteFailure)
{
  // Set up connected state
  EXPECT_CALL(*mock_serial_, setPort(_));
  EXPECT_CALL(*mock_serial_, setBaudrate(_));
  EXPECT_CALL(*mock_serial_, setTimeout(_));
  EXPECT_CALL(*mock_serial_, open());
  EXPECT_CALL(*mock_serial_, isOpen())
  .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_serial_, write("PING\n"))
  .WillOnce(Return(5));

  comms_->setup("/dev/ttyUSB0", 115200, 50);

  // Test ping write failure (should not throw, just log warning)
  EXPECT_CALL(*mock_serial_, write("PING\n"))
  .WillOnce(Throw(std::runtime_error("Write failed")));

  EXPECT_NO_THROW(
  {
    comms_->sendPing();
  });
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
