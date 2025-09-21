#ifndef DRIVE_ARDUINO__TESTABLE_MECABRIDGE_SERIAL_PROTOCOL_H_
#define DRIVE_ARDUINO__TESTABLE_MECABRIDGE_SERIAL_PROTOCOL_H_


#include "mock_serial.h"

#include <string>
#include <vector>
#include <memory>

namespace mecabridge_hardware
{

// Testable version of MecaBridgeSerialProtocol that uses dependency injection
  class TestableMecaBridgeSerialProtocol
  {
public:
    TestableMecaBridgeSerialProtocol();
    TestableMecaBridgeSerialProtocol(std::shared_ptr < MockSerial > mock_serial);

    void setMockSerial(std::shared_ptr < MockSerial > mock_serial);
    void setup(const std::string & serial_device, int32_t baud_rate, int32_t timeout_ms);
    void sendPing();

    // Motor command methods for different drive types
    void setDifferentialMotors(int left_val, int right_val);
    void setFourMotors(int fl, int fr, int rl, int rr);

    // Encoder reading methods (optional)
    void readDifferentialEncoders(int & left_enc, int & right_enc);
    void readFourEncoders(int & fl, int & fr, int & rl, int & rr);

    bool connected() const;

    // Connection recovery
    bool attemptReconnection();

    // Error statistics and diagnostics
    void getConnectionStats(
      int & write_errors, int & read_errors,
      int & reconnection_attempts) const;
    void resetErrorCounters();

private:
    std::shared_ptr < MockSerial > mock_serial_;

    std::string findSerialPort(const std::string & preferred_port);
    void sendMsg(const std::string & msg);
    std::string sendMsgWithResponse(const std::string & msg);

    // Connection parameters for reconnection
    std::string last_device_;
    int32_t last_baud_rate_;
    int32_t last_timeout_ms_;

    // Serial port candidates for auto-detection
    const std::vector < std::string > SERIAL_PORT_CANDIDATES = {
      "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"
    };

    // Error tracking
    mutable int write_error_count_;
    mutable int read_error_count_;
    mutable int reconnection_attempts_;

    // Connection state
    bool is_connected_;
  };

}  // namespace mecabridge_hardware

#endif  // DRIVE_ARDUINO__TESTABLE_MECABRIDGE_SERIAL_PROTOCOL_H_
