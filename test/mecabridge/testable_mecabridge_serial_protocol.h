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

#ifndef MECABRIDGE__TESTABLE_MECABRIDGE_SERIAL_PROTOCOL_H_
#define MECABRIDGE__TESTABLE_MECABRIDGE_SERIAL_PROTOCOL_H_

#include "mock_serial.hpp"

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

} // namespace mecabridge_hardware

#endif // MECABRIDGE__TESTABLE_MECABRIDGE_SERIAL_PROTOCOL_H_
