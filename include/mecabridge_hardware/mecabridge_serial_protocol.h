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

#ifndef MECABRIDGE_HARDWARE_MECABRIDGE_SERIAL_PROTOCOL_H
#define MECABRIDGE_HARDWARE_MECABRIDGE_SERIAL_PROTOCOL_H

#include <string>
#include <vector>
#include <serial/serial.h>

namespace mecabridge_hardware
{

  class MecaBridgeSerialProtocol
  {
public:
    MecaBridgeSerialProtocol();
    MecaBridgeSerialProtocol(
      const std::string & serial_device, int32_t baud_rate,
      int32_t timeout_ms);

    void setup(const std::string & serial_device, int32_t baud_rate, int32_t timeout_ms);
    void sendPing(); // Sends "PING\n" for Arduino reset sync

    // Motor command methods for different drive types
    void setDifferentialMotors(int left_val, int right_val); // "V {left} {right}\n"
    void setFourMotors(int fl, int fr, int rl, int rr);     // "M {fl} {fr} {rl} {rr}\n"

    // Encoder reading methods (optional)
    void readDifferentialEncoders(int & left_enc, int & right_enc); // "E\n" -> "left right"
    void readFourEncoders(int & fl, int & fr, int & rl, int & rr);  // "E\n" -> "fl fr rl rr"

    bool connected() const;

    // Connection recovery
    bool attemptReconnection();

    // Testing support - allows injection of mock serial for unit tests
  #ifdef TESTING_MODE
    void setMockSerial(std::shared_ptr < class MockSerial > mock_serial);
  #endif

    // Error statistics and diagnostics
    void getConnectionStats(
      int & write_errors, int & read_errors,
      int & reconnection_attempts) const;
    void resetErrorCounters();

private:
    serial::Serial serial_conn_;

  #ifdef TESTING_MODE
    std::shared_ptr < class MockSerial > mock_serial_;
    bool use_mock_serial_;
  #endif

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
  };

}  // namespace mecabridge_hardware

#endif  // MECABRIDGE_HARDWARE_MECABRIDGE_SERIAL_PROTOCOL_H
