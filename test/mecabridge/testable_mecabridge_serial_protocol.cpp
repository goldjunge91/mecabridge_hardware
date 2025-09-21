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

#include "testable_mecabridge_serial_protocol.h"

#include <thread>
#include <chrono>
#include <memory>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>
#include <stdexcept>

namespace mecabridge_hardware
{

TestableMecaBridgeSerialProtocol::TestableMecaBridgeSerialProtocol()
: last_baud_rate_(115200), last_timeout_ms_(50),
  write_error_count_(0), read_error_count_(0), reconnection_attempts_(0),
  is_connected_(false)
{
}

TestableMecaBridgeSerialProtocol::TestableMecaBridgeSerialProtocol(
  std::shared_ptr<MockSerial> mock_serial)
: mock_serial_(mock_serial), last_baud_rate_(115200), last_timeout_ms_(50),
  write_error_count_(0), read_error_count_(0), reconnection_attempts_(0),
  is_connected_(false)
{
}

void TestableMecaBridgeSerialProtocol::setMockSerial(std::shared_ptr<MockSerial> mock_serial)
{
  mock_serial_ = mock_serial;
}

void TestableMecaBridgeSerialProtocol::setup(
  const std::string & serial_device, int32_t baud_rate,
  int32_t timeout_ms)
{
  if (!mock_serial_) {
    throw std::runtime_error("Mock serial not set for testing");
  }

  std::string port_to_use = serial_device;

  // If no specific port provided, auto-detect
  if (port_to_use.empty()) {
    RCLCPP_INFO(
      rclcpp::get_logger(
        "TestableMecaBridgeSerialProtocol"), "No device specified, attempting auto-detection...");
    try {
      port_to_use = findSerialPort("");
      RCLCPP_INFO(
        rclcpp::get_logger(
          "TestableMecaBridgeSerialProtocol"), "Auto-detected serial port: %s",
        port_to_use.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        rclcpp::get_logger("TestableMecaBridgeSerialProtocol"), "Auto-detection failed: %s",
        e.what());
      throw std::runtime_error("Serial port auto-detection failed. Please specify device parameter.");
    }
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger(
        "TestableMecaBridgeSerialProtocol"), "Using specified serial port: %s",
      port_to_use.c_str());
  }

  // Validate parameters
  if (baud_rate <= 0) {
    RCLCPP_WARN(
      rclcpp::get_logger(
        "TestableMecaBridgeSerialProtocol"), "Invalid baud rate %d, using 115200", baud_rate);
    baud_rate = 115200;
  }

  if (timeout_ms <= 0) {
    RCLCPP_WARN(
      rclcpp::get_logger(
        "TestableMecaBridgeSerialProtocol"), "Invalid timeout %d ms, using 50 ms", timeout_ms);
    timeout_ms = 50;
  }

  try {
    RCLCPP_INFO(
      rclcpp::get_logger(
        "TestableMecaBridgeSerialProtocol"), "Configuring serial connection...");
    mock_serial_->setPort(port_to_use);
    mock_serial_->setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    mock_serial_->setTimeout(tt);

    RCLCPP_INFO(
      rclcpp::get_logger(
        "TestableMecaBridgeSerialProtocol"), "Opening serial connection...");
    mock_serial_->open();

    if (!mock_serial_->isOpen()) {
      throw std::runtime_error("Serial port opened but reports as not open");
    }

    is_connected_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger(
        "TestableMecaBridgeSerialProtocol"), "Serial connection established successfully");

    // Wait for Arduino reset (matching Python behavior)
    RCLCPP_INFO(
      rclcpp::get_logger(
        "TestableMecaBridgeSerialProtocol"), "Waiting for Arduino reset (2 seconds)...");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Send initial PING for synchronization
    RCLCPP_INFO(
      rclcpp::get_logger("TestableMecaBridgeSerialProtocol"),
      "Sending initial synchronization ping...");
    sendPing();

    // Store connection parameters for potential reconnection
    last_device_ = port_to_use;
    last_baud_rate_ = baud_rate;
    last_timeout_ms_ = timeout_ms;

    RCLCPP_INFO(
      rclcpp::get_logger(
        "TestableMecaBridgeSerialProtocol"), "mecabridge communication setup complete");
    RCLCPP_INFO(
      rclcpp::get_logger("TestableMecaBridgeSerialProtocol"), "Port: %s, Baud: %d, Timeout: %d ms",
      port_to_use.c_str(), baud_rate, timeout_ms);
  } catch (const std::exception & e) {
    is_connected_ = false;
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "TestableMecaBridgeSerialProtocol"), "Failed to setup serial communication on port %s: %s",
      port_to_use.c_str(), e.what());
    throw;
  }
}

void TestableMecaBridgeSerialProtocol::sendPing()
{
  try {
    sendMsg("PING\n");
    RCLCPP_DEBUG(rclcpp::get_logger("TestableMecaBridgeSerialProtocol"), "PING sent successfully");
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      rclcpp::get_logger(
        "TestableMecaBridgeSerialProtocol"), "Failed to send PING: %s", e.what());
    RCLCPP_WARN(
      rclcpp::get_logger(
        "TestableMecaBridgeSerialProtocol"), "Arduino synchronization may be incomplete");
  }
}

void TestableMecaBridgeSerialProtocol::setDifferentialMotors(int left_val, int right_val)
{
  // Clamp values to valid range [-100, 100] with logging if clamping occurs
  int orig_left = left_val, orig_right = right_val;
  left_val = std::max(-100, std::min(100, left_val));
  right_val = std::max(-100, std::min(100, right_val));

  if (orig_left != left_val || orig_right != right_val) {
    RCLCPP_WARN(
      rclcpp::get_logger("TestableMecaBridgeSerialProtocol"),
      "Motor values clamped: L(%d->%d), R(%d->%d)",
      orig_left, left_val, orig_right, right_val);
  }

  std::stringstream ss;
  ss << "V " << left_val << " " << right_val << "\n";

  try {
    sendMsg(ss.str());
    RCLCPP_DEBUG(
      rclcpp::get_logger(
        "TestableMecaBridgeSerialProtocol"), "Differential motors: L=%d, R=%d", left_val,
      right_val);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "TestableMecaBridgeSerialProtocol"), "Failed to send differential motor command (L=%d, R=%d): %s",
      left_val, right_val, e.what());
    throw;
  }
}

void TestableMecaBridgeSerialProtocol::setFourMotors(int fl, int fr, int rl, int rr)
{
  // Clamp values to valid range [-100, 100] with logging if clamping occurs
  int orig_fl = fl, orig_fr = fr, orig_rl = rl, orig_rr = rr;
  fl = std::max(-100, std::min(100, fl));
  fr = std::max(-100, std::min(100, fr));
  rl = std::max(-100, std::min(100, rl));
  rr = std::max(-100, std::min(100, rr));

  if (orig_fl != fl || orig_fr != fr || orig_rl != rl || orig_rr != rr) {
    RCLCPP_WARN(
      rclcpp::get_logger("TestableMecaBridgeSerialProtocol"),
      "Motor values clamped: FL(%d->%d), FR(%d->%d), RL(%d->%d), RR(%d->%d)",
      orig_fl, fl, orig_fr, fr, orig_rl, rl, orig_rr, rr);
  }

  std::stringstream ss;
  ss << "M " << fl << " " << fr << " " << rl << " " << rr << "\n";

  try {
    sendMsg(ss.str());
    RCLCPP_DEBUG(
      rclcpp::get_logger(
        "TestableMecaBridgeSerialProtocol"), "Four motors: FL=%d, FR=%d, RL=%d, RR=%d", fl, fr, rl,
      rr);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "TestableMecaBridgeSerialProtocol"), "Failed to send four motor command (FL=%d, FR=%d, RL=%d, RR=%d): %s",
      fl, fr, rl, rr, e.what());
    throw;
  }
}

void TestableMecaBridgeSerialProtocol::readDifferentialEncoders(int & left_enc, int & right_enc)
{
  try {
    std::string response = sendMsgWithResponse("E\n");

    // Parse response: "left_count right_count"
    std::istringstream iss(response);
    std::string left_str, right_str;

    if (iss >> left_str >> right_str) {
      try {
        left_enc = std::stoi(left_str);
        right_enc = std::stoi(right_str);
        RCLCPP_DEBUG(
          rclcpp::get_logger(
            "TestableMecaBridgeSerialProtocol"), "Encoder values: L=%d, R=%d", left_enc, right_enc);
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          rclcpp::get_logger(
            "TestableMecaBridgeSerialProtocol"), "Failed to convert encoder values ('%s', '%s'): %s",
          left_str.c_str(), right_str.c_str(), e.what());
        left_enc = 0;
        right_enc = 0;
      }
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger(
          "TestableMecaBridgeSerialProtocol"), "Invalid encoder response format: '%s' (expected: 'left right')",
        response.c_str());
      left_enc = 0;
      right_enc = 0;
    }
  } catch (const std::exception & e) {
    static int encoder_error_count = 0;
    encoder_error_count++;

    if (encoder_error_count % 20 == 1) {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "TestableMecaBridgeSerialProtocol"), "Failed to read differential encoders (error count: %d): %s",
        encoder_error_count, e.what());
    }
    left_enc = 0;
    right_enc = 0;
  }
}

void TestableMecaBridgeSerialProtocol::readFourEncoders(int & fl, int & fr, int & rl, int & rr)
{
  try {
    std::string response = sendMsgWithResponse("E\n");

    // Parse response: "fl_count fr_count rl_count rr_count"
    std::istringstream iss(response);
    std::string fl_str, fr_str, rl_str, rr_str;

    if (iss >> fl_str >> fr_str >> rl_str >> rr_str) {
      try {
        fl = std::stoi(fl_str);
        fr = std::stoi(fr_str);
        rl = std::stoi(rl_str);
        rr = std::stoi(rr_str);
        RCLCPP_DEBUG(
          rclcpp::get_logger(
            "TestableMecaBridgeSerialProtocol"), "Four encoder values: FL=%d, FR=%d, RL=%d, RR=%d", fl, fr, rl,
          rr);
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          rclcpp::get_logger(
            "TestableMecaBridgeSerialProtocol"), "Failed to convert four encoder values ('%s', '%s', '%s', '%s'): %s",
          fl_str.c_str(), fr_str.c_str(), rl_str.c_str(), rr_str.c_str(), e.what());
        fl = fr = rl = rr = 0;
      }
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger(
          "TestableMecaBridgeSerialProtocol"), "Invalid four encoder response format: '%s' (expected: 'fl fr rl rr')",
        response.c_str());
      fl = fr = rl = rr = 0;
    }
  } catch (const std::exception & e) {
    static int encoder_error_count = 0;
    encoder_error_count++;

    if (encoder_error_count % 20 == 1) {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "TestableMecaBridgeSerialProtocol"), "Failed to read four encoders (error count: %d): %s",
        encoder_error_count, e.what());
    }
    fl = fr = rl = rr = 0;
  }
}

bool TestableMecaBridgeSerialProtocol::connected() const
{
  return mock_serial_ && mock_serial_->isOpen() && is_connected_;
}

std::string TestableMecaBridgeSerialProtocol::findSerialPort(const std::string & preferred_port)
{
  // If preferred port is specified and exists, use it
  if (!preferred_port.empty()) {
    RCLCPP_INFO(
      rclcpp::get_logger(
        "TestableMecaBridgeSerialProtocol"), "Testing preferred port: %s", preferred_port.c_str());
    try {
      // In test mode, we simulate port testing by trying to create a mock connection
      mock_serial_->setPort(preferred_port);
      mock_serial_->setBaudrate(115200);
      serial::Timeout tt = serial::Timeout::simpleTimeout(50);
      mock_serial_->setTimeout(tt);
      mock_serial_->open();
      if (mock_serial_->isOpen()) {
        mock_serial_->close();
        RCLCPP_INFO(
          rclcpp::get_logger(
            "TestableMecaBridgeSerialProtocol"), "Preferred port %s is available",
          preferred_port.c_str());
        return preferred_port;
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        rclcpp::get_logger(
          "TestableMecaBridgeSerialProtocol"), "Preferred port %s failed: %s",
        preferred_port.c_str(), e.what());
      RCLCPP_INFO(
        rclcpp::get_logger(
          "TestableMecaBridgeSerialProtocol"), "Falling back to auto-detection...");
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger(
      "TestableMecaBridgeSerialProtocol"), "Scanning for available serial ports...");

  // Try each candidate port
  std::vector<std::string> failed_ports;
  for (const auto & port : SERIAL_PORT_CANDIDATES) {
    try {
      RCLCPP_INFO(
        rclcpp::get_logger(
          "TestableMecaBridgeSerialProtocol"), "Testing port: %s", port.c_str());
      mock_serial_->setPort(port);
      mock_serial_->setBaudrate(115200);
      serial::Timeout tt = serial::Timeout::simpleTimeout(50);
      mock_serial_->setTimeout(tt);
      mock_serial_->open();
      if (mock_serial_->isOpen()) {
        mock_serial_->close();
        RCLCPP_INFO(
          rclcpp::get_logger(
            "TestableMecaBridgeSerialProtocol"), "Successfully found working port: %s",
          port.c_str());
        return port;
      }
    } catch (const std::exception & e) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("TestableMecaBridgeSerialProtocol"), "Port %s failed: %s",
        port.c_str(), e.what());
      failed_ports.push_back(port + " (" + e.what() + ")");
      continue;
    }
  }

  // Provide detailed error information
  RCLCPP_ERROR(
    rclcpp::get_logger(
      "TestableMecaBridgeSerialProtocol"),
    "No working serial port found after testing all candidates");
  throw std::runtime_error("No serial port found. Check device connection and permissions.");
}

void TestableMecaBridgeSerialProtocol::sendMsg(const std::string & msg)
{
  if (!connected()) {
    throw std::runtime_error("Serial connection not open - cannot send message");
  }

  try {
    size_t bytes_written = mock_serial_->write(msg);
    if (bytes_written != msg.length()) {
      RCLCPP_WARN(
        rclcpp::get_logger("TestableMecaBridgeSerialProtocol"),
        "Incomplete write: sent %zu bytes, expected %zu bytes",
        bytes_written, msg.length());
    }
    RCLCPP_DEBUG(
      rclcpp::get_logger(
        "TestableMecaBridgeSerialProtocol"), "Sent message: %s", msg.c_str());
  } catch (const std::exception & e) {
    write_error_count_++;
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "TestableMecaBridgeSerialProtocol"), "Serial write failed for message '%s' (error count: %d): %s",
      msg.c_str(), write_error_count_, e.what());
    throw std::runtime_error("Failed to write to serial port: " + std::string(e.what()));
  }
}

std::string TestableMecaBridgeSerialProtocol::sendMsgWithResponse(const std::string & msg)
{
  if (!connected()) {
    throw std::runtime_error("Serial connection not open - cannot send message with response");
  }

  try {
    // Send the message
    size_t bytes_written = mock_serial_->write(msg);
    if (bytes_written != msg.length()) {
      RCLCPP_WARN(
        rclcpp::get_logger("TestableMecaBridgeSerialProtocol"),
        "Incomplete write for message with response: sent %zu bytes, expected %zu bytes",
        bytes_written, msg.length());
    }

    // Read the response
    std::string response = mock_serial_->readline();

    if (response.empty()) {
      RCLCPP_WARN(
        rclcpp::get_logger(
          "TestableMecaBridgeSerialProtocol"), "Empty response received for message: %s",
        msg.c_str());
    }

    // Remove trailing newline/carriage return
    while (!response.empty() && (response.back() == '\n' || response.back() == '\r')) {
      response.pop_back();
    }

    RCLCPP_DEBUG(
      rclcpp::get_logger("TestableMecaBridgeSerialProtocol"), "Message: %s -> Response: %s",
      msg.c_str(), response.c_str());
    return response;
  } catch (const std::exception & e) {
    read_error_count_++;
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "TestableMecaBridgeSerialProtocol"), "Failed to send message '%s' with response (error count: %d): %s",
      msg.c_str(), read_error_count_, e.what());
    throw std::runtime_error("Failed to send message with response: " + std::string(e.what()));
  }
}

bool TestableMecaBridgeSerialProtocol::attemptReconnection()
{
  if (connected()) {
    return true;  // Already connected
  }

  reconnection_attempts_++;
  RCLCPP_WARN(
    rclcpp::get_logger(
      "TestableMecaBridgeSerialProtocol"), "Attempting to reconnect to mecabridge (attempt #%d)...",
    reconnection_attempts_);

  try {
    // Close any existing connection
    if (mock_serial_->isOpen()) {
      mock_serial_->close();
    }

    // Try to reconnect using last known parameters
    setup(last_device_, last_baud_rate_, last_timeout_ms_);

    if (connected()) {
      RCLCPP_INFO(
        rclcpp::get_logger(
          "TestableMecaBridgeSerialProtocol"), "Successfully reconnected to mecabridge");
      return true;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "TestableMecaBridgeSerialProtocol"), "Reconnection attempt failed: %s", e.what());
  }

  RCLCPP_ERROR(
    rclcpp::get_logger(
      "TestableMecaBridgeSerialProtocol"), "Failed to reconnect to mecabridge");
  return false;
}

void TestableMecaBridgeSerialProtocol::getConnectionStats(
  int & write_errors, int & read_errors,
  int & reconnection_attempts) const
{
  write_errors = write_error_count_;
  read_errors = read_error_count_;
  reconnection_attempts = reconnection_attempts_;
}

void TestableMecaBridgeSerialProtocol::resetErrorCounters()
{
  write_error_count_ = 0;
  read_error_count_ = 0;
  reconnection_attempts_ = 0;
  RCLCPP_INFO(rclcpp::get_logger("TestableMecaBridgeSerialProtocol"), "Error counters reset");
}

}  // namespace mecabridge_hardware
