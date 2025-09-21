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

// NOTE: Removed UTF-8 BOM that previously broke first preprocessor directive
#include "mecabridge_hardware/mecabridge_hardware.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstring>
#include <chrono>

namespace mecabridge_hardware
{

hardware_interface::CallbackReturn MecaBridgeHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get config file path from <param>
  auto it = info.hardware_parameters.find("config_file");
  if (it == info.hardware_parameters.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("MecaBridgeHardware"), "config_file parameter not found");
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::string config_file = it->second;

  try {
    cfg_ = mecabridge::config::parse_from_yaml_file(config_file);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("MecaBridgeHardware"), "Failed to parse YAML config: %s",
      e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize vectors based on joints from config and compile-time features
  size_t num_joints = 8;  // 4 wheels * 2 (velocity + position)

#if MECABRIDGE_ENABLE_SERVOS
  if (cfg_.features.enable_servos) {
    num_joints += 2;  // positional + continuous servo
  }
#endif

#if MECABRIDGE_ENABLE_ESCS
  if (cfg_.features.enable_escs) {
    num_joints += 2;  // 2 ESC channels
  }
#endif

  hw_states_.assign(num_joints, 0.0);
  hw_commands_.assign(num_joints, 0.0);

  configured_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MecaBridgeHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  si.reserve(hw_states_.size() * 2);  // Each wheel has velocity + position

  auto wheel_names = cfg_.wheel_joint_names();
  for (size_t i = 0; i < 4; ++i) {
    si.emplace_back(
      hardware_interface::StateInterface(
        wheel_names[i],
        hardware_interface::HW_IF_VELOCITY, &hw_states_[i]));
    si.emplace_back(
      hardware_interface::StateInterface(
        wheel_names[i],
        hardware_interface::HW_IF_POSITION, &hw_states_[i + 4]));  // Use separate positions
  }

  size_t offset = 8;  // After 4 wheels * 2 states each

#if MECABRIDGE_ENABLE_SERVOS
  if (cfg_.features.enable_servos) {
    si.emplace_back(
      hardware_interface::StateInterface(
        cfg_.servos.positional.joint_name,
        hardware_interface::HW_IF_POSITION, &hw_states_[offset++]));
    si.emplace_back(
      hardware_interface::StateInterface(
        cfg_.servos.continuous.joint_name,
        hardware_interface::HW_IF_VELOCITY, &hw_states_[offset++]));
  }
#endif

#if MECABRIDGE_ENABLE_ESCS
  if (cfg_.features.enable_escs) {
    si.emplace_back(
      hardware_interface::StateInterface(
        cfg_.escs.left.joint_name,
        hardware_interface::HW_IF_VELOCITY, &hw_states_[offset++]));
    si.emplace_back(
      hardware_interface::StateInterface(
        cfg_.escs.right.joint_name,
        hardware_interface::HW_IF_VELOCITY, &hw_states_[offset++]));
  }
#endif

  return si;
}

std::vector<hardware_interface::CommandInterface> MecaBridgeHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  ci.reserve(hw_commands_.size());

  auto wheel_names = cfg_.wheel_joint_names();
  for (size_t i = 0; i < 4; ++i) {
    ci.emplace_back(
      hardware_interface::CommandInterface(
        wheel_names[i],
        hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  size_t offset = 4;

#if MECABRIDGE_ENABLE_SERVOS
  if (cfg_.features.enable_servos) {
    ci.emplace_back(
      hardware_interface::CommandInterface(
        cfg_.servos.positional.joint_name,
        hardware_interface::HW_IF_POSITION, &hw_commands_[offset++]));
    ci.emplace_back(
      hardware_interface::CommandInterface(
        cfg_.servos.continuous.joint_name,
        hardware_interface::HW_IF_VELOCITY, &hw_commands_[offset++]));
  }
#endif

#if MECABRIDGE_ENABLE_ESCS
  if (cfg_.features.enable_escs) {
    ci.emplace_back(
      hardware_interface::CommandInterface(
        cfg_.escs.left.joint_name,
        hardware_interface::HW_IF_VELOCITY, &hw_commands_[offset++]));
    ci.emplace_back(
      hardware_interface::CommandInterface(
        cfg_.escs.right.joint_name,
        hardware_interface::HW_IF_VELOCITY, &hw_commands_[offset++]));
  }
#endif

  return ci;
}

hardware_interface::CallbackReturn MecaBridgeHardware::on_configure(const rclcpp_lifecycle::State &)
{
  if (!configured_) {return hardware_interface::CallbackReturn::ERROR;}

  // Initialize serial backend
  serial_backend_ = std::make_unique<mecabridge::serial::MockSerialBackend>();

  mecabridge::serial::SerialOptions opts;
  opts.device = cfg_.serial_port;
  opts.baud_rate = 115200; // Default baud rate for now
  opts.read_timeout_ms = 20;
  opts.write_timeout_ms = 20;

  if (!serial_backend_->open(opts)) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "MecaBridgeHardware"), "Failed to open serial port: %s", opts.device.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize watchdog
  watchdog_.reset();

  RCLCPP_INFO(
    rclcpp::get_logger(
      "MecaBridgeHardware"), "MecaBridge hardware configured successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecaBridgeHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  active_ = false;

  // Close serial connection
  if (serial_backend_ && serial_backend_->is_open()) {
    serial_backend_->close();
  }
  serial_backend_.reset();

  RCLCPP_INFO(rclcpp::get_logger("MecaBridgeHardware"), "MecaBridge hardware cleaned up");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecaBridgeHardware::on_activate(const rclcpp_lifecycle::State &)
{
  if (!configured_) {return hardware_interface::CallbackReturn::ERROR;}
  active_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecaBridgeHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MecaBridgeHardware::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!active_) {return hardware_interface::return_type::ERROR;}

  auto now = std::chrono::steady_clock::now();

  // Check watchdog status first
  if (watchdog_.tripped(now)) {
    RCLCPP_WARN(
      rclcpp::get_logger(
        "MecaBridgeHardware"), "Watchdog timeout - zeroing all state values");
    // Zero all state values on watchdog timeout
    std::fill(hw_states_.begin(), hw_states_.end(), 0.0);
    return hardware_interface::return_type::OK;
  }

  // Read state frame from hardware
  return readStateFrame();
}

hardware_interface::return_type MecaBridgeHardware::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!active_) {return hardware_interface::return_type::ERROR;}

  auto now = std::chrono::steady_clock::now();

  // Check watchdog status - send zero commands if tripped
  if (watchdog_.tripped(now)) {
    RCLCPP_WARN(
      rclcpp::get_logger("MecaBridgeHardware"),
      "Watchdog timeout - sending zero commands");
    // Create zero command frame
    mecabridge::protocol::CommandFramePayload zero_command = {};
    zero_command.seq = ++command_sequence_;

    size_t bytes_written = 0;
    auto result = mecabridge::protocol::encodeCommand(
      zero_command, tx_buffer_, sizeof(tx_buffer_),
      bytes_written);
    if (result != mecabridge::protocol::ErrorCode::OK) {
      RCLCPP_ERROR(rclcpp::get_logger("MecaBridgeHardware"), "Failed to encode zero command frame");
      return hardware_interface::return_type::ERROR;
    }

    // Send the frame
    if (serial_backend_ && serial_backend_->is_open()) {
      serial_backend_->write(tx_buffer_, bytes_written);
    }

    return hardware_interface::return_type::OK;
  }

  // Send normal command frame
  return writeCommandFrame();
}

hardware_interface::return_type MecaBridgeHardware::readStateFrame()
{
  if (!serial_backend_ || !serial_backend_->is_open()) {
    return hardware_interface::return_type::ERROR;
  }

  // Read available data from serial
  int bytes_read = serial_backend_->read(rx_buffer_, sizeof(rx_buffer_));
  if (bytes_read <= 0) {
    // No data available or error
    return hardware_interface::return_type::OK;
  }

  // Try to parse the frame
  mecabridge::protocol::ByteSpan input(rx_buffer_, bytes_read);
  mecabridge::protocol::ParsedFrame parsed_frame;

  auto parse_result = mecabridge::protocol::tryParseFrame(input, parsed_frame);
  if (parse_result != mecabridge::protocol::ParseResult::SUCCESS) {
    RCLCPP_WARN(rclcpp::get_logger("MecaBridgeHardware"), "Failed to parse state frame");
    return hardware_interface::return_type::ERROR;
  }

  // Check if it's a state frame
  if (parsed_frame.frame_id != mecabridge::protocol::FrameId::STATE) {
    RCLCPP_WARN(rclcpp::get_logger("MecaBridgeHardware"), "Received non-state frame");
    return hardware_interface::return_type::OK;
  }

  // Parse state payload
  if (parsed_frame.payload_len != sizeof(mecabridge::protocol::StateFramePayload)) {
    RCLCPP_ERROR(rclcpp::get_logger("MecaBridgeHardware"), "Invalid state frame payload size");
    return hardware_interface::return_type::ERROR;
  }

  const auto * state_payload =
    reinterpret_cast<const mecabridge::protocol::StateFramePayload *>(parsed_frame.payload_data);

  // Update watchdog on valid frame
  auto now = std::chrono::steady_clock::now();
  watchdog_.updateOnValidFrame(now);

  // Record latency measurement for echoed sequence
  latency_tracker_.recordStateEcho(state_payload->seq_echo, now);

  // Update hardware states from parsed frame
  updateStateFromFrame(*state_payload);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MecaBridgeHardware::writeCommandFrame()
{
  if (!serial_backend_ || !serial_backend_->is_open()) {
    return hardware_interface::return_type::ERROR;
  }

  // Build command frame from current command values
  mecabridge::protocol::CommandFramePayload command_payload = {};
  buildCommandFromState(command_payload);
  command_payload.seq = ++command_sequence_;

  // Record command send time for latency measurement
  auto send_time = std::chrono::steady_clock::now();
  latency_tracker_.recordCommandSent(command_payload.seq, send_time);

  // Encode the command frame
  size_t bytes_written = 0;
  auto result = mecabridge::protocol::encodeCommand(
    command_payload, tx_buffer_, sizeof(tx_buffer_),
    bytes_written);
  if (result != mecabridge::protocol::ErrorCode::OK) {
    RCLCPP_ERROR(rclcpp::get_logger("MecaBridgeHardware"), "Failed to encode command frame");
    return hardware_interface::return_type::ERROR;
  }

  // Send the frame
  int sent_bytes = serial_backend_->write(tx_buffer_, bytes_written);
  if (sent_bytes != static_cast<int>(bytes_written)) {
    RCLCPP_ERROR(rclcpp::get_logger("MecaBridgeHardware"), "Failed to send complete command frame");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

void MecaBridgeHardware::updateStateFromFrame(
  const mecabridge::protocol::StateFramePayload & state_payload)
{
  // Update wheel velocities from encoder deltas
  if (state_payload.dt_ms > 0) {
    double dt_sec = state_payload.dt_ms / 1000.0;
    auto wheel_names = cfg_.wheel_joint_names();

    for (size_t i = 0; i < 4 && i < hw_states_.size(); ++i) {
      // Convert encoder counts to velocity (simplified - needs proper encoder scaling)
      // This would need encoder ticks per revolution and wheel radius from config
      double encoder_delta = state_payload.encoder_counts[i]; // Simplified
      double wheel_velocity = encoder_delta / dt_sec; // Simplified conversion
      hw_states_[i] = wheel_velocity;
    }
  }

  // Update servo and ESC states if enabled
  size_t offset = 4;

#if MECABRIDGE_ENABLE_SERVOS
  if (cfg_.features.enable_servos) {
    if (offset < hw_states_.size()) {
      hw_states_[offset] = state_payload.servo_pos_rad; // Positional servo position
      offset++;
    }
    if (offset < hw_states_.size()) {
      hw_states_[offset] = state_payload.servo_cont_vel_norm; // Continuous servo velocity
      offset++;
    }
  }
#endif

#if MECABRIDGE_ENABLE_ESCS
  if (cfg_.features.enable_escs) {
    for (size_t i = 0; i < 2 && offset + i < hw_states_.size(); ++i) {
      hw_states_[offset + i] = state_payload.esc_norm[i];
    }
  }
#endif
}

void MecaBridgeHardware::buildCommandFromState(
  mecabridge::protocol::CommandFramePayload & command_payload)
{
  // Clear the payload
  std::memset(&command_payload, 0, sizeof(command_payload));

  // Set wheel velocities
  for (size_t i = 0; i < 4 && i < hw_commands_.size(); ++i) {
    command_payload.wheel_vel_rad_s[i] = static_cast<float>(hw_commands_[i]);
  }

  // Set servo and ESC commands if enabled
  size_t offset = 4;

#if MECABRIDGE_ENABLE_SERVOS
  if (cfg_.features.enable_servos) {
    if (offset < hw_commands_.size()) {
      command_payload.servo_pos_rad = static_cast<float>(hw_commands_[offset]);
      offset++;
    }
    if (offset < hw_commands_.size()) {
      command_payload.servo_cont_vel_norm = static_cast<float>(hw_commands_[offset]);
      offset++;
    }
  }
#endif

#if MECABRIDGE_ENABLE_ESCS
  if (cfg_.features.enable_escs) {
    for (size_t i = 0; i < 2 && offset + i < hw_commands_.size(); ++i) {
      command_payload.esc_norm[i] = static_cast<float>(hw_commands_[offset + i]);
    }
  }
#endif
}

void MecaBridgeHardware::logLatencyStats()
{
  if (latency_tracker_.getSampleCount() == 0) {
    return;
  }

  auto p95 = latency_tracker_.getP95Latency();
  auto mean = latency_tracker_.getMeanLatency();
  auto max = latency_tracker_.getMaxLatency();
  auto count = latency_tracker_.getSampleCount();

  auto p95_ms = std::chrono::duration_cast<std::chrono::milliseconds>(p95).count();
  auto mean_ms = std::chrono::duration_cast<std::chrono::milliseconds>(mean).count();
  auto max_ms = std::chrono::duration_cast<std::chrono::milliseconds>(max).count();

  RCLCPP_INFO(
    rclcpp::get_logger("MecaBridgeHardware"),
    "Latency stats: P95=%ldms, Mean=%ldms, Max=%ldms, Samples=%zu",
    p95_ms, mean_ms, max_ms, count);
}

} // namespace mecabridge_hardware

PLUGINLIB_EXPORT_CLASS(mecabridge_hardware::MecaBridgeHardware, hardware_interface::SystemInterface)
