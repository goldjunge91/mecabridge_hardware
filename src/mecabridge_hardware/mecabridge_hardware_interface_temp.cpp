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

#include "mecabridge_hardware/mecabridge_hardware_interface.h"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <cmath>

namespace mecabridge_hardware
{

MecaBridgeHardwareInterface::MecaBridgeHardwareInterface()
: logger_(rclcpp::get_logger("MecaBridgeHardwareInterface")),
  connection_error_count_(0),
  read_error_count_(0),
  write_error_count_(0)
{
}

hardware_interface::CallbackReturn MecaBridgeHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(logger_, "Failed to initialize SystemInterface base class");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring MecaBridge Hardware Interface...");

  time_ = std::chrono::system_clock::now();

  // Parse and validate drive type parameter
  std::string drive_type_str = "differential";  // default
  if (info_.hardware_parameters.find("drive_type") != info_.hardware_parameters.end()) {
    drive_type_str = info_.hardware_parameters["drive_type"];
  }

  // Convert string to enum with validation
  if (drive_type_str == "differential") {
    cfg_.drive_type = DriveType::DIFFERENTIAL;
    RCLCPP_INFO(logger_, "Drive type set to: differential");
  } else if (drive_type_str == "four_wheel") {
    cfg_.drive_type = DriveType::FOUR_WHEEL;
    RCLCPP_INFO(logger_, "Drive type set to: four_wheel");
  } else if (drive_type_str == "mecanum") {
    cfg_.drive_type = DriveType::MECANUM;
    RCLCPP_INFO(logger_, "Drive type set to: mecanum");
  } else {
    RCLCPP_WARN(
      logger_,
      "Invalid drive type '%s', using default 'differential'. Valid options: differential, four_wheel, mecanum",
      drive_type_str.c_str());
    cfg_.drive_type = DriveType::DIFFERENTIAL;
    drive_type_str = "differential";
  }

  // Parse and validate encoder support parameter
  if (info_.hardware_parameters.find("has_encoders") != info_.hardware_parameters.end()) {
    std::string encoder_str = info_.hardware_parameters["has_encoders"];
    cfg_.has_encoders = (encoder_str == "true" || encoder_str == "1");
    RCLCPP_INFO(logger_, "Encoder support: %s", cfg_.has_encoders ? "enabled" : "disabled");
  } else {
    cfg_.has_encoders = false;
    RCLCPP_INFO(logger_, "Encoder support: disabled (default)");
  }

  // Parse and validate communication parameters
  cfg_.device = info_.hardware_parameters.find("device") != info_.hardware_parameters.end() ?
    info_.hardware_parameters["device"] : "";
  RCLCPP_INFO(
    logger_, "Serial device: %s",
    cfg_.device.empty() ? "auto-detect" : cfg_.device.c_str());

  // Parse baud rate with validation
  try {
    cfg_.baud_rate = info_.hardware_parameters.find("baud_rate") !=
      info_.hardware_parameters.end() ?
      std::stoi(info_.hardware_parameters["baud_rate"]) : 115200;
    if (cfg_.baud_rate <= 0) {
      RCLCPP_WARN(logger_, "Invalid baud rate %d, using default 115200", cfg_.baud_rate);
      cfg_.baud_rate = 115200;
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Failed to parse baud_rate parameter: %s, using default 115200", e.what());
    cfg_.baud_rate = 115200;
  }
  RCLCPP_INFO(logger_, "Baud rate: %d", cfg_.baud_rate);

  // Parse timeout with validation
  try {
    cfg_.timeout = info_.hardware_parameters.find("timeout") != info_.hardware_parameters.end() ?
      std::stoi(info_.hardware_parameters["timeout"]) : 50;
    if (cfg_.timeout <= 0) {
      RCLCPP_WARN(logger_, "Invalid timeout %d ms, using default 50 ms", cfg_.timeout);
      cfg_.timeout = 50;
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Failed to parse timeout parameter: %s, using default 50 ms", e.what());
    cfg_.timeout = 50;
  }
  RCLCPP_INFO(logger_, "Serial timeout: %d ms", cfg_.timeout);

  // Parse loop rate with validation
  try {
    cfg_.loop_rate = info_.hardware_parameters.find("loop_rate") !=
      info_.hardware_parameters.end() ?
      std::stof(info_.hardware_parameters["loop_rate"]) : 20.0f;
    if (cfg_.loop_rate <= 0.0f || cfg_.loop_rate > 1000.0f) {
      RCLCPP_WARN(logger_, "Invalid loop rate %.2f Hz, using default 20.0 Hz", cfg_.loop_rate);
      cfg_.loop_rate = 20.0f;
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger_, "Failed to parse loop_rate parameter: %s, using default 20.0 Hz",
      e.what());
    cfg_.loop_rate = 20.0f;
  }
  RCLCPP_INFO(logger_, "Loop rate: %.2f Hz", cfg_.loop_rate);

  // Parse and validate velocity limits
  try {
    cfg_.max_lin_vel = info_.hardware_parameters.find("max_lin_vel") !=
      info_.hardware_parameters.end() ?
      std::stof(info_.hardware_parameters["max_lin_vel"]) : 0.3f;
    if (cfg_.max_lin_vel <= 0.0f) {
      RCLCPP_WARN(
        logger_, "Invalid max linear velocity %.3f m/s, using default 0.3 m/s",
        cfg_.max_lin_vel);
      cfg_.max_lin_vel = 0.3f;
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger_, "Failed to parse max_lin_vel parameter: %s, using default 0.3 m/s",
      e.what());
    cfg_.max_lin_vel = 0.3f;
  }
  RCLCPP_INFO(logger_, "Max linear velocity: %.3f m/s", cfg_.max_lin_vel);

  try {
    cfg_.max_ang_vel = info_.hardware_parameters.find("max_ang_vel") !=
      info_.hardware_parameters.end() ?
      std::stof(info_.hardware_parameters["max_ang_vel"]) : 1.0f;
    if (cfg_.max_ang_vel <= 0.0f) {
      RCLCPP_WARN(
        logger_, "Invalid max angular velocity %.3f rad/s, using default 1.0 rad/s",
        cfg_.max_ang_vel);
      cfg_.max_ang_vel = 1.0f;
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger_, "Failed to parse max_ang_vel parameter: %s, using default 1.0 rad/s",
      e.what());
    cfg_.max_ang_vel = 1.0f;
  }
  RCLCPP_INFO(logger_, "Max angular velocity: %.3f rad/s", cfg_.max_ang_vel);

  // Parse and validate encoder parameters (only used if has_encoders = true)
  try {
    cfg_.enc_counts_per_rev = info_.hardware_parameters.find("enc_counts_per_rev") !=
      info_.hardware_parameters.end() ?
      std::stoi(info_.hardware_parameters["enc_counts_per_rev"]) : 1920;
    if (cfg_.enc_counts_per_rev <= 0) {
      RCLCPP_WARN(
        logger_, "Invalid encoder counts per revolution %d, using default 1920",
        cfg_.enc_counts_per_rev);
      cfg_.enc_counts_per_rev = 1920;
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger_, "Failed to parse enc_counts_per_rev parameter: %s, using default 1920",
      e.what());
    cfg_.enc_counts_per_rev = 1920;
  }
  if (cfg_.has_encoders) {
    RCLCPP_INFO(logger_, "Encoder counts per revolution: %d", cfg_.enc_counts_per_rev);
  }

  try {
    cfg_.wheel_radius = info_.hardware_parameters.find("wheel_radius") !=
      info_.hardware_parameters.end() ?
      std::stof(info_.hardware_parameters["wheel_radius"]) : 0.05f;
    if (cfg_.wheel_radius <= 0.0f) {
      RCLCPP_WARN(logger_, "Invalid wheel radius %.3f m, using default 0.05 m", cfg_.wheel_radius);
      cfg_.wheel_radius = 0.05f;
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger_, "Failed to parse wheel_radius parameter: %s, using default 0.05 m",
      e.what());
    cfg_.wheel_radius = 0.05f;
  }
  RCLCPP_INFO(logger_, "Wheel radius: %.3f m", cfg_.wheel_radius);

  // Parse and validate drive-specific parameters
  try {
    cfg_.mix_factor = info_.hardware_parameters.find("mix_factor") !=
      info_.hardware_parameters.end() ?
      std::stof(info_.hardware_parameters["mix_factor"]) : 0.5f;
    if (cfg_.mix_factor < 0.0f || cfg_.mix_factor > 1.0f) {
      RCLCPP_WARN(logger_, "Invalid mix factor %.3f, using default 0.5", cfg_.mix_factor);
      cfg_.mix_factor = 0.5f;
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Failed to parse mix_factor parameter: %s, using default 0.5", e.what());
    cfg_.mix_factor = 0.5f;
  }

  try {
    cfg_.wheel_base = info_.hardware_parameters.find("wheel_base") !=
      info_.hardware_parameters.end() ?
      std::stof(info_.hardware_parameters["wheel_base"]) : 0.3f;
    if (cfg_.wheel_base <= 0.0f) {
      RCLCPP_WARN(logger_, "Invalid wheel base %.3f m, using default 0.3 m", cfg_.wheel_base);
      cfg_.wheel_base = 0.3f;
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Failed to parse wheel_base parameter: %s, using default 0.3 m", e.what());
    cfg_.wheel_base = 0.3f;
  }

  try {
    cfg_.track_width = info_.hardware_parameters.find("track_width") !=
      info_.hardware_parameters.end() ?
      std::stof(info_.hardware_parameters["track_width"]) : 0.3f;
    if (cfg_.track_width <= 0.0f) {
      RCLCPP_WARN(logger_, "Invalid track width %.3f m, using default 0.3 m", cfg_.track_width);
      cfg_.track_width = 0.3f;
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger_, "Failed to parse track_width parameter: %s, using default 0.3 m",
      e.what());
    cfg_.track_width = 0.3f;
  }

  if (cfg_.drive_type == DriveType::MECANUM) {
    RCLCPP_INFO(
      logger_, "Mecanum drive parameters - Wheel base: %.3f m, Track width: %.3f m",
      cfg_.wheel_base, cfg_.track_width);
  }

  // Initialize wheels based on drive configuration
  wheels_.clear();

  switch (cfg_.drive_type) {
    case DriveType::DIFFERENTIAL:
      {
        // Parse wheel names for differential drive
        cfg_.left_wheel_name = info_.hardware_parameters.find("left_wheel_name") !=
          info_.hardware_parameters.end() ?
          info_.hardware_parameters["left_wheel_name"] : "left_wheel";
        cfg_.right_wheel_name = info_.hardware_parameters.find("right_wheel_name") !=
          info_.hardware_parameters.end() ?
          info_.hardware_parameters["right_wheel_name"] : "right_wheel";

        // Create 2 wheels for differential drive
        wheels_.resize(2);
        wheels_[0].setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
        wheels_[1].setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

        RCLCPP_INFO(
          logger_, "Configured for differential drive with wheels: %s, %s",
          cfg_.left_wheel_name.c_str(), cfg_.right_wheel_name.c_str());
      }
      break;

    case DriveType::FOUR_WHEEL:
    case DriveType::MECANUM:
      {
        // Parse wheel names for four-wheel drive
        cfg_.front_left_wheel_name = info_.hardware_parameters.find("front_left_wheel_name") !=
          info_.hardware_parameters.end() ?
          info_.hardware_parameters["front_left_wheel_name"] : "front_left_wheel";
        cfg_.front_right_wheel_name = info_.hardware_parameters.find("front_right_wheel_name") !=
          info_.hardware_parameters.end() ?
          info_.hardware_parameters["front_right_wheel_name"] : "front_right_wheel";
        cfg_.rear_left_wheel_name = info_.hardware_parameters.find("rear_left_wheel_name") !=
          info_.hardware_parameters.end() ?
          info_.hardware_parameters["rear_left_wheel_name"] : "rear_left_wheel";
        cfg_.rear_right_wheel_name = info_.hardware_parameters.find("rear_right_wheel_name") !=
          info_.hardware_parameters.end() ?
          info_.hardware_parameters["rear_right_wheel_name"] : "rear_right_wheel";

        // Create 4 wheels for four-wheel/mecanum drive
        wheels_.resize(4);
        wheels_[0].setup(cfg_.front_left_wheel_name, cfg_.enc_counts_per_rev);
        wheels_[1].setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev);
        wheels_[2].setup(cfg_.rear_left_wheel_name, cfg_.enc_counts_per_rev);
        wheels_[3].setup(cfg_.rear_right_wheel_name, cfg_.enc_counts_per_rev);

        const char * drive_type_name =
          (cfg_.drive_type == DriveType::MECANUM) ? "mecanum" : "four-wheel";
        RCLCPP_INFO(
          logger_, "Configured for %s drive with wheels: %s, %s, %s, %s",
          drive_type_name,
          cfg_.front_left_wheel_name.c_str(), cfg_.front_right_wheel_name.c_str(),
          cfg_.rear_left_wheel_name.c_str(), cfg_.rear_right_wheel_name.c_str());
      }
      break;
  }

  // Set up the serial protocol communication with enhanced error handling
  try {
    RCLCPP_INFO(logger_, "Attempting to establish MecaBridge communication...");
    serial_protocol_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);
    RCLCPP_INFO(logger_, "MecaBridge communication established successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to setup MecaBridge communication: %s", e.what());
    RCLCPP_ERROR(logger_, "Check that the MecaBridge device is connected and accessible");
    if (cfg_.device.empty()) {
      RCLCPP_ERROR(
        logger_,
        "Auto-detection failed. Try specifying the device parameter explicitly");
    } else {
      RCLCPP_ERROR(logger_, "Failed to connect to specified device: %s", cfg_.device.c_str());
    }
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "MecaBridge Hardware Interface configured successfully");
  RCLCPP_INFO(logger_, "Configuration summary:");
  RCLCPP_INFO(
    logger_, "  Drive type: %s",
    cfg_.drive_type == DriveType::DIFFERENTIAL ? "differential" :
    cfg_.drive_type == DriveType::FOUR_WHEEL ? "four_wheel" : "mecanum");
  RCLCPP_INFO(logger_, "  Encoders: %s", cfg_.has_encoders ? "enabled" : "disabled");
  RCLCPP_INFO(
    logger_, "  Serial device: %s",
    cfg_.device.empty() ? "auto-detected" : cfg_.device.c_str());
  RCLCPP_INFO(logger_, "  Baud rate: %d", cfg_.baud_rate);
  RCLCPP_INFO(logger_, "  Timeout: %d ms", cfg_.timeout);
  RCLCPP_INFO(
    logger_, "  Max velocities: %.3f m/s linear, %.3f rad/s angular", cfg_.max_lin_vel,
    cfg_.max_ang_vel);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MecaBridgeHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Export state interfaces for all wheels based on drive configuration
  for (auto & wheel : wheels_) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        wheel.name, hardware_interface::HW_IF_VELOCITY, &wheel.vel));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        wheel.name, hardware_interface::HW_IF_POSITION, &wheel.pos));
  }

  RCLCPP_INFO(logger_, "Exported %zu state interfaces", state_interfaces.size());
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MecaBridgeHardwareInterface::
export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Export command interfaces for all wheels based on drive configuration
  for (auto & wheel : wheels_) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        wheel.name, hardware_interface::HW_IF_VELOCITY, &wheel.cmd));
  }

  RCLCPP_INFO(logger_, "Exported %zu command interfaces", command_interfaces.size());
  return command_interfaces;
}

hardware_interface::CallbackReturn MecaBridgeHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Activating MecaBridge Hardware Interface...");

  // Check connection status with detailed diagnostics
  if (!serial_protocol_.connected()) {
    RCLCPP_ERROR(logger_, "MecaBridge device not connected, cannot activate interface");
    RCLCPP_ERROR(logger_, "Ensure the MecaBridge device is properly connected and configured");

    // Provide detailed connection diagnostics
    int write_errors, read_errors, reconnection_attempts;
    serial_protocol_.getConnectionStats(write_errors, read_errors, reconnection_attempts);
    RCLCPP_ERROR(
      logger_, "Connection statistics: write_errors=%d, read_errors=%d, reconnection_attempts=%d",
      write_errors, read_errors, reconnection_attempts);

    RCLCPP_ERROR(
      logger_,
      "CRITICAL: Cannot activate MecaBridge interface - check device connection and permissions");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Send initial ping for synchronization with error handling
  try {
    RCLCPP_INFO(logger_, "Sending synchronization ping to MecaBridge...");
    serial_protocol_.sendPing();
    RCLCPP_INFO(logger_, "Synchronization ping sent successfully");
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Failed to send synchronization ping: %s", e.what());
    RCLCPP_WARN(logger_, "Continuing activation, but communication may be unreliable");
  }

  // Initialize wheel states with logging
  RCLCPP_INFO(logger_, "Initializing %zu wheel states...", wheels_.size());
  for (size_t i = 0; i < wheels_.size(); ++i) {
    wheels_[i].enc = 0;
    wheels_[i].pos = 0.0;
    wheels_[i].vel = 0.0;
    wheels_[i].cmd = 0.0;
    wheels_[i].eff = 0.0;
    wheels_[i].velSetPt = 0.0;
    RCLCPP_DEBUG(logger_, "Initialized wheel %zu (%s)", i, wheels_[i].name.c_str());
  }

  RCLCPP_INFO(logger_, "MecaBridge Hardware Interface activated successfully");
  RCLCPP_INFO(logger_, "Interface is ready to receive commands and provide state feedback");
  RCLCPP_INFO(logger_, "Activation summary:");
  RCLCPP_INFO(
    logger_, "  Drive type: %s",
    cfg_.drive_type == DriveType::DIFFERENTIAL ? "differential" :
    cfg_.drive_type == DriveType::FOUR_WHEEL ? "four_wheel" : "mecanum");
  RCLCPP_INFO(logger_, "  Wheels configured: %zu", wheels_.size());
  RCLCPP_INFO(logger_, "  Encoder support: %s", cfg_.has_encoders ? "enabled" : "disabled");
  RCLCPP_INFO(logger_, "  Serial connection: active");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecaBridgeHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Deactivating MecaBridge Hardware Interface...");

  // Stop all motors by sending zero commands with error handling
  try {
    if (serial_protocol_.connected()) {
      RCLCPP_INFO(logger_, "Stopping all motors...");
      switch (cfg_.drive_type) {
        case DriveType::DIFFERENTIAL:
          serial_protocol_.setDifferentialMotors(0, 0);
          RCLCPP_INFO(logger_, "Differential motors stopped");
          break;
        case DriveType::FOUR_WHEEL:
        case DriveType::MECANUM:
          serial_protocol_.setFourMotors(0, 0, 0, 0);
          RCLCPP_INFO(logger_, "All four motors stopped");
          break;
      }
    } else {
      RCLCPP_WARN(logger_, "MecaBridge device not connected, cannot send stop commands");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to stop motors during deactivation: %s", e.what());
    RCLCPP_WARN(logger_, "Motors may still be running - check hardware manually");
  }

  // Reset wheel command states
  for (auto & wheel : wheels_) {
    wheel.cmd = 0.0;
    wheel.velSetPt = 0.0;
  }

  RCLCPP_INFO(logger_, "MecaBridge Hardware Interface deactivated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MecaBridgeHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Check connection status with automatic recovery attempt
  if (!serial_protocol_.connected()) {
    connection_error_count_++;

    // Attempt recovery periodically
    if (connection_error_count_ % 100 == 1) {  // Try recovery every 100 failures
      RCLCPP_ERROR(
        logger_, "MecaBridge not connected, cannot read state (error count: %d)",
        connection_error_count_);

      if (attemptConnectionRecovery()) {
        RCLCPP_INFO(logger_, "Connection recovered, resuming normal operation");
        connection_error_count_ = 0;  // Reset error count on successful recovery
        read_error_count_ = 0;  // Reset read errors on successful recovery
        serial_protocol_.resetErrorCounters();  // Reset communication layer error counters
      } else {
        RCLCPP_ERROR(
          logger_,
          "Connection recovery failed - check serial connection and device availability");

        // Provide detailed error information every 500 failures
        if (connection_error_count_ % 500 == 0) {
          RCLCPP_ERROR(
            logger_,
            "CRITICAL: Persistent connection failure in read() - check hardware connections");
        }
      }
    }

    // Return error but allow system to continue
    return hardware_interface::return_type::ERROR;
  }

  try {
    if (cfg_.has_encoders) {
      // Read encoder values and calculate position/velocity from encoder feedback
      switch (cfg_.drive_type) {
        case DriveType::DIFFERENTIAL:
          {
            int left_enc, right_enc;
            serial_protocol_.readDifferentialEncoders(left_enc, right_enc);

            // Update encoder counts and calculate positions
            wheels_[0].enc = left_enc;
            wheels_[1].enc = right_enc;

            // Calculate new positions from encoder counts
            double new_left_pos = wheels_[0].calcEncAngle();
            double new_right_pos = wheels_[1].calcEncAngle();

            // Calculate velocities from position change over time
            double dt = period.seconds();
            if (dt > 0.0) {
              wheels_[0].vel = (new_left_pos - wheels_[0].pos) / dt;
              wheels_[1].vel = (new_right_pos - wheels_[1].pos) / dt;
            }

            // Update positions
            wheels_[0].pos = new_left_pos;
            wheels_[1].pos = new_right_pos;
          }
          break;

        case DriveType::FOUR_WHEEL:
        case DriveType::MECANUM:
          {
            int fl_enc, fr_enc, rl_enc, rr_enc;
            serial_protocol_.readFourEncoders(fl_enc, fr_enc, rl_enc, rr_enc);

            // Update encoder counts and calculate positions
            wheels_[0].enc = fl_enc;  // front_left
            wheels_[1].enc = fr_enc;  // front_right
            wheels_[2].enc = rl_enc;  // rear_left
            wheels_[3].enc = rr_enc;  // rear_right

            // Calculate new positions from encoder counts
            std::vector<double> new_positions(4);
            for (size_t i = 0; i < 4; ++i) {
              new_positions[i] = wheels_[i].calcEncAngle();
            }

            // Calculate velocities from position change over time
            double dt = period.seconds();
            if (dt > 0.0) {
              for (size_t i = 0; i < 4; ++i) {
                wheels_[i].vel = (new_positions[i] - wheels_[i].pos) / dt;
                wheels_[i].pos = new_positions[i];
              }
            } else {
              // Update positions even if dt is 0
              for (size_t i = 0; i < 4; ++i) {
                wheels_[i].pos = new_positions[i];
              }
            }
          }
          break;
      }
    } else {
      // No encoders: integrate velocity commands over time to estimate position
      double dt = period.seconds();

      for (auto & wheel : wheels_) {
        // Use the velocity command as the actual velocity (open-loop assumption)
        wheel.vel = wheel.cmd;

        // Integrate velocity to get position
        if (dt > 0.0) {
          wheel.pos += wheel.vel * dt;
        }
      }
    }

    return hardware_interface::return_type::OK;
  } catch (const std::exception & e) {
    read_error_count_++;

    if (read_error_count_ % 50 == 1) {  // Log every 50 failures to avoid spam
      RCLCPP_ERROR(
        logger_, "Error reading from MecaBridge device (error count: %d): %s", read_error_count_,
        e.what());
      RCLCPP_ERROR(logger_, "This may indicate communication issues or hardware problems");
    }

    // Provide detailed diagnostics for persistent read errors
    if (read_error_count_ % 200 == 0) {
      RCLCPP_ERROR(
        logger_,
        "CRITICAL: Persistent read errors from MecaBridge device - check serial communication");
    }

    // Continue operation with last known values rather than stopping completely
    RCLCPP_DEBUG(logger_, "Continuing with last known wheel states due to read error");
    return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::return_type MecaBridgeHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Check connection status with automatic recovery attempt
  if (!serial_protocol_.connected()) {
    connection_error_count_++;

    // Attempt recovery periodically
    if (connection_error_count_ % 100 == 1) {  // Try recovery every 100 failures
      RCLCPP_ERROR(
        logger_, "MecaBridge device not connected, cannot write commands (error count: %d)",
        connection_error_count_);

      if (attemptConnectionRecovery()) {
        RCLCPP_INFO(logger_, "Connection recovered, resuming command transmission");
        connection_error_count_ = 0;  // Reset error count on successful recovery
        write_error_count_ = 0;  // Reset write errors on successful recovery
        serial_protocol_.resetErrorCounters();  // Reset communication layer error counters
      } else {
        RCLCPP_ERROR(
          logger_,
          "Connection recovery failed - robot will not respond to movement commands");

        // Provide detailed error information every 500 failures
        if (connection_error_count_ % 500 == 0) {
          RCLCPP_ERROR(
            logger_,
            "CRITICAL: Persistent connection failure in write() - robot not receiving commands");
        }
      }
    }

    // Return error but allow system to continue
    return hardware_interface::return_type::ERROR;
  }

  try {
    switch (cfg_.drive_type) {
      case DriveType::DIFFERENTIAL:
        {
          // Differential drive: convert left/right wheel velocities to motor commands
          if (wheels_.size() != 2) {
            RCLCPP_ERROR(
              logger_, "Expected 2 wheels for differential drive, got %zu",
              wheels_.size());
            return hardware_interface::return_type::ERROR;
          }

          // Get velocity commands for left and right wheels (rad/s)
          double left_vel_cmd = wheels_[0].cmd;   // left wheel
          double right_vel_cmd = wheels_[1].cmd;  // right wheel

          // Convert wheel velocities to motor command values (-100 to 100)
          int left_motor_cmd = convertVelocityToMotorCommand(left_vel_cmd);
          int right_motor_cmd = convertVelocityToMotorCommand(right_vel_cmd);

          // Send differential motor commands
          serial_protocol_.setDifferentialMotors(left_motor_cmd, right_motor_cmd);

          // Store the velocity setpoints for debugging
          wheels_[0].velSetPt = left_vel_cmd;
          wheels_[1].velSetPt = right_vel_cmd;

          // Debug logging (throttled to avoid spam)
          static int log_counter = 0;
          if (++log_counter % 100 == 0) {  // Log every 100 calls (~5Hz at 20Hz)
            RCLCPP_DEBUG(
              logger_, "Differential: L_vel=%.3f->%d, R_vel=%.3f->%d",
              left_vel_cmd, left_motor_cmd, right_vel_cmd, right_motor_cmd);
          }
        }
        break;

      case DriveType::FOUR_WHEEL:
        {
          // Four-wheel independent drive: direct control of each wheel
          if (wheels_.size() != 4) {
            RCLCPP_ERROR(
              logger_, "Expected 4 wheels for four-wheel drive, got %zu",
              wheels_.size());
            return hardware_interface::return_type::ERROR;
          }

          // Get velocity commands for all four wheels (rad/s)
          double fl_vel_cmd = wheels_[0].cmd;  // front_left
          double fr_vel_cmd = wheels_[1].cmd;  // front_right
          double rl_vel_cmd = wheels_[2].cmd;  // rear_left
          double rr_vel_cmd = wheels_[3].cmd;  // rear_right

          // Convert wheel velocities to motor command values (-100 to 100)
          int fl_motor_cmd = convertVelocityToMotorCommand(fl_vel_cmd);
          int fr_motor_cmd = convertVelocityToMotorCommand(fr_vel_cmd);
          int rl_motor_cmd = convertVelocityToMotorCommand(rl_vel_cmd);
          int rr_motor_cmd = convertVelocityToMotorCommand(rr_vel_cmd);

          // Send four motor commands
          serial_protocol_.setFourMotors(fl_motor_cmd, fr_motor_cmd, rl_motor_cmd, rr_motor_cmd);

          // Store the velocity setpoints for debugging
          wheels_[0].velSetPt = fl_vel_cmd;
          wheels_[1].velSetPt = fr_vel_cmd;
          wheels_[2].velSetPt = rl_vel_cmd;
          wheels_[3].velSetPt = rr_vel_cmd;
        }
        break;

      case DriveType::MECANUM:
        {
          // Mecanum drive: use inverse kinematics to convert Twist to wheel velocities
          if (wheels_.size() != 4) {
            RCLCPP_ERROR(logger_, "Expected 4 wheels for mecanum drive, got %zu", wheels_.size());
            return hardware_interface::return_type::ERROR;
          }

          // For mecanum drive, the wheel commands should already be computed by the controller
          // (e.g., mecanum_drive_controller) and passed as individual wheel velocity commands
          // We just need to convert them to motor commands

          // Get velocity commands for all four wheels (rad/s)
          double fl_vel_cmd = wheels_[0].cmd;  // front_left
          double fr_vel_cmd = wheels_[1].cmd;  // front_right
          double rl_vel_cmd = wheels_[2].cmd;  // rear_left
          double rr_vel_cmd = wheels_[3].cmd;  // rear_right

          // Convert wheel velocities to motor command values (-100 to 100)
          int fl_motor_cmd = convertVelocityToMotorCommand(fl_vel_cmd);
          int fr_motor_cmd = convertVelocityToMotorCommand(fr_vel_cmd);
          int rl_motor_cmd = convertVelocityToMotorCommand(rl_vel_cmd);
          int rr_motor_cmd = convertVelocityToMotorCommand(rr_vel_cmd);

          // Send four motor commands
          serial_protocol_.setFourMotors(fl_motor_cmd, fr_motor_cmd, rl_motor_cmd, rr_motor_cmd);

          // Store the velocity setpoints for debugging
          wheels_[0].velSetPt = fl_vel_cmd;
          wheels_[1].velSetPt = fr_vel_cmd;
          wheels_[2].velSetPt = rl_vel_cmd;
          wheels_[3].velSetPt = rr_vel_cmd;
        }
        break;

      default:
        RCLCPP_ERROR(logger_, "Unknown drive type in write() method");
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
  } catch (const std::exception & e) {
    write_error_count_++;

    if (write_error_count_ % 50 == 1) {  // Log every 50 failures to avoid spam
      RCLCPP_ERROR(
        logger_, "Error writing to MecaBridge device (error count: %d): %s", write_error_count_,
        e.what());
      RCLCPP_ERROR(logger_, "Motor commands may not be reaching the hardware");
      RCLCPP_WARN(logger_, "Check serial connection stability and hardware status");
    }

    // Provide detailed diagnostics for persistent write errors
    if (write_error_count_ % 200 == 0) {
      RCLCPP_ERROR(
        logger_,
        "CRITICAL: Persistent write errors to MecaBridge device - motor commands not transmitted");
    }

    // Continue operation - don't stop the entire system due to communication errors
    RCLCPP_DEBUG(logger_, "Continuing operation despite write error - system remains responsive");
    return hardware_interface::return_type::ERROR;
  }
}

int MecaBridgeHardwareInterface::convertVelocityToMotorCommand(double wheel_vel_rad_s)
{
  // Convert wheel velocity (rad/s) to motor command value (-100 to 100)
  // Based on the maximum velocity configuration and MecaBridge command range

  // Calculate the maximum wheel velocity in rad/s based on linear velocity limit
  // For a wheel: v_linear = v_angular * wheel_radius
  // So: v_angular_max = v_linear_max / wheel_radius
  double max_wheel_vel_rad_s = cfg_.max_lin_vel / cfg_.wheel_radius;

  // Normalize the velocity command to [-1.0, 1.0] range
  double normalized_vel = wheel_vel_rad_s / max_wheel_vel_rad_s;

  // Clamp to valid range
  normalized_vel = std::max(-1.0, std::min(1.0, normalized_vel));

  // Convert to motor command range [-100, 100]
  int motor_cmd = static_cast<int>(std::round(100.0 * normalized_vel));

  // Final clamp to ensure we're within MecaBridge command range
  motor_cmd = std::max(-100, std::min(100, motor_cmd));

  return motor_cmd;
}

bool MecaBridgeHardwareInterface::attemptConnectionRecovery()
{
  static auto last_recovery_attempt = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();

  // Limit recovery attempts to avoid overwhelming the system
  const auto min_recovery_interval = std::chrono::seconds(5);
  if (now - last_recovery_attempt < min_recovery_interval) {
    RCLCPP_DEBUG(logger_, "Skipping recovery attempt - too soon since last attempt");
    return false;
  }

  last_recovery_attempt = now;

  RCLCPP_WARN(logger_, "Attempting connection recovery for MecaBridge device...");
  RCLCPP_INFO(logger_, "Recovery attempt details:");
  RCLCPP_INFO(logger_, "  Device: %s", cfg_.device.empty() ? "auto-detect" : cfg_.device.c_str());
  RCLCPP_INFO(logger_, "  Baud rate: %d", cfg_.baud_rate);
  RCLCPP_INFO(logger_, "  Timeout: %d ms", cfg_.timeout);

  try {
    // Attempt to reconnect using the MecaBridgeSerialProtocol reconnection method
    bool recovery_success = serial_protocol_.attemptReconnection();

    if (recovery_success) {
      RCLCPP_INFO(logger_, "Connection recovery successful!");
      RCLCPP_INFO(logger_, "MecaBridge hardware interface is now operational");

      // Send a test ping to verify communication
      try {
        serial_protocol_.sendPing();
        RCLCPP_INFO(logger_, "Communication test (ping) successful");
        return true;
      } catch (const std::exception & e) {
        RCLCPP_WARN(logger_, "Connection recovered but communication test failed: %s", e.what());
        RCLCPP_WARN(logger_, "Connection may be unstable");
        return false;
      }
    } else {
      RCLCPP_ERROR(logger_, "Connection recovery failed");
      RCLCPP_ERROR(logger_, "Hardware interface will continue in degraded mode");
      RCLCPP_ERROR(logger_, "Manual intervention may be required:");
      RCLCPP_ERROR(logger_, "  - Check physical connections");
      RCLCPP_ERROR(logger_, "  - Verify device power");
      RCLCPP_ERROR(logger_, "  - Restart the hardware interface node");
      return false;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Exception during connection recovery: %s", e.what());
    RCLCPP_ERROR(logger_, "Recovery process encountered an unexpected error");
    return false;
  }
}

// Validation and logging methods will be added back after fixing compilation issues

}  // namespace mecabridge_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mecabridge_hardware::MecaBridgeHardwareInterface,
  hardware_interface::SystemInterface
)
