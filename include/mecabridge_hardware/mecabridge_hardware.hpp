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

#ifndef MECABRIDGE_HARDWARE__MECABRIDGE_HARDWARE_HPP_
#define MECABRIDGE_HARDWARE__MECABRIDGE_HARDWARE_HPP_


#pragma once

#include <string>
#include <vector>
#include <memory>
#include <chrono>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "mecabridge_utils/config/config.hpp"
#include "mecabridge_utils/protocol/frame.hpp"
#include "mecabridge_utils/serial/serial_backend.hpp"
#include "mecabridge_utils/serial/mock_serial_backend.hpp"
#include "mecabridge_utils/serial/loopback_serial_backend.hpp"
#include "mecabridge_utils/safety/watchdog.hpp"
#include "mecabridge_utils/latency/latency_tracker.hpp"
#include "mecabridge_hardware/feature_config.hpp"

namespace mecabridge_hardware
{

class MecaBridgeHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MecaBridgeHardware)

  MecaBridgeHardware() = default;
  ~MecaBridgeHardware() override = default;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state)
  override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // Deterministic joint ordering: 4 wheels (vel), 1 positional servo (pos), 1 continuous servo (vel), 2 ESC (vel normalized)
  std::vector<double> hw_states_{};   // size 8-? state interfaces
  std::vector<double> hw_commands_{}; // size depends on interfaces

  mecabridge::config::Config cfg_;

  // Protocol and serial communication
  std::unique_ptr<mecabridge::serial::SerialBackend> serial_backend_;
  mecabridge::safety::Watchdog watchdog_;
  uint16_t command_sequence_{0};
  std::chrono::steady_clock::time_point last_state_time_;

  // Frame buffers
  uint8_t tx_buffer_[128];
  uint8_t rx_buffer_[128];

  bool configured_ = false;
  bool active_ = false;

  // Latency measurement
  mecabridge::latency::LatencyTracker latency_tracker_;

  // Helper methods for protocol integration
  hardware_interface::return_type readStateFrame();
  hardware_interface::return_type writeCommandFrame();
  void updateStateFromFrame(const mecabridge::protocol::StateFramePayload & state_payload);
  void buildCommandFromState(mecabridge::protocol::CommandFramePayload & command_payload);
  void logLatencyStats();
};

} // namespace mecabridge_hardware
#endif  // MECABRIDGE_HARDWARE__MECABRIDGE_HARDWARE_HPP_
