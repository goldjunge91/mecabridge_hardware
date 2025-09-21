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

#ifndef MECABRIDGE_HARDWARE_MECABRIDGE_DRIVE_CONFIG_H
#define MECABRIDGE_HARDWARE_MECABRIDGE_DRIVE_CONFIG_H

#include <string>

namespace mecabridge_hardware
{

  enum class DriveType
  {
    DIFFERENTIAL,  // 2 motors, differential drive
    FOUR_WHEEL,    // 4 motors, independent control
    MECANUM        // 4 motors, mecanum kinematics
  };

  struct MecaBridgeDriveConfig
  {
    // Drive configuration
    DriveType drive_type = DriveType::DIFFERENTIAL;
    bool has_encoders = false; // Whether the setup includes encoders

    // Wheel names (varies by drive type)
    std::string left_wheel_name = "left_wheel";
    std::string right_wheel_name = "right_wheel";
    std::string front_left_wheel_name = "front_left_wheel";
    std::string front_right_wheel_name = "front_right_wheel";
    std::string rear_left_wheel_name = "rear_left_wheel";
    std::string rear_right_wheel_name = "rear_right_wheel";

    // Communication parameters
    float loop_rate = 20.0;
    std::string device = "";
    int baud_rate = 115200;
    int timeout = 50;

    // Velocity limits
    float max_lin_vel = 0.3; // m/s
    float max_ang_vel = 1.0; // rad/s

    // Drive-specific parameters
    float mix_factor = 0.5; // For differential drive
    float wheel_base = 0.3; // L - distance between front and rear axles (mecanum)
    float track_width = 0.3; // W - distance between left and right wheels (mecanum)

    // Encoder parameters (only used if has_encoders = true)
    int enc_counts_per_rev = 1920;
    float wheel_radius = 0.05; // meters
  };

}  // namespace mecabridge_hardware

#endif  // MECABRIDGE_HARDWARE_MECABRIDGE_DRIVE_CONFIG_H
