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

// Primary include guard
#ifndef MECABRIDGE_UTILS__CONFIG__CONFIG_HPP_
#define MECABRIDGE_UTILS__CONFIG__CONFIG_HPP_

#pragma once  // Optional, redundant with guard but retained for faster builds

#include <string>
#include <array>

#include <cstdint>
#include <stdexcept>

namespace mecabridge
{
namespace config
{

struct WheelDef
{
  int encoder_index = -1;     // 0..3
  std::string joint_name;
};

struct WheelsConfig
{
  WheelDef front_left;
  WheelDef front_right;
  WheelDef rear_left;
  WheelDef rear_right;
};

struct ServoPositionalConfig
{
  std::string joint_name;
  double min_rad = 0.0;
  double max_rad = 0.0;
};

struct ServoContinuousConfig
{
  std::string joint_name;
  double max_velocity_rad_s = 0.0;     // optional clamp (0 means disabled)
};

struct ServosConfig
{
  ServoPositionalConfig positional;
  ServoContinuousConfig continuous;
};

struct EscConfig
{
  std::string joint_name;
  int esc_min_pwm = 0;
  int esc_max_pwm = 0;
  int esc_deadband = 0;     // optional
};

struct EscsConfig
{
  EscConfig left;
  EscConfig right;
};

struct FeaturesConfig
{
  bool enable_servos = true;
  bool enable_escs = true;
  bool enable_diagnostics = false;
};

struct Config
{
  // Top level
  std::string serial_port;
  int baud_rate = 115200;
  int state_publish_rate_hz = 50;     // 10..200

  // Geometry / mechanics
  double wheel_radius = 0.0;           // >0
  double wheel_separation_x = 0.0;     // >0
  double wheel_separation_y = 0.0;     // >0
  int encoder_ticks_per_rev = 0;       // >0

  WheelsConfig wheels;
  ServosConfig servos;
  EscsConfig escs;
  FeaturesConfig features;

  // Validate constraints; throws std::runtime_error on failure
  void validate() const;

  // Convenience helpers to build deterministic arrays
  std::array<int, 4> wheel_encoder_indices() const
  {
    return {wheels.front_left.encoder_index,
      wheels.front_right.encoder_index,
      wheels.rear_left.encoder_index,
      wheels.rear_right.encoder_index};
  }
  std::array<std::string, 4> wheel_joint_names() const
  {
    return {wheels.front_left.joint_name,
      wheels.front_right.joint_name,
      wheels.rear_left.joint_name,
      wheels.rear_right.joint_name};
  }
};

// Parse a YAML string (minimal parser supporting the documented schema)
// Throws std::runtime_error on parse/validation error.
Config parse_from_yaml_string(const std::string & yaml);

// Load from a YAML file path
Config parse_from_yaml_file(const std::string & path);

}   // namespace config
} // namespace mecabridge

#endif  // MECABRIDGE_UTILS__CONFIG__CONFIG_HPP_
