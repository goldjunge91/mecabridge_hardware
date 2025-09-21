// Copyright (c) 2024 MecaBridge
// Licensed under the Apache License, Version 2.0

#pragma once

#include <string>
#include <vector>

namespace mecabridge
{
namespace config
{
struct WheelDef
{
  int encoder_index = -1;
  std::string joint_name;
};

struct Wheels
{
  WheelDef front_left;
  WheelDef front_right;
  WheelDef rear_left;
  WheelDef rear_right;
};

struct PositionalServo
{
  std::string joint_name;
  double min_rad = -1.0;
  double max_rad = 1.0;
};

struct ContinuousServo
{
  std::string joint_name;
  double max_velocity_rad_s = 0.0;
};

struct EscConfig
{
  std::string joint_name;
  int esc_min_pwm = 1000;
  int esc_max_pwm = 2000;
  int esc_deadband = 20;
};

struct Servos
{
  PositionalServo positional;
  ContinuousServo continuous;
};

struct Features
{
  bool enable_servos = false;
  bool enable_escs = false;
  bool enable_diagnostics = false;
};

struct Escs
{
  EscConfig left;
  EscConfig right;
};

struct Config
{
  std::string serial_port;
  int baud_rate = 115200;
  int state_publish_rate_hz = 50;
  double wheel_radius = 0.05;
  double wheel_separation_x = 0.15;
  double wheel_separation_y = 0.15;
  int encoder_ticks_per_rev = 1024;
  Wheels wheels;
  Servos servos;
  Escs escs;
  Features features;

  std::vector<int> wheel_encoder_indices() const
  {
    return {wheels.front_left.encoder_index, wheels.front_right.encoder_index,
      wheels.rear_left.encoder_index, wheels.rear_right.encoder_index};
  }

  std::vector<std::string> wheel_joint_names() const
  {
    return {wheels.front_left.joint_name, wheels.front_right.joint_name,
      wheels.rear_left.joint_name, wheels.rear_right.joint_name};
  }

  void validate() const;
};

Config parse_from_yaml_file(const std::string & path);
Config parse_from_yaml_string(const std::string & yaml);

} // namespace config
} // namespace mecabridge
