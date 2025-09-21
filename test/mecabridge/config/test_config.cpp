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

#include <gtest/gtest.h>
#include "mecabridge_utils/config/config.hpp"

using mecabridge::config::Config;
using mecabridge::config::parse_from_yaml_string;

static const char * kValidYaml =
  R"YAML(
mecabridge_hardware:
  serial_port: /dev/ttyACM0
  baud_rate: 115200
  state_publish_rate_hz: 50
  wheel_radius: 0.048
  wheel_separation_x: 0.20
  wheel_separation_y: 0.18
  encoder_ticks_per_rev: 1024
  wheels:
    front_left:  { encoder_index: 0, joint_name: front_left_wheel_joint }
    front_right: { encoder_index: 1, joint_name: front_right_wheel_joint }
    rear_left:   { encoder_index: 2, joint_name: rear_left_wheel_joint }
    rear_right:  { encoder_index: 3, joint_name: rear_right_wheel_joint }
  servos:
    positional:
      joint_name: pan_servo_joint
      min_rad: 0.0
      max_rad: 3.14159
    continuous:
      joint_name: rot_servo_joint
      max_velocity_rad_s: 10.0
  escs:
    left:
      joint_name: esc_left_joint
      esc_min_pwm: 1000
      esc_max_pwm: 2000
      esc_deadband: 0
    right:
      joint_name: esc_right_joint
      esc_min_pwm: 1000
      esc_max_pwm: 2000
      esc_deadband: 0
  features:
    enable_servos: true
    enable_escs: true
    enable_diagnostics: false
)YAML";

TEST(ConfigLoader, ParsesValidConfig) {
  Config cfg = parse_from_yaml_string(kValidYaml);
  EXPECT_EQ(cfg.serial_port, "/dev/ttyACM0");
  EXPECT_EQ(cfg.baud_rate, 115200);
  EXPECT_EQ(cfg.state_publish_rate_hz, 50);
  EXPECT_NEAR(cfg.wheel_radius, 0.048, 1e-9);
  EXPECT_NEAR(cfg.wheel_separation_x, 0.20, 1e-9);
  EXPECT_NEAR(cfg.wheel_separation_y, 0.18, 1e-9);
  EXPECT_EQ(cfg.encoder_ticks_per_rev, 1024);
  auto idx = cfg.wheel_encoder_indices();
  EXPECT_EQ(idx[0], 0);
  EXPECT_EQ(idx[1], 1);
  EXPECT_EQ(idx[2], 2);
  EXPECT_EQ(idx[3], 3);
  auto names = cfg.wheel_joint_names();
  EXPECT_EQ(names[0], "front_left_wheel_joint");
  EXPECT_EQ(names[3], "rear_right_wheel_joint");
  EXPECT_EQ(cfg.servos.positional.joint_name, "pan_servo_joint");
  EXPECT_LT(cfg.servos.positional.min_rad, cfg.servos.positional.max_rad);
  EXPECT_EQ(cfg.escs.left.joint_name, "esc_left_joint");
  EXPECT_EQ(cfg.features.enable_servos, true);
}

static void expect_parse_error(const std::string & yaml, const char * msg_contains)
{
  try {
    (void)parse_from_yaml_string(yaml);
    FAIL() << "Expected parse/validation error";
  } catch (const std::exception & e) {
    std::string m = e.what();
    if (msg_contains) {
      ASSERT_NE(m.find(msg_contains), std::string::npos) << m;
    }
  }
}

TEST(ConfigLoader, MissingSerialPort) {
  std::string y = std::string(kValidYaml);
  auto pos = y.find("serial_port:");
  ASSERT_NE(pos, std::string::npos);
  y.erase(pos, 30);
  expect_parse_error(y, "serial_port missing");
}

TEST(ConfigLoader, PublishRateOutOfRange) {
  std::string y = std::string(kValidYaml);
  auto pos = y.find("state_publish_rate_hz:");
  ASSERT_NE(pos, std::string::npos);
  auto end = y.find('\n', pos);
  y.replace(pos, end - pos, "state_publish_rate_hz: 5");
  expect_parse_error(y, "state_publish_rate_hz out of range");
}

TEST(ConfigLoader, WheelRadiusPositive) {
  std::string y = std::string(kValidYaml);
  auto pos = y.find("wheel_radius:");
  ASSERT_NE(pos, std::string::npos);
  auto end = y.find('\n', pos);
  y.replace(pos, end - pos, "wheel_radius: 0.0");
  expect_parse_error(y, "wheel_radius must be >0");
}

TEST(ConfigLoader, SeparationPositive) {
  std::string y = std::string(kValidYaml);
  auto pos = y.find("wheel_separation_x:");
  ASSERT_NE(pos, std::string::npos);
  auto end = y.find('\n', pos);
  y.replace(pos, end - pos, "wheel_separation_x: 0.0");
  expect_parse_error(y, "wheel separation must be >0");
}

TEST(ConfigLoader, EncoderTicksPositive) {
  std::string y = std::string(kValidYaml);
  auto pos = y.find("encoder_ticks_per_rev:");
  ASSERT_NE(pos, std::string::npos);
  auto end = y.find('\n', pos);
  y.replace(pos, end - pos, "encoder_ticks_per_rev: 0");
  expect_parse_error(y, "encoder_ticks_per_rev must be >0");
}

TEST(ConfigLoader, DuplicateEncoderIndex) {
  std::string y = std::string(kValidYaml);
  // make two wheels share index 0
  auto pos = y.find("front_right:");
  ASSERT_NE(pos, std::string::npos);
  auto end = y.find('\n', pos);
  y.replace(
    pos, end - pos,
    "front_right: { encoder_index: 0, joint_name: front_right_wheel_joint }");
  expect_parse_error(y, "duplicate or invalid encoder_index");
}

TEST(ConfigLoader, DuplicateJointName) {
  std::string y = std::string(kValidYaml);
  // duplicate front_left joint name to rear_right
  auto pos = y.find("rear_right:");
  ASSERT_NE(pos, std::string::npos);
  auto end = y.find('\n', pos);
  y.replace(
    pos, end - pos,
    "rear_right:  { encoder_index: 3, joint_name: front_left_wheel_joint }");
  expect_parse_error(y, "duplicate joint_name");
}

TEST(ConfigLoader, ServoMinLessThanMax) {
  std::string y = std::string(kValidYaml);
  auto pos = y.find("min_rad:");
  ASSERT_NE(pos, std::string::npos);
  auto end = y.find('\n', pos);
  y.replace(pos, end - pos, "min_rad: 3.2");
  expect_parse_error(y, "servo positional min>=max");
}

TEST(ConfigLoader, EscPwmRange) {
  std::string y = std::string(kValidYaml);
  auto pos = y.find("esc_max_pwm:");
  ASSERT_NE(pos, std::string::npos);
  auto end = y.find('\n', pos);
  y.replace(pos, end - pos, "esc_max_pwm: 900");
  expect_parse_error(y, "esc pwm range invalid");
}
