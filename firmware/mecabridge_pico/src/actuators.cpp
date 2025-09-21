// Copyright 2025 MecaBridge Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.#include "actuators.hpp"
#include "actuators.hpp"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

#include <algorithm>

#include <cstring>

namespace mecabridge
{

// Motor Controller Implementation
MotorController::MotorController()
: initialized_(false) {}

bool MotorController::init()
{
  if (initialized_) {return true;}

  // Initialize PWM for motors (placeholder implementation)
  // In a real implementation, you would configure specific GPIO pins for motor PWM
  // For now, we'll use placeholders

  // Motor PWM pins (example - adjust based on hardware)
  const uint motor_pwm_pins[4] = {2, 3, 4, 5};    // FL, FR, RL, RR
  const uint motor_dir_pins[4] = {6, 7, 8, 9};    // Direction pins

  for (int i = 0; i < 4; i++) {
    // Set up PWM
    gpio_set_function(motor_pwm_pins[i], GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(motor_pwm_pins[i]);
    pwm_set_wrap(slice_num, 1000);      // 1000 steps for PWM resolution
    pwm_set_enabled(slice_num, true);

    // Set up direction pin
    gpio_init(motor_dir_pins[i]);
    gpio_set_dir(motor_dir_pins[i], GPIO_OUT);
    gpio_put(motor_dir_pins[i], false);
  }

  initialized_ = true;
  setNeutral();
  return true;
}

void MotorController::applyWheelVelocities(const float wheel_vel_rad_s[4])
{
  if (!initialized_) {return;}

  const uint motor_pwm_pins[4] = {2, 3, 4, 5};
  const uint motor_dir_pins[4] = {6, 7, 8, 9};

  for (int i = 0; i < 4; i++) {
    uint16_t pwm_value = velocityToPWM(std::abs(wheel_vel_rad_s[i]));
    bool direction = wheel_vel_rad_s[i] >= 0;

    setMotorPWM(i, pwm_value, direction);
  }
}

void MotorController::setNeutral()
{
  if (!initialized_) {return;}

  const float neutral_velocities[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  applyWheelVelocities(neutral_velocities);
}

uint16_t MotorController::velocityToPWM(float velocity_rad_s)
{
  // Placeholder implementation: convert rad/s to PWM duty cycle
  // Assume max velocity is 10 rad/s for full PWM
  const float max_velocity = 10.0f;
  float normalized = std::clamp(std::abs(velocity_rad_s) / max_velocity, 0.0f, 1.0f);
  return static_cast<uint16_t>(normalized * 1000);    // 0-1000 PWM range
}

void MotorController::setMotorPWM(uint8_t motor_index, uint16_t pwm_value, bool direction)
{
  if (motor_index >= 4) {return;}

  const uint motor_pwm_pins[4] = {2, 3, 4, 5};
  const uint motor_dir_pins[4] = {6, 7, 8, 9};

  // Set direction
  gpio_put(motor_dir_pins[motor_index], direction);

  // Set PWM
  uint slice_num = pwm_gpio_to_slice_num(motor_pwm_pins[motor_index]);
  uint channel = pwm_gpio_to_channel(motor_pwm_pins[motor_index]);
  pwm_set_chan_level(slice_num, channel, pwm_value);
}

// Servo Controller Implementation
ServoController::ServoController()
: initialized_(false)
  , current_pos_rad_(0.0f)
  , current_vel_norm_(0.0f) {}

bool ServoController::init()
{
  if (initialized_) {return true;}

  // Initialize servo PWM pins (placeholder)
  const uint positional_servo_pin = 10;
  const uint continuous_servo_pin = 11;

  // Set up positional servo PWM
  gpio_set_function(positional_servo_pin, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(positional_servo_pin);
  pwm_set_wrap(slice_num, 20000);    // 20ms period for servo PWM
  pwm_set_enabled(slice_num, true);

  // Set up continuous servo PWM
  gpio_set_function(continuous_servo_pin, GPIO_FUNC_PWM);
  slice_num = pwm_gpio_to_slice_num(continuous_servo_pin);
  pwm_set_wrap(slice_num, 20000);    // 20ms period for servo PWM
  pwm_set_enabled(slice_num, true);

  initialized_ = true;
  setNeutral();
  return true;
}

void ServoController::setPositionalServo(float angle_rad)
{
  if (!initialized_) {return;}

  // Clamp to valid range
  angle_rad = std::clamp(angle_rad, MIN_SERVO_POS_RAD, MAX_SERVO_POS_RAD);
  current_pos_rad_ = angle_rad;

  uint16_t pwm_us = angleToPWM(angle_rad);

  // Set PWM for positional servo
  const uint positional_servo_pin = 10;
  uint slice_num = pwm_gpio_to_slice_num(positional_servo_pin);
  uint channel = pwm_gpio_to_channel(positional_servo_pin);
  pwm_set_chan_level(slice_num, channel, pwm_us);
}

void ServoController::setContinuousServo(float velocity_norm)
{
  if (!initialized_) {return;}

  // Clamp to valid range
  velocity_norm = std::clamp(velocity_norm, -1.0f, 1.0f);
  current_vel_norm_ = velocity_norm;

  uint16_t pwm_us = velocityToPWM(velocity_norm);

  // Set PWM for continuous servo
  const uint continuous_servo_pin = 11;
  uint slice_num = pwm_gpio_to_slice_num(continuous_servo_pin);
  uint channel = pwm_gpio_to_channel(continuous_servo_pin);
  pwm_set_chan_level(slice_num, channel, pwm_us);
}

void ServoController::setNeutral()
{
  if (!initialized_) {return;}

  setPositionalServo(0.0f);    // Center position
  setContinuousServo(0.0f);    // Stopped
}

uint16_t ServoController::angleToPWM(float angle_rad)
{
  // Map angle from [MIN_SERVO_POS_RAD, MAX_SERVO_POS_RAD] to [SERVO_PWM_MIN_US, SERVO_PWM_MAX_US]
  float normalized = (angle_rad - MIN_SERVO_POS_RAD) / (MAX_SERVO_POS_RAD - MIN_SERVO_POS_RAD);
  normalized = std::clamp(normalized, 0.0f, 1.0f);

  return SERVO_PWM_MIN_US +
         static_cast<uint16_t>(normalized * (SERVO_PWM_MAX_US - SERVO_PWM_MIN_US));
}

uint16_t ServoController::velocityToPWM(float velocity_norm)
{
  // Map velocity from [-1, 1] to [SERVO_PWM_MIN_US, SERVO_PWM_MAX_US]
  float normalized = (velocity_norm + 1.0f) / 2.0f;    // Map [-1,1] to [0,1]
  normalized = std::clamp(normalized, 0.0f, 1.0f);

  return SERVO_PWM_MIN_US +
         static_cast<uint16_t>(normalized * (SERVO_PWM_MAX_US - SERVO_PWM_MIN_US));
}

// ESC Controller Implementation
ESCController::ESCController()
: initialized_(false)
{
  current_esc_norm_[0] = 0.0f;
  current_esc_norm_[1] = 0.0f;
}

bool ESCController::init()
{
  if (initialized_) {return true;}

  // Initialize ESC PWM pins
  const uint esc_pins[2] = {12, 13};    // ESC channels

  for (int i = 0; i < 2; i++) {
    gpio_set_function(esc_pins[i], GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(esc_pins[i]);
    pwm_set_wrap(slice_num, 20000);      // 20ms period for ESC PWM
    pwm_set_enabled(slice_num, true);
  }

  initialized_ = true;
  setNeutral();
  return true;
}

void ESCController::setESCVelocities(const float esc_norm[2])
{
  if (!initialized_) {return;}

  const uint esc_pins[2] = {12, 13};

  for (int i = 0; i < 2; i++) {
    float clamped = std::clamp(esc_norm[i], -1.0f, 1.0f);
    current_esc_norm_[i] = clamped;

    uint16_t pwm_us = velocityToPWM(clamped);

    uint slice_num = pwm_gpio_to_slice_num(esc_pins[i]);
    uint channel = pwm_gpio_to_channel(esc_pins[i]);
    pwm_set_chan_level(slice_num, channel, pwm_us);
  }
}

void ESCController::getESCVelocities(float esc_norm[2]) const
{
  esc_norm[0] = current_esc_norm_[0];
  esc_norm[1] = current_esc_norm_[1];
}

void ESCController::setNeutral()
{
  if (!initialized_) {return;}

  const float neutral_velocities[2] = {0.0f, 0.0f};
  setESCVelocities(neutral_velocities);
}

uint16_t ESCController::velocityToPWM(float velocity_norm)
{
  // Apply deadband
  if (std::abs(velocity_norm) < ESC_DEADBAND) {
    return ESC_PWM_NEUTRAL_US;
  }

  // Map velocity from [-1, 1] to [ESC_PWM_MIN_US, ESC_PWM_MAX_US]
  float normalized = (velocity_norm + 1.0f) / 2.0f;    // Map [-1,1] to [0,1]
  normalized = std::clamp(normalized, 0.0f, 1.0f);

  return ESC_PWM_MIN_US + static_cast<uint16_t>(normalized * (ESC_PWM_MAX_US - ESC_PWM_MIN_US));
}

// Actuator Controller Implementation
ActuatorController::ActuatorController()
: status_flags_(0) {}

bool ActuatorController::init()
{
  bool success = true;

  success &= motors_.init();
  success &= servos_.init();
  success &= escs_.init();

  if (success) {
    setAllNeutral();
  }

  return success;
}

void ActuatorController::applyCommand(const CommandFramePayload & command, uint16_t & flags_out)
{
  status_flags_ = 0;

  // Apply wheel velocities
  motors_.applyWheelVelocities(command.wheel_vel_rad_s);

  // Apply positional servo with clamping
  float clamped_servo_pos = clampAndFlag(
    command.servo_pos_rad,
    -1.57f, 1.57f,      // -90 to +90 degrees
    Flags::POSITIONAL_SERVO_LIMIT_HIT,
    status_flags_
  );
  servos_.setPositionalServo(clamped_servo_pos);

  // Apply continuous servo
  float clamped_servo_vel = std::clamp(command.servo_cont_vel_norm, -1.0f, 1.0f);
  servos_.setContinuousServo(clamped_servo_vel);

  // Apply ESCs with clamping
  float clamped_esc[2];
  clamped_esc[0] = std::clamp(command.esc_norm[0], -1.0f, 1.0f);
  clamped_esc[1] = std::clamp(command.esc_norm[1], -1.0f, 1.0f);
  escs_.setESCVelocities(clamped_esc);

  flags_out = status_flags_;
}

void ActuatorController::getCurrentState(StateFramePayload & state)
{
  // Placeholder encoder counts (would come from actual encoders)
  for (int i = 0; i < 4; i++) {
    state.encoder_counts[i] = 0;      // TODO: Read actual encoder values
  }

  // Get servo states
  state.servo_pos_rad = servos_.getPositionalServoAngle();
  state.servo_cont_vel_norm = servos_.getContinuousServoVelocity();

  // Get ESC states
  escs_.getESCVelocities(state.esc_norm);
}

void ActuatorController::setAllNeutral()
{
  motors_.setNeutral();
  servos_.setNeutral();
  escs_.setNeutral();
  status_flags_ = 0;
}

float ActuatorController::clampAndFlag(
  float value, float min_val, float max_val, uint16_t flag_bit,
  uint16_t & flags)
{
  if (value < min_val) {
    flags |= flag_bit;
    return min_val;
  } else if (value > max_val) {
    flags |= flag_bit;
    return max_val;
  }
  return value;
}

} // namespace mecabridge
