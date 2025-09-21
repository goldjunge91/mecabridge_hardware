#ifndef DRIVE_ARDUINO__FIRMWARE__MECABRIDGE_PICO__ACTUATORS_HPP_
#define DRIVE_ARDUINO__FIRMWARE__MECABRIDGE_PICO__ACTUATORS_HPP_


#pragma once

#include "frame_parser.hpp"

#include <cstdint>

namespace mecabridge
{

// Motor controller for wheel velocities
class MotorController
{
public:
  MotorController();

  // Initialize PWM and GPIO for motors
  bool init();

  // Apply wheel velocities (rad/s) - placeholder implementation
  void applyWheelVelocities(const float wheel_vel_rad_s[4]);

  // Set motors to safe neutral state
  void setNeutral();

private:
  bool initialized_;

  // Convert rad/s to PWM duty cycle (placeholder)
  uint16_t velocityToPWM(float velocity_rad_s);

  // Set PWM for a specific motor (0-3: FL, FR, RL, RR)
  void setMotorPWM(uint8_t motor_index, uint16_t pwm_value, bool direction);
};

// Servo controller
class ServoController
{
public:
  ServoController();

  // Initialize servo PWM
  bool init();

  // Set positional servo angle (rad)
  void setPositionalServo(float angle_rad);

  // Set continuous servo velocity (normalized -1..1)
  void setContinuousServo(float velocity_norm);

  // Get current positional servo angle (placeholder - returns last set value)
  float getPositionalServoAngle() const {return current_pos_rad_;}

  // Get current continuous servo velocity (placeholder - returns last set value)
  float getContinuousServoVelocity() const {return current_vel_norm_;}

  // Set servos to neutral/safe state
  void setNeutral();

private:
  bool initialized_;
  float current_pos_rad_;
  float current_vel_norm_;

  // Configuration from config.h
  static constexpr float MIN_SERVO_POS_RAD = -1.57f;    // -90 degrees
  static constexpr float MAX_SERVO_POS_RAD = 1.57f;     // +90 degrees
  static constexpr uint16_t SERVO_PWM_MIN_US = 1000;    // 1ms
  static constexpr uint16_t SERVO_PWM_MAX_US = 2000;    // 2ms
  static constexpr uint16_t SERVO_PWM_NEUTRAL_US = 1500;   // 1.5ms

  // Convert angle to PWM microseconds
  uint16_t angleToPWM(float angle_rad);

  // Convert normalized velocity to PWM microseconds
  uint16_t velocityToPWM(float velocity_norm);
};

// ESC (Electronic Speed Controller) controller
class ESCController
{
public:
  ESCController();

  // Initialize ESC PWM
  bool init();

  // Set ESC velocities (normalized -1..1)
  void setESCVelocities(const float esc_norm[2]);

  // Get current ESC velocities (placeholder - returns last set values)
  void getESCVelocities(float esc_norm[2]) const;

  // Set ESCs to neutral state
  void setNeutral();

private:
  bool initialized_;
  float current_esc_norm_[2];

  // ESC configuration
  static constexpr uint16_t ESC_PWM_MIN_US = 1000;     // Minimum PWM (full reverse)
  static constexpr uint16_t ESC_PWM_MAX_US = 2000;     // Maximum PWM (full forward)
  static constexpr uint16_t ESC_PWM_NEUTRAL_US = 1500;   // Neutral (stop)
  static constexpr float ESC_DEADBAND = 0.05f;         // Deadband around neutral

  // Convert normalized velocity to PWM microseconds
  uint16_t velocityToPWM(float velocity_norm);
};

// Main actuator controller that coordinates all actuators
class ActuatorController
{
public:
  ActuatorController();

  // Initialize all actuators
  bool init();

  // Apply command frame to all actuators
  void applyCommand(const CommandFramePayload & command, uint16_t & flags_out);

  // Get current state for state frame
  void getCurrentState(StateFramePayload & state);

  // Set all actuators to safe neutral state
  void setAllNeutral();

  // Check if any actuator limits were hit and update flags
  uint16_t getStatusFlags() const {return status_flags_;}

private:
  MotorController motors_;
  ServoController servos_;
  ESCController escs_;

  uint16_t status_flags_;

  // Clamp value and set flag if clamped
  float clampAndFlag(
    float value, float min_val, float max_val, uint16_t flag_bit,
    uint16_t & flags);
};

} // namespace mecabridge
#endif  // DRIVE_ARDUINO__FIRMWARE__MECABRIDGE_PICO__ACTUATORS_HPP_
