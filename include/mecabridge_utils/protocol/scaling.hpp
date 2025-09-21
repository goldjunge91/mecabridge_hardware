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

#pragma once

#include <cstdint>
#include <algorithm>
#include <cmath>

namespace mecabridge
{
namespace protocol
{

/**
 * @brief Utility class for ESC PWM scaling and normalization
 */
class ESCScaler
{
public:
  /**
   * @brief Convert normalized velocity [-1,1] to PWM microseconds
   * @param velocity_norm Normalized velocity [-1.0, 1.0]
   * @param min_pwm_us Minimum PWM value in microseconds (e.g., 1000)
   * @param max_pwm_us Maximum PWM value in microseconds (e.g., 2000)
   * @param deadband Deadband around neutral [0.0, 1.0)
   * @return PWM value in microseconds
   */
  static uint16_t velocityToPWM(
    float velocity_norm,
    uint16_t min_pwm_us,
    uint16_t max_pwm_us,
    float deadband = 0.05f)
  {
    // Clamp input to valid range
    velocity_norm = std::clamp(velocity_norm, -1.0f, 1.0f);

    // Apply deadband - if within deadband, return neutral
    if (std::abs(velocity_norm) < deadband) {
      return (min_pwm_us + max_pwm_us) / 2;  // Neutral position
    }

    // Adjust for deadband - map remaining range to full PWM range
    float adjusted_velocity;
    if (velocity_norm > 0) {
      // Positive: map [deadband, 1.0] to [0.5, 1.0]
      adjusted_velocity = 0.5f + 0.5f * (velocity_norm - deadband) / (1.0f - deadband);
    } else {
      // Negative: map [-1.0, -deadband] to [0.0, 0.5]
      adjusted_velocity = 0.5f * (velocity_norm + 1.0f) / (1.0f - deadband);
    }

    // Map [0.0, 1.0] to [min_pwm_us, max_pwm_us]
    return static_cast<uint16_t>(min_pwm_us + adjusted_velocity * (max_pwm_us - min_pwm_us));
  }

  /**
   * @brief Convert PWM microseconds back to normalized velocity
   * @param pwm_us PWM value in microseconds
   * @param min_pwm_us Minimum PWM value in microseconds
   * @param max_pwm_us Maximum PWM value in microseconds
   * @param deadband Deadband around neutral
   * @return Normalized velocity [-1.0, 1.0]
   */
  static float pwmToVelocity(
    uint16_t pwm_us,
    uint16_t min_pwm_us,
    uint16_t max_pwm_us,
    float deadband = 0.05f)
  {
    // Clamp PWM to valid range
    pwm_us = std::clamp(pwm_us, min_pwm_us, max_pwm_us);

    uint16_t neutral_pwm = (min_pwm_us + max_pwm_us) / 2;
    uint16_t deadband_pwm = static_cast<uint16_t>(deadband * (max_pwm_us - min_pwm_us) / 2);

    // If within deadband around neutral, return 0
    if (std::abs(static_cast<int>(pwm_us) - static_cast<int>(neutral_pwm)) < deadband_pwm) {
      return 0.0f;
    }

    // Convert back to normalized range
    float normalized = static_cast<float>(pwm_us - min_pwm_us) / (max_pwm_us - min_pwm_us);

    // Adjust for deadband
    if (normalized > 0.5f) {
      // Above neutral: map [0.5 + deadband/2, 1.0] to [deadband, 1.0]
      float upper_start = 0.5f + deadband / 2.0f;
      return deadband + (1.0f - deadband) * (normalized - upper_start) / (1.0f - upper_start);
    } else {
      // Below neutral: map [0.0, 0.5 - deadband/2] to [-1.0, -deadband]
      float lower_end = 0.5f - deadband / 2.0f;
      return -1.0f + (1.0f - deadband) * normalized / lower_end;
    }
  }
};

/**
 * @brief Utility class for servo scaling and limit enforcement
 */
class ServoScaler
{
public:
  /**
   * @brief Clamp continuous servo velocity and apply scaling
   * @param velocity_rad_s Velocity in rad/s
   * @param max_velocity_rad_s Maximum allowed velocity
   * @param clamped_out Optional output flag indicating if clamping occurred
   * @return Clamped velocity in rad/s
   */
  static float clampContinuousVelocity(
    float velocity_rad_s,
    float max_velocity_rad_s,
    bool * clamped_out = nullptr)
  {
    bool was_clamped = false;
    float clamped_velocity = velocity_rad_s;

    if (std::abs(velocity_rad_s) > max_velocity_rad_s) {
      clamped_velocity = std::copysign(max_velocity_rad_s, velocity_rad_s);
      was_clamped = true;
    }

    if (clamped_out) {
      *clamped_out = was_clamped;
    }

    return clamped_velocity;
  }

  /**
   * @brief Convert continuous servo velocity to normalized [-1,1] range
   * @param velocity_rad_s Velocity in rad/s
   * @param max_velocity_rad_s Maximum velocity for normalization
   * @return Normalized velocity [-1.0, 1.0]
   */
  static float velocityToNormalized(float velocity_rad_s, float max_velocity_rad_s)
  {
    if (max_velocity_rad_s <= 0.0f) {
      return 0.0f;
    }

    float normalized = velocity_rad_s / max_velocity_rad_s;
    return std::clamp(normalized, -1.0f, 1.0f);
  }

  /**
   * @brief Convert normalized velocity back to rad/s
   * @param velocity_norm Normalized velocity [-1.0, 1.0]
   * @param max_velocity_rad_s Maximum velocity for scaling
   * @return Velocity in rad/s
   */
  static float normalizedToVelocity(float velocity_norm, float max_velocity_rad_s)
  {
    velocity_norm = std::clamp(velocity_norm, -1.0f, 1.0f);
    return velocity_norm * max_velocity_rad_s;
  }

  /**
   * @brief Clamp positional servo angle to valid range
   * @param angle_rad Angle in radians
   * @param min_rad Minimum allowed angle
   * @param max_rad Maximum allowed angle
   * @param clamped_out Optional output flag indicating if clamping occurred
   * @return Clamped angle in radians
   */
  static float clampPositionalAngle(
    float angle_rad,
    float min_rad,
    float max_rad,
    bool * clamped_out = nullptr)
  {
    bool was_clamped = false;
    float clamped_angle = angle_rad;

    if (angle_rad < min_rad) {
      clamped_angle = min_rad;
      was_clamped = true;
    } else if (angle_rad > max_rad) {
      clamped_angle = max_rad;
      was_clamped = true;
    }

    if (clamped_out) {
      *clamped_out = was_clamped;
    }

    return clamped_angle;
  }

  /**
   * @brief Convert servo angle to PWM microseconds
   * @param angle_rad Angle in radians
   * @param min_rad Minimum servo angle
   * @param max_rad Maximum servo angle
   * @param min_pwm_us Minimum PWM value (e.g., 1000us)
   * @param max_pwm_us Maximum PWM value (e.g., 2000us)
   * @return PWM value in microseconds
   */
  static uint16_t angleToPWM(
    float angle_rad,
    float min_rad,
    float max_rad,
    uint16_t min_pwm_us,
    uint16_t max_pwm_us)
  {
    // Clamp angle to valid range
    angle_rad = std::clamp(angle_rad, min_rad, max_rad);

    // Normalize angle to [0, 1]
    float normalized = (angle_rad - min_rad) / (max_rad - min_rad);

    // Map to PWM range
    return static_cast<uint16_t>(min_pwm_us + normalized * (max_pwm_us - min_pwm_us));
  }

  /**
   * @brief Convert PWM microseconds back to servo angle
   * @param pwm_us PWM value in microseconds
   * @param min_rad Minimum servo angle
   * @param max_rad Maximum servo angle
   * @param min_pwm_us Minimum PWM value
   * @param max_pwm_us Maximum PWM value
   * @return Angle in radians
   */
  static float pwmToAngle(
    uint16_t pwm_us,
    float min_rad,
    float max_rad,
    uint16_t min_pwm_us,
    uint16_t max_pwm_us)
  {
    // Clamp PWM to valid range
    pwm_us = std::clamp(pwm_us, min_pwm_us, max_pwm_us);

    // Normalize PWM to [0, 1]
    float normalized = static_cast<float>(pwm_us - min_pwm_us) / (max_pwm_us - min_pwm_us);

    // Map to angle range
    return min_rad + normalized * (max_rad - min_rad);
  }
};

} // namespace protocol
} // namespace mecabridge
