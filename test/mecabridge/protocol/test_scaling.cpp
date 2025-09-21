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
#include "mecabridge_utils/protocol/scaling.hpp"
#include <cmath>

using namespace mecabridge::protocol;

class ESCScalerTest : public ::testing::Test {};

TEST_F(ESCScalerTest, VelocityToPWMBasicMapping)
{
  uint16_t min_pwm = 1000;
  uint16_t max_pwm = 2000;
  float deadband = 0.05f;

  // Test neutral (zero velocity)
  uint16_t neutral_pwm = ESCScaler::velocityToPWM(0.0f, min_pwm, max_pwm, deadband);
  EXPECT_EQ(neutral_pwm, 1500);  // Midpoint

  // Test within deadband - should return neutral
  uint16_t near_zero = ESCScaler::velocityToPWM(0.04f, min_pwm, max_pwm, deadband);
  EXPECT_EQ(near_zero, 1500);

  // Test full forward
  uint16_t full_forward = ESCScaler::velocityToPWM(1.0f, min_pwm, max_pwm, deadband);
  EXPECT_EQ(full_forward, max_pwm);

  // Test full reverse
  uint16_t full_reverse = ESCScaler::velocityToPWM(-1.0f, min_pwm, max_pwm, deadband);
  EXPECT_EQ(full_reverse, min_pwm);
}

TEST_F(ESCScalerTest, VelocityToPWMClamping)
{
  uint16_t min_pwm = 1000;
  uint16_t max_pwm = 2000;

  // Test over-range clamping
  uint16_t over_max = ESCScaler::velocityToPWM(1.5f, min_pwm, max_pwm);
  EXPECT_EQ(over_max, max_pwm);

  uint16_t under_min = ESCScaler::velocityToPWM(-1.5f, min_pwm, max_pwm);
  EXPECT_EQ(under_min, min_pwm);
}

TEST_F(ESCScalerTest, PWMToVelocityRoundTrip)
{
  uint16_t min_pwm = 1000;
  uint16_t max_pwm = 2000;
  float deadband = 0.05f;

  float test_velocities[] = {-1.0f, -0.5f, -0.1f, 0.0f, 0.1f, 0.5f, 1.0f};

  for (float vel : test_velocities) {
    uint16_t pwm = ESCScaler::velocityToPWM(vel, min_pwm, max_pwm, deadband);
    float recovered_vel = ESCScaler::pwmToVelocity(pwm, min_pwm, max_pwm, deadband);

    // Allow some tolerance for floating point precision and deadband effects
    if (std::abs(vel) < deadband) {
      EXPECT_NEAR(recovered_vel, 0.0f, 0.01f) << "Velocity within deadband should map to zero";
    } else {
      EXPECT_NEAR(recovered_vel, vel, 0.1f) << "Round-trip conversion failed for velocity: " << vel;
    }
  }
}

TEST_F(ESCScalerTest, DeadbandBehavior)
{
  uint16_t min_pwm = 1000;
  uint16_t max_pwm = 2000;
  float deadband = 0.1f;  // 10% deadband

  // Test values within deadband
  float within_deadband[] = {-0.09f, -0.05f, 0.0f, 0.05f, 0.09f};

  for (float vel : within_deadband) {
    uint16_t pwm = ESCScaler::velocityToPWM(vel, min_pwm, max_pwm, deadband);
    EXPECT_EQ(pwm, 1500) << "Velocity " << vel << " within deadband should map to neutral PWM";
  }

  // Test values outside deadband
  uint16_t just_above = ESCScaler::velocityToPWM(0.11f, min_pwm, max_pwm, deadband);
  EXPECT_GT(just_above, 1500) << "Velocity just above deadband should map above neutral";

  uint16_t just_below = ESCScaler::velocityToPWM(-0.11f, min_pwm, max_pwm, deadband);
  EXPECT_LT(just_below, 1500) << "Velocity just below deadband should map below neutral";
}

class ServoScalerTest : public ::testing::Test {};

TEST_F(ServoScalerTest, ContinuousVelocityClamping)
{
  float max_vel = 5.0f;  // rad/s
  bool clamped = false;

  // Test within limits
  float within_limits = ServoScaler::clampContinuousVelocity(3.0f, max_vel, &clamped);
  EXPECT_FLOAT_EQ(within_limits, 3.0f);
  EXPECT_FALSE(clamped);

  // Test over positive limit
  float over_limit = ServoScaler::clampContinuousVelocity(6.0f, max_vel, &clamped);
  EXPECT_FLOAT_EQ(over_limit, max_vel);
  EXPECT_TRUE(clamped);

  // Test under negative limit
  clamped = false;
  float under_limit = ServoScaler::clampContinuousVelocity(-6.0f, max_vel, &clamped);
  EXPECT_FLOAT_EQ(under_limit, -max_vel);
  EXPECT_TRUE(clamped);
}

TEST_F(ServoScalerTest, VelocityNormalization)
{
  float max_vel = 10.0f;

  // Test full range
  EXPECT_FLOAT_EQ(ServoScaler::velocityToNormalized(10.0f, max_vel), 1.0f);
  EXPECT_FLOAT_EQ(ServoScaler::velocityToNormalized(-10.0f, max_vel), -1.0f);
  EXPECT_FLOAT_EQ(ServoScaler::velocityToNormalized(0.0f, max_vel), 0.0f);
  EXPECT_FLOAT_EQ(ServoScaler::velocityToNormalized(5.0f, max_vel), 0.5f);

  // Test over-range clamping
  EXPECT_FLOAT_EQ(ServoScaler::velocityToNormalized(15.0f, max_vel), 1.0f);
  EXPECT_FLOAT_EQ(ServoScaler::velocityToNormalized(-15.0f, max_vel), -1.0f);
}

TEST_F(ServoScalerTest, VelocityRoundTrip)
{
  float max_vel = 8.0f;
  float test_velocities[] = {-8.0f, -4.0f, -1.0f, 0.0f, 1.0f, 4.0f, 8.0f};

  for (float vel : test_velocities) {
    float normalized = ServoScaler::velocityToNormalized(vel, max_vel);
    float recovered = ServoScaler::normalizedToVelocity(normalized, max_vel);
    EXPECT_FLOAT_EQ(recovered, vel) << "Round-trip failed for velocity: " << vel;
  }
}

TEST_F(ServoScalerTest, PositionalAngleClamping)
{
  float min_angle = -1.57f;  // -90 degrees
  float max_angle = 1.57f;   // +90 degrees
  bool clamped = false;

  // Test within limits
  float within_limits = ServoScaler::clampPositionalAngle(0.5f, min_angle, max_angle, &clamped);
  EXPECT_FLOAT_EQ(within_limits, 0.5f);
  EXPECT_FALSE(clamped);

  // Test over limit
  float over_limit = ServoScaler::clampPositionalAngle(2.0f, min_angle, max_angle, &clamped);
  EXPECT_FLOAT_EQ(over_limit, max_angle);
  EXPECT_TRUE(clamped);

  // Test under limit
  clamped = false;
  float under_limit = ServoScaler::clampPositionalAngle(-2.0f, min_angle, max_angle, &clamped);
  EXPECT_FLOAT_EQ(under_limit, min_angle);
  EXPECT_TRUE(clamped);
}

TEST_F(ServoScalerTest, AngleToPWMMapping)
{
  float min_angle = -1.57f;
  float max_angle = 1.57f;
  uint16_t min_pwm = 1000;
  uint16_t max_pwm = 2000;

  // Test center position
  uint16_t center_pwm = ServoScaler::angleToPWM(0.0f, min_angle, max_angle, min_pwm, max_pwm);
  EXPECT_EQ(center_pwm, 1500);  // Midpoint

  // Test extremes
  uint16_t min_pwm_result = ServoScaler::angleToPWM(
    min_angle, min_angle, max_angle, min_pwm,
    max_pwm);
  EXPECT_EQ(min_pwm_result, min_pwm);

  uint16_t max_pwm_result = ServoScaler::angleToPWM(
    max_angle, min_angle, max_angle, min_pwm,
    max_pwm);
  EXPECT_EQ(max_pwm_result, max_pwm);
}

TEST_F(ServoScalerTest, AnglePWMRoundTrip)
{
  float min_angle = -3.14f;
  float max_angle = 3.14f;
  uint16_t min_pwm = 500;
  uint16_t max_pwm = 2500;

  float test_angles[] = {-3.14f, -1.57f, 0.0f, 1.57f, 3.14f};

  for (float angle : test_angles) {
    uint16_t pwm = ServoScaler::angleToPWM(angle, min_angle, max_angle, min_pwm, max_pwm);
    float recovered = ServoScaler::pwmToAngle(pwm, min_angle, max_angle, min_pwm, max_pwm);
    EXPECT_NEAR(recovered, angle, 0.01f) << "Round-trip failed for angle: " << angle;
  }
}
