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

#include "mecabridge_utils/protocol/frame.hpp"
using namespace mecabridge::protocol; // use SafetyFlags, CommandFramePayload, encodeCommand, ErrorCode

#include <memory>

#include <gtest/gtest.h>
#include <cmath>


/**
 * @brief Integration test for ESC and servo limit clamping
 *
 * This test verifies that over-limit commands are properly clamped
 * and appropriate flag bits are set when limits are exceeded.
 */
class ESCServoLimitClampTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Setup test limits
    servo_pos_min_rad_ = -1.57;  // -90 degrees
    servo_pos_max_rad_ = 1.57;   // +90 degrees
    servo_cont_max_vel_rad_s_ = 3.14;  // Max velocity for continuous servo
    esc_min_norm_ = -1.0;
    esc_max_norm_ = 1.0;
  }

  /**
   * @brief Apply servo position limits and return flags
   */
  uint16_t clampServoPosition(float & value)
  {
    uint16_t flags = 0;

    if (value > servo_pos_max_rad_) {
      value = servo_pos_max_rad_;
      flags |= static_cast<uint16_t>(SafetyFlags::SERVO_POSITION_LIMIT);
    } else if (value < servo_pos_min_rad_) {
      value = servo_pos_min_rad_;
      flags |= static_cast<uint16_t>(SafetyFlags::SERVO_POSITION_LIMIT);
    }

    return flags;
  }

  /**
   * @brief Apply servo velocity limits and return flags
   */
  uint16_t clampServoVelocity(float & value)
  {
    uint16_t flags = 0;

    if (std::abs(value) > servo_cont_max_vel_rad_s_) {
      value = (value > 0) ? servo_cont_max_vel_rad_s_ : -servo_cont_max_vel_rad_s_;
      flags |= static_cast<uint16_t>(SafetyFlags::SERVO_VELOCITY_LIMIT);
    }

    return flags;
  }

  /**
   * @brief Apply ESC limits and return flags
   */
  uint16_t clampESC(float & value)
  {
    uint16_t flags = 0;

    if (value > esc_max_norm_) {
      value = esc_max_norm_;
      flags |= static_cast<uint16_t>(SafetyFlags::ESC_LIMIT);
    } else if (value < esc_min_norm_) {
      value = esc_min_norm_;
      flags |= static_cast<uint16_t>(SafetyFlags::ESC_LIMIT);
    }

    return flags;
  }

  float servo_pos_min_rad_;
  float servo_pos_max_rad_;
  float servo_cont_max_vel_rad_s_;
  float esc_min_norm_;
  float esc_max_norm_;
};

TEST_F(ESCServoLimitClampTest, ServoPositionOverLimitPositive) {
  float test_value = 2.0f;  // Above max limit of 1.57
  uint16_t flags = clampServoPosition(test_value);

  EXPECT_FLOAT_EQ(test_value, servo_pos_max_rad_)
    << "Value should be clamped to max limit";
  EXPECT_NE(flags & static_cast<uint16_t>(SafetyFlags::SERVO_POSITION_LIMIT), 0)
    << "SERVO_POSITION_LIMIT flag should be set";
}

TEST_F(ESCServoLimitClampTest, ServoPositionOverLimitNegative) {
  float test_value = -2.0f;  // Below min limit of -1.57
  uint16_t flags = clampServoPosition(test_value);

  EXPECT_FLOAT_EQ(test_value, servo_pos_min_rad_)
    << "Value should be clamped to min limit";
  EXPECT_NE(flags & static_cast<uint16_t>(SafetyFlags::SERVO_POSITION_LIMIT), 0)
    << "SERVO_POSITION_LIMIT flag should be set";
}

TEST_F(ESCServoLimitClampTest, ServoPositionWithinLimits) {
  float test_value = 1.0f;  // Within limits [-1.57, 1.57]
  uint16_t flags = clampServoPosition(test_value);

  EXPECT_FLOAT_EQ(test_value, 1.0f)
    << "Value should remain unchanged when within limits";
  EXPECT_EQ(flags, 0) << "No flags should be set for values within limits";
}

TEST_F(ESCServoLimitClampTest, ServoPositionAtExactLimits) {
  // Test at positive limit
  float test_value_pos = servo_pos_max_rad_;
  uint16_t flags_pos = clampServoPosition(test_value_pos);
  EXPECT_FLOAT_EQ(test_value_pos, servo_pos_max_rad_);
  EXPECT_EQ(flags_pos, 0) << "No flags for value exactly at limit";

  // Test at negative limit
  float test_value_neg = servo_pos_min_rad_;
  uint16_t flags_neg = clampServoPosition(test_value_neg);
  EXPECT_FLOAT_EQ(test_value_neg, servo_pos_min_rad_);
  EXPECT_EQ(flags_neg, 0) << "No flags for value exactly at limit";
}

TEST_F(ESCServoLimitClampTest, ServoVelocityOverLimitPositive) {
  float test_value = 5.0f;  // Above max velocity of 3.14
  uint16_t flags = clampServoVelocity(test_value);

  EXPECT_FLOAT_EQ(test_value, servo_cont_max_vel_rad_s_)
    << "Velocity should be clamped to max limit";
  EXPECT_NE(flags & static_cast<uint16_t>(SafetyFlags::SERVO_VELOCITY_LIMIT), 0)
    << "SERVO_VELOCITY_LIMIT flag should be set";
}

TEST_F(ESCServoLimitClampTest, ServoVelocityOverLimitNegative) {
  float test_value = -5.0f;  // Below min velocity of -3.14
  uint16_t flags = clampServoVelocity(test_value);

  EXPECT_FLOAT_EQ(test_value, -servo_cont_max_vel_rad_s_)
    << "Velocity should be clamped to min limit";
  EXPECT_NE(flags & static_cast<uint16_t>(SafetyFlags::SERVO_VELOCITY_LIMIT), 0)
    << "SERVO_VELOCITY_LIMIT flag should be set";
}

TEST_F(ESCServoLimitClampTest, ServoVelocityWithinLimits) {
  float test_value = 2.0f;  // Within velocity limits
  uint16_t flags = clampServoVelocity(test_value);

  EXPECT_FLOAT_EQ(test_value, 2.0f)
    << "Velocity should remain unchanged when within limits";
  EXPECT_EQ(flags, 0) << "No flags should be set for velocities within limits";
}

TEST_F(ESCServoLimitClampTest, ESCOverLimitPositive) {
  float test_value = 1.5f;  // Above max ESC value of 1.0
  uint16_t flags = clampESC(test_value);

  EXPECT_FLOAT_EQ(test_value, esc_max_norm_)
    << "ESC value should be clamped to max limit";
  EXPECT_NE(flags & static_cast<uint16_t>(SafetyFlags::ESC_LIMIT), 0)
    << "ESC_LIMIT flag should be set";
}

TEST_F(ESCServoLimitClampTest, ESCOverLimitNegative) {
  float test_value = -1.5f;  // Below min ESC value of -1.0
  uint16_t flags = clampESC(test_value);

  EXPECT_FLOAT_EQ(test_value, esc_min_norm_)
    << "ESC value should be clamped to min limit";
  EXPECT_NE(flags & static_cast<uint16_t>(SafetyFlags::ESC_LIMIT), 0)
    << "ESC_LIMIT flag should be set";
}

TEST_F(ESCServoLimitClampTest, ESCWithinLimits) {
  float test_value = 0.8f;  // Within ESC limits [-1.0, 1.0]
  uint16_t flags = clampESC(test_value);

  EXPECT_FLOAT_EQ(test_value, 0.8f)
    << "ESC value should remain unchanged when within limits";
  EXPECT_EQ(flags, 0) << "No flags should be set for values within limits";
}

TEST_F(ESCServoLimitClampTest, ESCAtExactLimits) {
  // Test at positive limit
  float test_value_pos = esc_max_norm_;
  uint16_t flags_pos = clampESC(test_value_pos);
  EXPECT_FLOAT_EQ(test_value_pos, esc_max_norm_);
  EXPECT_EQ(flags_pos, 0) << "No flags for ESC value exactly at limit";

  // Test at negative limit
  float test_value_neg = esc_min_norm_;
  uint16_t flags_neg = clampESC(test_value_neg);
  EXPECT_FLOAT_EQ(test_value_neg, esc_min_norm_);
  EXPECT_EQ(flags_neg, 0) << "No flags for ESC value exactly at limit";
}

TEST_F(ESCServoLimitClampTest, MultipleLimitsExceeded) {
  // Test case where multiple limits are exceeded simultaneously

  float servo_pos = 3.0f;      // Over position limit
  float servo_vel = -10.0f;    // Over velocity limit
  float esc1 = 2.0f;           // Over ESC limit
  float esc2 = -2.0f;          // Over ESC limit

  uint16_t flags = 0;
  flags |= clampServoPosition(servo_pos);
  flags |= clampServoVelocity(servo_vel);
  flags |= clampESC(esc1);
  flags |= clampESC(esc2);

  // Verify all values are clamped
  EXPECT_FLOAT_EQ(servo_pos, servo_pos_max_rad_);
  EXPECT_FLOAT_EQ(servo_vel, -servo_cont_max_vel_rad_s_);
  EXPECT_FLOAT_EQ(esc1, esc_max_norm_);
  EXPECT_FLOAT_EQ(esc2, esc_min_norm_);

  // Verify all relevant flags are set
  EXPECT_NE(flags & static_cast<uint16_t>(SafetyFlags::SERVO_POSITION_LIMIT), 0);
  EXPECT_NE(flags & static_cast<uint16_t>(SafetyFlags::SERVO_VELOCITY_LIMIT), 0);
  EXPECT_NE(flags & static_cast<uint16_t>(SafetyFlags::ESC_LIMIT), 0);
}

TEST_F(ESCServoLimitClampTest, CommandFrameClampingIntegration) {
  // Test with actual command frame structure
  CommandFramePayload cmd = {};

  // Set over-limit values
  cmd.servo_pos_rad = 2.5f;        // Over position limit
  cmd.servo_cont_vel_norm = -5.0f; // Over velocity limit
  cmd.esc_norm[0] = 1.8f;          // Over ESC limit
  cmd.esc_norm[1] = -1.3f;         // Over ESC limit
  cmd.seq = 42;

  // Apply clamping (simulating what hardware interface should do)
  uint16_t safety_flags = 0;

  // Copy values to avoid packed struct binding issues
  float servo_pos = cmd.servo_pos_rad;
  float servo_vel = cmd.servo_cont_vel_norm;
  float esc0 = cmd.esc_norm[0];
  float esc1 = cmd.esc_norm[1];

  safety_flags |= clampServoPosition(servo_pos);
  safety_flags |= clampServoVelocity(servo_vel);
  safety_flags |= clampESC(esc0);
  safety_flags |= clampESC(esc1);

  // Copy back the clamped values
  cmd.servo_pos_rad = servo_pos;
  cmd.servo_cont_vel_norm = servo_vel;
  cmd.esc_norm[0] = esc0;
  cmd.esc_norm[1] = esc1;

  // Verify clamped values
  EXPECT_FLOAT_EQ(cmd.servo_pos_rad, servo_pos_max_rad_);
  EXPECT_FLOAT_EQ(cmd.servo_cont_vel_norm, -servo_cont_max_vel_rad_s_);
  EXPECT_FLOAT_EQ(cmd.esc_norm[0], esc_max_norm_);
  EXPECT_FLOAT_EQ(cmd.esc_norm[1], esc_min_norm_);

  // Verify safety flags are appropriate
  EXPECT_NE(safety_flags, 0) << "Safety flags should be set when limits exceeded";

  // Test frame encoding with clamped values
  uint8_t buffer[64];
  size_t encoded_bytes = 0;
  auto result = encodeCommand(cmd, buffer, sizeof(buffer), encoded_bytes);
  EXPECT_EQ(result, ErrorCode::OK) << "Should successfully encode clamped command";
  EXPECT_GT(encoded_bytes, 0) << "Should produce encoded bytes";
}

TEST_F(ESCServoLimitClampTest, BoundaryValueTesting) {
  // Test values very close to limits
  float epsilon = 1e-6f;

  // Just under positive limit
  float servo_pos = servo_pos_max_rad_ - epsilon;
  uint16_t flags = clampServoPosition(servo_pos);
  EXPECT_FLOAT_EQ(servo_pos, servo_pos_max_rad_ - epsilon);
  EXPECT_EQ(flags, 0) << "Just under limit should not trigger flag";

  // Just over positive limit
  servo_pos = servo_pos_max_rad_ + epsilon;
  flags = clampServoPosition(servo_pos);
  EXPECT_FLOAT_EQ(servo_pos, servo_pos_max_rad_);
  EXPECT_NE(flags & static_cast<uint16_t>(SafetyFlags::SERVO_POSITION_LIMIT), 0)
    << "Just over limit should trigger flag";
}

TEST_F(ESCServoLimitClampTest, ZeroAndNearZeroValues) {
  // Test zero values (should always be within limits)
  float zero_value = 0.0f;

  uint16_t flags = 0;
  flags |= clampServoPosition(zero_value);
  flags |= clampServoVelocity(zero_value);
  flags |= clampESC(zero_value);

  EXPECT_FLOAT_EQ(zero_value, 0.0f) << "Zero should remain zero";
  EXPECT_EQ(flags, 0) << "Zero should not trigger any limits";

  // Test very small values
  float small_positive = 1e-6f;
  float small_negative = -1e-6f;

  uint16_t flags_pos = 0;
  flags_pos |= clampServoPosition(small_positive);
  flags_pos |= clampServoVelocity(small_positive);
  flags_pos |= clampESC(small_positive);

  uint16_t flags_neg = 0;
  flags_neg |= clampServoPosition(small_negative);
  flags_neg |= clampServoVelocity(small_negative);
  flags_neg |= clampESC(small_negative);

  EXPECT_EQ(flags_pos, 0) << "Small positive values should not trigger limits";
  EXPECT_EQ(flags_neg, 0) << "Small negative values should not trigger limits";
}
