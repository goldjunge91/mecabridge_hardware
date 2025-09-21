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

#include "mecabridge_hardware/mecabridge_hardware.hpp"
#include "mecabridge_hardware/feature_config.hpp"

#include <gtest/gtest.h>

namespace mecabridge_hardware
{

class FeatureToggleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create a minimal hardware info for testing
    hardware_interface::HardwareInfo hw_info;
    hw_info.hardware_parameters["config_file"] = "test_config.yaml";

    // Set up a test config file content
    test_config_content_ =
      R"(
serial_port: "/dev/ttyUSB0"
state_publish_rate_hz: 50
wheel_radius: 0.0325
wheel_separation_x: 0.28
wheel_separation_y: 0.24
encoder_ticks_per_rev: 1000

wheels:
  front_left:
    joint_name: "front_left_wheel"
    encoder_index: 0
  front_right:
    joint_name: "front_right_wheel"
    encoder_index: 1
  rear_left:
    joint_name: "rear_left_wheel"
    encoder_index: 2
  rear_right:
    joint_name: "rear_right_wheel"
    encoder_index: 3

servos:
  positional:
    joint_name: "positional_servo"
    min_rad: -1.57
    max_rad: 1.57
  continuous:
    joint_name: "continuous_servo"
    max_velocity_rad_s: 5.0

escs:
  left:
    joint_name: "left_esc"
    esc_min_pwm: 1000
    esc_max_pwm: 2000
    esc_deadband: 0.05
  right:
    joint_name: "right_esc"
    esc_min_pwm: 1000
    esc_max_pwm: 2000
    esc_deadband: 0.05

features:
  enable_servos: true
  enable_escs: true
  enable_diagnostics: true
)";
  }

  std::string test_config_content_;
};

TEST_F(FeatureToggleTest, CompileTimeFeatureFlags)
{
  // Test that compile-time feature flags are correctly defined

#if MECABRIDGE_ENABLE_SERVOS
  EXPECT_TRUE(kEnableServos);
  EXPECT_EQ(kServoInterfaces, 2);
#else
  EXPECT_FALSE(kEnableServos);
  EXPECT_EQ(kServoInterfaces, 0);
#endif

#if MECABRIDGE_ENABLE_ESCS
  EXPECT_TRUE(kEnableESCs);
  EXPECT_EQ(kESCInterfaces, 2);
#else
  EXPECT_FALSE(kEnableESCs);
  EXPECT_EQ(kESCInterfaces, 0);
#endif

#if MECABRIDGE_ENABLE_DIAGNOSTICS
  EXPECT_TRUE(kEnableDiagnostics);
#else
  EXPECT_FALSE(kEnableDiagnostics);
#endif

  // Test interface counts
  EXPECT_EQ(kWheelInterfaces, 4);  // Always 4 wheels
  EXPECT_EQ(kTotalInterfaces, kWheelInterfaces + kServoInterfaces + kESCInterfaces);

  // Test index mappings
  EXPECT_EQ(kWheelStartIndex, 0);
  EXPECT_EQ(kServoStartIndex, kWheelStartIndex + kWheelInterfaces);
  EXPECT_EQ(kESCStartIndex, kServoStartIndex + kServoInterfaces);
}

TEST_F(FeatureToggleTest, InterfaceCountWithAllFeaturesEnabled)
{
  // This test assumes all features are enabled at compile time
#if MECABRIDGE_ENABLE_SERVOS && MECABRIDGE_ENABLE_ESCS
  MecaBridgeHardware hardware;

  // Create hardware info with test config
  hardware_interface::HardwareInfo hw_info;
  hw_info.hardware_parameters["config_file"] = "test_config.yaml";

  // Note: This test would need actual config file or mock config loading
  // For now, just test the feature flag constants

  EXPECT_EQ(kTotalInterfaces, 8);  // 4 wheels + 2 servos + 2 ESCs
#endif
}

TEST_F(FeatureToggleTest, InterfaceCountWithServosDisabled)
{
  // This test would need to be compiled with MECABRIDGE_ENABLE_SERVOS=0
#if !MECABRIDGE_ENABLE_SERVOS && MECABRIDGE_ENABLE_ESCS
  EXPECT_EQ(kTotalInterfaces, 6);  // 4 wheels + 0 servos + 2 ESCs
#endif
}

TEST_F(FeatureToggleTest, InterfaceCountWithESCsDisabled)
{
  // This test would need to be compiled with MECABRIDGE_ENABLE_ESCS=0
#if MECABRIDGE_ENABLE_SERVOS && !MECABRIDGE_ENABLE_ESCS
  EXPECT_EQ(kTotalInterfaces, 6);  // 4 wheels + 2 servos + 0 ESCs
#endif
}

TEST_F(FeatureToggleTest, InterfaceCountWithAllOptionalFeaturesDisabled)
{
  // This test would need to be compiled with both servo and ESC features disabled
#if !MECABRIDGE_ENABLE_SERVOS && !MECABRIDGE_ENABLE_ESCS
  EXPECT_EQ(kTotalInterfaces, 4);  // 4 wheels only
#endif
}

} // namespace mecabridge_hardware
