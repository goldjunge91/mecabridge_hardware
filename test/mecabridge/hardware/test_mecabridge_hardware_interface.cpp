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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <hardware_interface/resource_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcpputils/filesystem_helper.hpp>
#include <sstream>
#include <string>
#include <vector>

TEST(TestMecabridgeHardwareInterface, load_urdf_and_check_interfaces) {
  const auto share_dir = ament_index_cpp::get_package_share_directory("mecabridge_hardware");
  const auto config_path = rcpputils::fs::path(share_dir) / "test" / "mecabridge" / "hardware" /
    "test_mecabridge_hardware_interface.yaml";
  ASSERT_TRUE(rcpputils::fs::exists(config_path))
    << "Test config missing: " << config_path.string();
  const auto config_file = config_path.string();

  std::ostringstream urdf;
  urdf <<
    R"(<robot name="TestRobot">
  <ros2_control name="MecaBridgeSystem" type="system">
    <hardware>
      <plugin>mecabridge_hardware/MecaBridgeHardware</plugin>
      <param name="config_file">)"
       << config_file <<
    R"(</param>
    </hardware>
    <joint name="front_left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
    <joint name="front_right_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
    <joint name="rear_left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
    <joint name="rear_right_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
</robot>)";
  const std::string urdf_string = urdf.str();

  rclcpp::init(0, nullptr);

  try {
    hardware_interface::ResourceManager rm(urdf_string);

    const auto & command_interfaces = rm.command_interface_keys();
    const auto & state_interfaces = rm.state_interface_keys();

    ASSERT_EQ(command_interfaces.size(), 8u);
    ASSERT_EQ(state_interfaces.size(), 12u);

    using ::testing::ElementsAre;

    EXPECT_THAT(
      command_interfaces,
      ElementsAre(
        "front_left_wheel_joint/velocity", "front_right_wheel_joint/velocity",
        "rear_left_wheel_joint/velocity", "rear_right_wheel_joint/velocity",
        "pan_servo_joint/position", "rot_servo_joint/velocity",
        "esc_left_joint/velocity", "esc_right_joint/velocity"));

    EXPECT_THAT(
      state_interfaces,
      ElementsAre(
        "front_left_wheel_joint/velocity", "front_left_wheel_joint/position",
        "front_right_wheel_joint/velocity", "front_right_wheel_joint/position",
        "rear_left_wheel_joint/velocity", "rear_left_wheel_joint/position",
        "rear_right_wheel_joint/velocity", "rear_right_wheel_joint/position",
        "pan_servo_joint/position", "rot_servo_joint/velocity",
        "esc_left_joint/velocity", "esc_right_joint/velocity"));
  } catch (const std::exception & e) {
    FAIL() << "Exception thrown: " << e.what();
  }

  rclcpp::shutdown();
}
