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

#include <chrono>
#include <vector>
#include <memory>
#include <set>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>


/**
 * @brief Integration test node for MecaBridge hardware interface
 *
 * This node tests the full command flow:
 * 1. Publishes cmd_vel commands to mecanum drive controller
 * 2. Publishes servo position commands
 * 3. Publishes ESC velocity commands
 * 4. Monitors joint_states for updates
 * 5. Verifies commands are flowing through the system
 */
class IntegrationTestNode : public rclcpp::Node
{
public:
  IntegrationTestNode()
  : Node("integration_test_node"), test_passed_(false)
  {
    // Parameters
    declare_parameter("test_duration_seconds", 10.0);
    declare_parameter("expect_responses", true);

    test_duration_ = get_parameter("test_duration_seconds").as_double();
    expect_responses_ = get_parameter("expect_responses").as_bool();

    // Publishers
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/mecanum_drive_controller/cmd_vel",
      10);
    servo_pos_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/servo_position_controller/commands",
      10);
    servo_vel_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/servo_velocity_controller/commands",
      10);
    esc_vel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/esc_velocity_controller/commands", 10);

    // Subscribers
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&IntegrationTestNode::jointStatesCallback, this, std::placeholders::_1));

    // Test timer
    using namespace std::chrono_literals; // enable 100ms literal
    test_timer_ =
      create_wall_timer(100ms, std::bind(&IntegrationTestNode::testTimerCallback, this));

    // Shutdown timer
    shutdown_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(
          test_duration_)),
      std::bind(&IntegrationTestNode::shutdownCallback, this));

    start_time_ = now();
    last_joint_state_time_ = rclcpp::Time(0);

    RCLCPP_INFO(
      get_logger(), "Integration test node started, test duration: %.1f seconds", test_duration_);
  }

private:
  void testTimerCallback()
  {
    auto elapsed = (now() - start_time_).seconds();

    // Publish test commands
    publishTestCommands(elapsed);

    // Check for test completion
    if (elapsed > test_duration_) {
      evaluateTestResults();
      rclcpp::shutdown();
    }
  }

  void publishTestCommands(double elapsed)
  {
    // Vary commands over time to test different scenarios
    double phase = elapsed * 0.5; // Slow oscillation

    // Mecanum drive commands
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = 0.2 * sin(phase);
    twist.linear.y = 0.1 * cos(phase);
    twist.angular.z = 0.1 * sin(phase * 2.0);
    cmd_vel_pub_->publish(twist);

    // Servo position command
    auto servo_pos = std_msgs::msg::Float64();
    servo_pos.data = 0.5 * sin(phase * 1.5);
    servo_pos_pub_->publish(servo_pos);

    // Servo velocity command
    auto servo_vel = std_msgs::msg::Float64();
    servo_vel.data = 0.3 * cos(phase * 0.8);
    servo_vel_pub_->publish(servo_vel);

    // ESC velocity commands
    auto esc_vel = std_msgs::msg::Float64MultiArray();
    esc_vel.data = {0.2 * sin(phase), -0.2 * cos(phase)};
    esc_vel_pub_->publish(esc_vel);

    commands_sent_++;
  }

  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_joint_state_time_ = now();
    joint_states_received_++;

    // Store latest joint states for verification
    latest_joint_names_ = msg->name;
    latest_positions_ = msg->position;
    latest_velocities_ = msg->velocity;

    // Verify expected joints are present
    checkExpectedJoints(*msg);
  }

  void checkExpectedJoints(const sensor_msgs::msg::JointState & msg)
  {
    std::vector<std::string> expected_joints = {
      "front_left_wheel", "front_right_wheel",
      "rear_left_wheel", "rear_right_wheel",
      "servo_pos", "servo_cont",
      "esc_left", "esc_right"
    };

    for (const auto & expected : expected_joints) {
      auto it = std::find(msg.name.begin(), msg.name.end(), expected);
      if (it != msg.name.end()) {
        found_joints_.insert(expected);
      }
    }
  }

  void evaluateTestResults()
  {
    RCLCPP_INFO(get_logger(), "Test completed. Evaluating results...");
    RCLCPP_INFO(get_logger(), "Commands sent: %d", commands_sent_);
    RCLCPP_INFO(get_logger(), "Joint states received: %d", joint_states_received_);
    RCLCPP_INFO(get_logger(), "Found joints: %zu", found_joints_.size());

    // Print found joints
    for (const auto & joint : found_joints_) {
      RCLCPP_INFO(get_logger(), "  - %s", joint.c_str());
    }

    // Evaluate test criteria
    bool commands_sent_ok = commands_sent_ > 0;
    bool joint_states_ok = expect_responses_ ? (joint_states_received_ > 0) : true;
    bool joints_found_ok = found_joints_.size() >= 4; // At least wheel joints
    bool timing_ok = !expect_responses_ ||
      (last_joint_state_time_ != rclcpp::Time(0) &&
      (now() - last_joint_state_time_).seconds() < 1.0);

    test_passed_ = commands_sent_ok && joint_states_ok && joints_found_ok && timing_ok;

    if (test_passed_) {
      RCLCPP_INFO(get_logger(), "✅ Integration test PASSED");
    } else {
      RCLCPP_ERROR(get_logger(), "❌ Integration test FAILED");
      RCLCPP_ERROR(get_logger(), "  Commands sent OK: %s", commands_sent_ok ? "✅" : "❌");
      RCLCPP_ERROR(get_logger(), "  Joint states OK: %s", joint_states_ok ? "✅" : "❌");
      RCLCPP_ERROR(get_logger(), "  Joints found OK: %s", joints_found_ok ? "✅" : "❌");
      RCLCPP_ERROR(get_logger(), "  Timing OK: %s", timing_ok ? "✅" : "❌");
    }
  }

  void shutdownCallback()
  {
    RCLCPP_INFO(get_logger(), "Test duration reached, shutting down...");
    evaluateTestResults();
    rclcpp::shutdown();
  }

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_pos_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr esc_vel_pub_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr test_timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;

  // Test state
  rclcpp::Time start_time_;
  rclcpp::Time last_joint_state_time_;
  double test_duration_;
  bool expect_responses_;
  bool test_passed_;

  int commands_sent_ = 0;
  int joint_states_received_ = 0;
  std::set<std::string> found_joints_;

  // Latest joint state data
  std::vector<std::string> latest_joint_names_;
  std::vector<double> latest_positions_;
  std::vector<double> latest_velocities_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<IntegrationTestNode>();

  try {
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in integration test: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}
