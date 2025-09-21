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

#include "mecabridge_utils/serial/serial_backend.hpp"
#include "mecabridge_utils/protocol/frame.hpp"
using mecabridge::serial::LoopbackBackend;
using mecabridge::serial::SerialOptions;
using namespace mecabridge::protocol; // bring protocol symbols
using namespace std::chrono_literals;

#include <memory>
#include <chrono>
#include <thread>

#include <gtest/gtest.h>


class SerialLoopbackTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create loopback backend instance
    loopback_backend_ = std::make_shared<LoopbackBackend>();

    // Initialize with test configuration
    setupTestConfig();
  }

  void setupTestConfig()
  {
    // Configure the loopback backend for testing
    // This sets up the joint names and frame processing
  }

  std::shared_ptr<LoopbackBackend> loopback_backend_;
};

TEST_F(SerialLoopbackTest, BasicCommandEcho) {
  // Note: This test would need proper hardware interface initialization
  // which requires ROS 2 parameter loading. For now, we test the loopback
  // backend directly.

  ASSERT_TRUE(loopback_backend_->open(SerialOptions{}));

  // Create a test command frame
  CommandFramePayload cmd_payload = {};
  cmd_payload.wheel_vel_rad_s[0] = 1.0f;  // Left wheel
  cmd_payload.wheel_vel_rad_s[1] = -1.0f; // Right wheel
  cmd_payload.wheel_vel_rad_s[2] = 0.5f;  // Rear left
  cmd_payload.wheel_vel_rad_s[3] = -0.5f; // Rear right
  cmd_payload.servo_pos_rad = 0.25f;
  cmd_payload.servo_cont_vel_norm = 0.1f;
  cmd_payload.esc_norm[0] = 0.3f;
  cmd_payload.esc_norm[1] = -0.2f;
  cmd_payload.seq = 42;

  // Encode command frame
  uint8_t cmd_buffer[128];
  size_t cmd_bytes = 0;
  auto encode_result = encodeCommand(cmd_payload, cmd_buffer, sizeof(cmd_buffer), cmd_bytes);
  ASSERT_EQ(encode_result, ErrorCode::OK);

  // Write command frame to loopback backend
  int written = loopback_backend_->write(cmd_buffer, cmd_bytes);
  EXPECT_EQ(written, static_cast<int>(cmd_bytes));

  // Read the echoed state frame
  uint8_t read_buffer[128];
  int bytes_read = loopback_backend_->read(read_buffer, sizeof(read_buffer));
  EXPECT_GT(bytes_read, 0);

  // Parse the state frame
  ByteSpan input(read_buffer, bytes_read);
  ParsedFrame parsed_frame;
  auto parse_result = tryParseFrame(input, parsed_frame);
  EXPECT_EQ(parse_result, ParseResult::SUCCESS);
  EXPECT_EQ(parsed_frame.frame_id, FrameId::STATE);

  // Verify state frame contents
  ASSERT_EQ(parsed_frame.payload_len, sizeof(StateFramePayload));
  const auto * state_payload =
    reinterpret_cast<const StateFramePayload *>(parsed_frame.payload_data);

  // Check sequence echo
  EXPECT_EQ(state_payload->seq_echo, cmd_payload.seq);

  // Check protocol version is set correctly
  EXPECT_EQ(state_payload->protocol_version, PROTOCOL_VERSION);

  // Check servo/ESC echo
  EXPECT_FLOAT_EQ(state_payload->servo_pos_rad, cmd_payload.servo_pos_rad);
  EXPECT_FLOAT_EQ(state_payload->servo_cont_vel_norm, cmd_payload.servo_cont_vel_norm);
  EXPECT_FLOAT_EQ(state_payload->esc_norm[0], cmd_payload.esc_norm[0]);
  EXPECT_FLOAT_EQ(state_payload->esc_norm[1], cmd_payload.esc_norm[1]);

  // Check simulation properties
  EXPECT_GT(state_payload->dt_ms, 0);
  EXPECT_EQ(state_payload->flags, 0);
  EXPECT_EQ(state_payload->error_code, 0);
}

TEST_F(SerialLoopbackTest, MultipleFrameSequence) {
  ASSERT_TRUE(loopback_backend_->open(SerialOptions{}));

  // Send multiple command frames with different sequence numbers
  for (uint16_t seq = 1; seq <= 5; ++seq) {
    CommandFramePayload cmd_payload = {};
    cmd_payload.wheel_vel_rad_s[0] = static_cast<float>(seq) * 0.1f;
    cmd_payload.seq = seq;

    // Encode and send
    uint8_t cmd_buffer[128];
    size_t cmd_bytes = 0;
    auto encode_result = encodeCommand(cmd_payload, cmd_buffer, sizeof(cmd_buffer), cmd_bytes);
    ASSERT_EQ(encode_result, ErrorCode::OK);

    int written = loopback_backend_->write(cmd_buffer, cmd_bytes);
    EXPECT_EQ(written, static_cast<int>(cmd_bytes));

    // Read response
    uint8_t read_buffer[128];
    int bytes_read = loopback_backend_->read(read_buffer, sizeof(read_buffer));
    EXPECT_GT(bytes_read, 0);

    // Parse and verify sequence echo
    ByteSpan input(read_buffer, bytes_read);
    ParsedFrame parsed_frame;
    auto parse_result = tryParseFrame(input, parsed_frame);
    EXPECT_EQ(parse_result, ParseResult::SUCCESS);

    const auto * state_payload =
      reinterpret_cast<const StateFramePayload *>(parsed_frame.payload_data);
    EXPECT_EQ(state_payload->seq_echo, seq);
  }
}

TEST_F(SerialLoopbackTest, InvalidFrameHandling) {
  ASSERT_TRUE(loopback_backend_->open(SerialOptions{}));

  // Send invalid data
  uint8_t invalid_data[] = {0xFF, 0xFF, 0xFF, 0xFF};
  int written = loopback_backend_->write(invalid_data, sizeof(invalid_data));
  EXPECT_EQ(written, static_cast<int>(sizeof(invalid_data)));

  // Should not generate any response
  uint8_t read_buffer[128];
  int bytes_read = loopback_backend_->read(read_buffer, sizeof(read_buffer));
  EXPECT_EQ(bytes_read, 0);
}

TEST_F(SerialLoopbackTest, FrameInjection) {
  ASSERT_TRUE(loopback_backend_->open(SerialOptions{}));

  // Create a custom state frame
  StateFramePayload state_payload = {};
  state_payload.encoder_counts[0] = 1000;
  state_payload.encoder_counts[1] = 2000;
  state_payload.dt_ms = 25;
  state_payload.seq_echo = 99;
  state_payload.flags = 0x01; // Some test flag
  state_payload.error_code = 2; // Some test error

  // Encode the state frame
  uint8_t state_buffer[128];
  size_t state_bytes = 0;
  auto encode_result = encodeState(state_payload, state_buffer, sizeof(state_buffer), state_bytes);
  ASSERT_EQ(encode_result, ErrorCode::OK);

  // Inject it directly
  loopback_backend_->injectStateFrame(state_buffer, state_bytes);

  // Read it back
  uint8_t read_buffer[128];
  int bytes_read = loopback_backend_->read(read_buffer, sizeof(read_buffer));
  EXPECT_EQ(bytes_read, static_cast<int>(state_bytes));

  // Parse and verify
  ByteSpan input(read_buffer, bytes_read);
  ParsedFrame parsed_frame;
  auto parse_result = tryParseFrame(input, parsed_frame);
  EXPECT_EQ(parse_result, ParseResult::SUCCESS);

  const auto * read_state = reinterpret_cast<const StateFramePayload *>(parsed_frame.payload_data);
  EXPECT_EQ(read_state->encoder_counts[0], state_payload.encoder_counts[0]);
  EXPECT_EQ(read_state->encoder_counts[1], state_payload.encoder_counts[1]);
  EXPECT_EQ(read_state->dt_ms, state_payload.dt_ms);
  EXPECT_EQ(read_state->seq_echo, state_payload.seq_echo);
  EXPECT_EQ(read_state->flags, state_payload.flags);
  EXPECT_EQ(read_state->error_code, state_payload.error_code);
}

TEST_F(SerialLoopbackTest, ClearInput) {
  ASSERT_TRUE(loopback_backend_->open(SerialOptions{}));

  // Inject some data
  StateFramePayload state_payload = {};
  uint8_t state_buffer[128];
  size_t state_bytes = 0;
  encodeState(state_payload, state_buffer, sizeof(state_buffer), state_bytes);
  loopback_backend_->injectStateFrame(state_buffer, state_bytes);

  // Verify data is there
  uint8_t read_buffer[128];
  int bytes_read = loopback_backend_->read(read_buffer, sizeof(read_buffer));
  EXPECT_GT(bytes_read, 0);

  // Inject more data and clear
  loopback_backend_->injectStateFrame(state_buffer, state_bytes);
  loopback_backend_->clearInput();

  // Should read nothing
  bytes_read = loopback_backend_->read(read_buffer, sizeof(read_buffer));
  EXPECT_EQ(bytes_read, 0);
}
