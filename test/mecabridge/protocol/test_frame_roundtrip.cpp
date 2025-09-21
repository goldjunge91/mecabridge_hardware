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
#include "mecabridge_utils/protocol/frame.hpp"
#include <cstring>
#include <vector>

class FrameRoundtripTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Setup for each test
  }

  void TearDown() override
  {
    // Cleanup after each test
  }

  // Helper function to create a test command payload
  mecabridge::protocol::CommandFramePayload createTestCommandPayload()
  {
    mecabridge::protocol::CommandFramePayload cmd = {};

    // Set wheel velocities (FL, FR, RL, RR)
    cmd.wheel_vel_rad_s[0] = 1.0f;       // FL
    cmd.wheel_vel_rad_s[1] = -1.0f;      // FR
    cmd.wheel_vel_rad_s[2] = 0.5f;       // RL
    cmd.wheel_vel_rad_s[3] = -0.5f;      // RR

    // Set servo positions/velocities
    cmd.servo_pos_rad = 0.785f;          // ~45 degrees
    cmd.servo_cont_vel_norm = 0.3f;      // 30% speed

    // Set ESC values
    cmd.esc_norm[0] = 0.8f;
    cmd.esc_norm[1] = -0.2f;

    // Set sequence number
    cmd.seq = 42;

    // Reserved flags (should be 0)
    cmd.reserved_flags = 0;

    return cmd;
  }

  // Helper function to create a test state payload
  mecabridge::protocol::StateFramePayload createTestStatePayload()
  {
    mecabridge::protocol::StateFramePayload state = {};

    // Set encoder counts
    state.encoder_counts[0] = 1000;
    state.encoder_counts[1] = 2000;
    state.encoder_counts[2] = 1500;
    state.encoder_counts[3] = 2500;

    // Set timing
    state.dt_ms = 20;      // 20ms = 50Hz

    // Set servo feedback
    state.servo_pos_rad = 0.785f;
    state.servo_cont_vel_norm = 0.3f;

    // Set ESC feedback
    state.esc_norm[0] = 0.8f;
    state.esc_norm[1] = -0.2f;

    // Set sequence echo
    state.seq_echo = 42;

    // Set flags
    state.flags = 0x0001;      // Some flag set

    // Set error code
    state.error_code = static_cast<uint8_t>(mecabridge::protocol::ErrorCode::OK);

    return state;
  }
};

TEST_F(FrameRoundtripTest, CommandFrameSize) {
  // Test that command frame has expected total size
  // Structure: START_BYTE(1) + FRAME_ID(1) + LEN(1) + PAYLOAD(35) + CRC16(2) = 40 bytes

  mecabridge::protocol::CommandFramePayload cmd = createTestCommandPayload();
  uint8_t buffer[64];
  size_t bytes_written = 0;

  mecabridge::protocol::ErrorCode result =
    mecabridge::protocol::encodeCommand(cmd, buffer, sizeof(buffer), bytes_written);

  // Now that encoding is implemented, test the actual results
  EXPECT_EQ(result, mecabridge::protocol::ErrorCode::OK);
  EXPECT_EQ(bytes_written, 41);        // Total frame size with protocol_version
  EXPECT_EQ(buffer[0], 0xAA);      // START_BYTE
  EXPECT_EQ(buffer[1], 0x01);      // FRAME_ID::COMMAND
  EXPECT_EQ(buffer[2], 36);        // LEN (payload size)
}

TEST_F(FrameRoundtripTest, StateFrameSize) {
  // Test that state frame has expected total size
  // Structure: START_BYTE(1) + FRAME_ID(1) + LEN(1) + PAYLOAD(40) + CRC16(2) = 45 bytes

  mecabridge::protocol::StateFramePayload state = createTestStatePayload();
  uint8_t buffer[64];
  size_t bytes_written = 0;

  mecabridge::protocol::ErrorCode result =
    mecabridge::protocol::encodeState(state, buffer, sizeof(buffer), bytes_written);

  // Now that encoding is implemented, test the actual results
  EXPECT_EQ(result, mecabridge::protocol::ErrorCode::OK);
  EXPECT_EQ(bytes_written, 45);        // Total frame size with protocol_version
  EXPECT_EQ(buffer[0], 0xAA);      // START_BYTE
  EXPECT_EQ(buffer[1], 0x02);      // FRAME_ID::STATE
  EXPECT_EQ(buffer[2], 40);        // LEN (payload size)
}

TEST_F(FrameRoundtripTest, PayloadStructSizes) {
  // Verify that our payload structs have the correct sizes
  EXPECT_EQ(sizeof(mecabridge::protocol::CommandFramePayload), 36);
  EXPECT_EQ(sizeof(mecabridge::protocol::StateFramePayload), 40);
}

TEST_F(FrameRoundtripTest, BufferTooSmall) {
  // Test encoding with insufficient buffer size
  mecabridge::protocol::CommandFramePayload cmd = createTestCommandPayload();
  uint8_t small_buffer[10];    // Too small for 40-byte frame
  size_t bytes_written = 0;

  mecabridge::protocol::ErrorCode result = mecabridge::protocol::encodeCommand(
    cmd, small_buffer,
    sizeof(small_buffer),
    bytes_written);

  // Now that encoding is implemented, we expect an error for insufficient buffer
  EXPECT_EQ(result, mecabridge::protocol::ErrorCode::MALFORMED_FRAME);
  EXPECT_EQ(bytes_written, 0);
}

TEST_F(FrameRoundtripTest, FrameParsingPlaceholder) {
  // Test frame parsing with a valid encoded command frame
  mecabridge::protocol::CommandFramePayload cmd = createTestCommandPayload();
  uint8_t buffer[64];
  size_t bytes_written = 0;

  // First encode a command frame
  mecabridge::protocol::ErrorCode encode_result = mecabridge::protocol::encodeCommand(
    cmd, buffer,
    sizeof(buffer),
    bytes_written);
  EXPECT_EQ(encode_result, mecabridge::protocol::ErrorCode::OK);

  // Now try to parse it back
  mecabridge::protocol::ByteSpan input(buffer, bytes_written);
  mecabridge::protocol::ParsedFrame parsed;

  mecabridge::protocol::ParseResult result = mecabridge::protocol::tryParseFrame(input, parsed);

  // With the implementation, we expect success
  EXPECT_EQ(result, mecabridge::protocol::ParseResult::SUCCESS);
  EXPECT_EQ(parsed.frame_id, mecabridge::protocol::FrameId::COMMAND);
  EXPECT_EQ(parsed.payload_len, 36);
  EXPECT_NE(parsed.payload_data, nullptr);
}

TEST_F(FrameRoundtripTest, LittleEndianLayout) {
  // Test that our structs follow little-endian layout expectations
  mecabridge::protocol::CommandFramePayload cmd = {};
  cmd.seq = 0x1234;    // Set a recognizable pattern

  // Cast to bytes and check that it's stored little-endian
  const uint8_t * bytes = reinterpret_cast<const uint8_t *>(&cmd.seq);

  // In little-endian: 0x1234 -> 0x34, 0x12
  EXPECT_EQ(bytes[0], 0x34);
  EXPECT_EQ(bytes[1], 0x12);
}

TEST_F(FrameRoundtripTest, FloatLayout) {
  // Test that float values are stored as expected
  mecabridge::protocol::CommandFramePayload cmd = {};
  cmd.wheel_vel_rad_s[0] = 1.0f;

  // Verify the float is stored as expected (basic sanity check)
  EXPECT_FLOAT_EQ(cmd.wheel_vel_rad_s[0], 1.0f);

  // Check that the memory layout is 4 bytes as expected
  const uint8_t * start = reinterpret_cast<const uint8_t *>(&cmd.wheel_vel_rad_s[0]);
  const uint8_t * next = reinterpret_cast<const uint8_t *>(&cmd.wheel_vel_rad_s[1]);
  EXPECT_EQ(next - start, 4);    // sizeof(float) should be 4
}

TEST_F(FrameRoundtripTest, StructPacking) {
  // Verify that structs are packed without padding
  mecabridge::protocol::CommandFramePayload cmd = {};
  const uint8_t * start = reinterpret_cast<const uint8_t *>(&cmd);
  const uint8_t * seq_ptr = reinterpret_cast<const uint8_t *>(&cmd.seq);

  // seq should be at offset 32 (4*4 wheels + 4 servo_pos + 4 servo_cont + 4*2 esc)
  size_t expected_offset = 4 * 4 + 4 + 4 + 4 * 2; // = 32
  EXPECT_EQ(seq_ptr - start, expected_offset);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
