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

using namespace mecabridge::protocol;

TEST(ProtocolHandshakeTest, VersionMismatch) {
  // 1. Create a state frame with a mismatched protocol version
  StateFramePayload state_payload = {};   // Zero-initialize
  state_payload.protocol_version = PROTOCOL_VERSION + 1;   // Mismatch

  // 2. Manually encode this frame to preserve the wrong version
  uint8_t buffer[128];
  size_t offset = 0;

  // START_BYTE
  buffer[offset++] = START_BYTE;

  // FRAME_ID
  buffer[offset++] = static_cast<uint8_t>(FrameId::STATE);

  // LEN (payload size)
  buffer[offset++] = sizeof(StateFramePayload);

  // PAYLOAD - copy the struct directly with wrong version
  std::memcpy(&buffer[offset], &state_payload, sizeof(StateFramePayload));
  offset += sizeof(StateFramePayload);

  // Calculate CRC over FRAME_ID + LEN + PAYLOAD
  uint16_t crc = mecabridge::protocol::crc16_ccitt_false(&buffer[1], offset - 1);

  // CRC16 (MSB first)
  buffer[offset++] = static_cast<uint8_t>((crc >> 8) & 0xFF);
  buffer[offset++] = static_cast<uint8_t>(crc & 0xFF);

  size_t bytes_written = offset;

  // 3. Attempt to parse the frame
  ParsedFrame parsed_frame;
  ByteSpan input_span(buffer, bytes_written);
  ParseResult parse_res = tryParseFrame(input_span, parsed_frame);

  // 4. Assert that the parse fails with a version mismatch error
  ASSERT_EQ(parse_res, ParseResult::VERSION_MISMATCH);
}

TEST(ProtocolHandshakeTest, VersionMatch) {
  // 1. Create a state frame with a matching protocol version
  StateFramePayload state_payload = {};   // Zero-initialize
  // The protocol version is set automatically by encodeState

  // 2. Encode this frame into a buffer
  uint8_t buffer[128];
  size_t bytes_written = 0;
  ErrorCode err = encodeState(state_payload, buffer, sizeof(buffer), bytes_written);

  ASSERT_EQ(err, ErrorCode::OK);
  ASSERT_GT(bytes_written, 0);

  // 3. Attempt to parse the frame
  ParsedFrame parsed_frame;
  ByteSpan input_span(buffer, bytes_written);
  ParseResult parse_res = tryParseFrame(input_span, parsed_frame);

  // 4. Assert that the parse succeeds
  ASSERT_EQ(parse_res, ParseResult::SUCCESS);
}
