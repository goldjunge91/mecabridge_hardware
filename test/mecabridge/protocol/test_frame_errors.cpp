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
#include "../../../src/mecabridge_utils/protocol/crc16.hpp"

using namespace mecabridge::protocol;

// Test fixture for error handling tests
class FrameErrorTest : public ::testing::Test
{
protected:
  CommandFramePayload test_payload_;
  uint8_t buffer_[128];
  size_t bytes_written_ = 0;

  void SetUp() override
  {
    // Initialize with a valid payload
    test_payload_ = {};     // Zero-initialize
    test_payload_.seq = 123;
    // Encode a valid frame to be tampered with later
    ErrorCode err = encodeCommand(test_payload_, buffer_, sizeof(buffer_), bytes_written_);
    ASSERT_EQ(err, ErrorCode::OK);
    ASSERT_GT(bytes_written_, 0);
  }
};

TEST_F(FrameErrorTest, CrcMismatch) {
  // Corrupt the last byte of the payload (before CRC)
  buffer_[bytes_written_ - 3]++;

  ParsedFrame parsed_frame;
  ByteSpan input_span(buffer_, bytes_written_);
  ParseResult result = tryParseFrame(input_span, parsed_frame);

  ASSERT_EQ(result, ParseResult::CRC_MISMATCH);
}

TEST_F(FrameErrorTest, TruncatedFrame) {
  // Simulate a truncated frame by reducing the number of bytes
  ParsedFrame parsed_frame;
  ByteSpan input_span(buffer_, bytes_written_ - 1);
  ParseResult result = tryParseFrame(input_span, parsed_frame);

  ASSERT_EQ(result, ParseResult::INSUFFICIENT_DATA);
}

TEST_F(FrameErrorTest, UnknownFrameId) {
  // Set an invalid frame ID
  buffer_[1] = 0xFF;   // Invalid FrameId

  // We need to recalculate the CRC since we changed the frame ID
  uint16_t crc = ::mecabridge::protocol::crc16_ccitt_false(
    &buffer_[1],
    sizeof(CommandFramePayload) + 2);
  buffer_[bytes_written_ - 2] = static_cast<uint8_t>((crc >> 8) & 0xFF);
  buffer_[bytes_written_ - 1] = static_cast<uint8_t>(crc & 0xFF);

  ParsedFrame parsed_frame;
  ByteSpan input_span(buffer_, bytes_written_);
  ParseResult result = tryParseFrame(input_span, parsed_frame);

  ASSERT_EQ(result, ParseResult::INVALID_FRAME_ID);
}
