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
#include "mecabridge_utils/protocol/crc16.hpp"
#include <string>
#include <vector>

class CRC16Test : public ::testing::Test
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
};

TEST_F(CRC16Test, TestVector123456789) {
  // Test vector from CRC-16/CCITT-FALSE specification
  // Input: "123456789" -> Expected: 0x29B1
  const std::string test_data = "123456789";
  const uint8_t * data = reinterpret_cast<const uint8_t *>(test_data.c_str());

  uint16_t result = mecabridge::protocol::crc16_ccitt_false(data, test_data.length());
  EXPECT_EQ(result, 0x29B1);
}

TEST_F(CRC16Test, EmptyBuffer) {
  // Test with empty buffer should return init value
  uint16_t result = mecabridge::protocol::crc16_ccitt_false(nullptr, 0);
  EXPECT_EQ(result, 0xFFFF);    // Init value
}

TEST_F(CRC16Test, SingleByte) {
  // Test with single byte
  uint8_t data = 0x00;
  uint16_t result = mecabridge::protocol::crc16_ccitt_false(&data, 1);
  // Expected result for single 0x00 byte with CRC-16/CCITT-FALSE
  EXPECT_EQ(result, 0xE1F0);
}

TEST_F(CRC16Test, ClassBasedCalculation) {
  // Test using CRC16 class
  mecabridge::protocol::CRC16 crc;

  const std::string test_data = "123456789";
  crc.update(reinterpret_cast<const uint8_t *>(test_data.c_str()), test_data.length());

  uint16_t result = crc.finalize();
  EXPECT_EQ(result, 0x29B1);
}

TEST_F(CRC16Test, IncrementalUpdate) {
  // Test incremental updates produce same result as one-shot
  mecabridge::protocol::CRC16 crc;

  const std::string test_data = "123456789";

  // Update byte by byte
  for (char c : test_data) {
    crc.update(static_cast<uint8_t>(c));
  }

  uint16_t result = crc.finalize();
  EXPECT_EQ(result, 0x29B1);
}

TEST_F(CRC16Test, ResetFunctionality) {
  // Test reset functionality
  mecabridge::protocol::CRC16 crc;

  // Update with some data
  crc.update(0x42);

  // Reset and check we're back to init value
  crc.reset();
  EXPECT_EQ(crc.finalize(), mecabridge::protocol::CRC16::INIT_VALUE);
}

TEST_F(CRC16Test, RandomFuzzTesting) {
  // Test with some random small buffers
  std::vector<uint8_t> test_buffers[] = {
    {0x01, 0x02, 0x03},
    {0xFF, 0xFF, 0xFF},
    {0x00, 0x01, 0x02, 0x03, 0x04},
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}
  };

  for (const auto & buffer : test_buffers) {
    // Calculate using both methods and ensure they match
    uint16_t one_shot = mecabridge::protocol::crc16_ccitt_false(buffer.data(), buffer.size());

    mecabridge::protocol::CRC16 crc;
    crc.update(buffer.data(), buffer.size());
    uint16_t incremental = crc.finalize();

    EXPECT_EQ(one_shot, incremental) << "CRC mismatch for buffer of size " << buffer.size();
  }
}

TEST_F(CRC16Test, FrameProtocolExample) {
  // Test with a realistic frame protocol example
  // START_BYTE (0xAA) + FRAME_ID (0x01) + LEN (0x24 = 36) + minimal payload
  std::vector<uint8_t> frame_header = {0x01, 0x24};   // FRAME_ID + LEN (excluding START_BYTE per protocol)

  uint16_t crc_result = mecabridge::protocol::crc16_ccitt_false(
    frame_header.data(), frame_header.size());

  // This should produce a consistent result (exact value depends on algorithm correctness)
  EXPECT_NE(crc_result, 0x0000);    // Should not be zero for this input
  EXPECT_NE(crc_result, 0xFFFF);    // Should not be init value
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
