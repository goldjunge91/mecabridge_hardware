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
#include "mecabridge_utils/protocol/crc16.hpp"

class ProtocolEmptyTest : public ::testing::Test
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

TEST_F(ProtocolEmptyTest, HeadersIncludeCleanly) {
  // Verify headers can be included without errors
  EXPECT_TRUE(true);
}

TEST_F(ProtocolEmptyTest, EnumsAreDefined) {
  // Verify enums are properly defined
  using namespace mecabridge::protocol;

  EXPECT_EQ(static_cast<uint8_t>(FrameId::COMMAND), 0x01);
  EXPECT_EQ(static_cast<uint8_t>(FrameId::STATE), 0x02);
  EXPECT_EQ(static_cast<uint8_t>(ErrorCode::OK), 0);
  EXPECT_EQ(static_cast<uint8_t>(ErrorCode::NOT_IMPLEMENTED), 255);
}

TEST_F(ProtocolEmptyTest, ConstantsAreDefined) {
  using namespace mecabridge::protocol;

  EXPECT_EQ(START_BYTE, 0xAA);
  EXPECT_EQ(PROTOCOL_VERSION, 1);
}

TEST_F(ProtocolEmptyTest, StructSizesAreCorrect) {
  using namespace mecabridge::protocol;

  EXPECT_EQ(sizeof(CommandFramePayload), 36);
  EXPECT_EQ(sizeof(StateFramePayload), 40);
}

TEST_F(ProtocolEmptyTest, CRC16ClassInstantiates) {
  using namespace mecabridge::protocol;

  CRC16 crc;
  EXPECT_EQ(crc.finalize(), CRC16::INIT_VALUE);    // Should return init value before any updates
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
