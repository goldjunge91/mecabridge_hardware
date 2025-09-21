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

#include "crc16.hpp"

namespace mecabridge
{
namespace protocol
{

CRC16::CRC16()
: crc_(INIT_VALUE)
{
}

void CRC16::reset()
{
  crc_ = INIT_VALUE;
}

void CRC16::update(uint8_t byte)
{
  // CRC-16/CCITT-FALSE algorithm implementation
  crc_ ^= static_cast<uint16_t>(byte) << 8;
  for (int b = 0; b < 8; ++b) {
    if (crc_ & 0x8000) {
      crc_ = (crc_ << 1) ^ POLYNOMIAL;
    } else {
      crc_ <<= 1;
    }
  }
}

void CRC16::update(const uint8_t * data, size_t len)
{
  for (size_t i = 0; i < len; ++i) {
    update(data[i]);
  }
}

uint16_t CRC16::finalize() const
{
  return crc_;
}

uint16_t crc16_ccitt_false(const uint8_t * data, size_t len)
{
  if (data == nullptr || len == 0) {
    return CRC16::INIT_VALUE;
  }

  CRC16 crc;
  crc.update(data, len);
  return crc.finalize();
}

} // namespace protocol
} // namespace mecabridge
