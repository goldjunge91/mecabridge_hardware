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

#pragma once

#include <cstdint>
#include <cstddef>

namespace mecabridge
{
namespace protocol
{

/**
 * CRC-16/CCITT-FALSE implementation
 *
 * Specifications:
 * - Polynomial: 0x1021
 * - Init: 0xFFFF
 * - Reflect In: false
 * - Reflect Out: false
 * - XorOut: 0x0000
 *
 * Test vector: "123456789" -> 0x29B1
 */

class CRC16
{
public:
  static constexpr uint16_t POLYNOMIAL = 0x1021;
  static constexpr uint16_t INIT_VALUE = 0xFFFF;

  CRC16();

  void reset();
  void update(uint8_t byte);
  void update(const uint8_t * data, size_t len);
  uint16_t finalize() const;

private:
  uint16_t crc_;
};

// Convenience function for one-shot calculation
uint16_t crc16_ccitt_false(const uint8_t * data, size_t len);

} // namespace protocol
} // namespace mecabridge

// Provide a global convenience forwarding symbol so legacy tests that call
// crc16_ccitt_false(...) without namespace qualification still compile/link.
inline uint16_t crc16_ccitt_false(const uint8_t * data, size_t len)
{
  return ::mecabridge::protocol::crc16_ccitt_false(data, len);
}
