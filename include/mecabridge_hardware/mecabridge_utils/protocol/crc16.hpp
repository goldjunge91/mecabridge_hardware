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

// #ifndef MECABRIDGE_UTILS__PROTOCOL__CRC16_HPP_
// #define MECABRIDGE_UTILS__PROTOCOL__CRC16_HPP_

// #pragma once

// // Wrapper header to expose package-prefixed include path
// // "mecabridge_hardware/mecabridge_utils/protocol/crc16.hpp"
// // and provide a legacy unqualified symbol for tests that expect
// // crc16_ccitt_false to live in the global namespace.

// #include "mecabridge_utils/protocol/crc16.hpp"

// // Inline forwarding function in the global namespace so older test code that
// // calls crc16_ccitt_false(...) without qualification still links and compiles.
// inline uint16_t crc16_ccitt_false(const uint8_t * data, size_t len)
// {
//   return ::mecabridge::protocol::crc16_ccitt_false(data, len);
// }
// #endif  // MECABRIDGE_HARDWARE__MECABRIDGE_UTILS__PROTOCOL__CRC16_HPP_
#ifndef MECABRIDGE_HARDWARE__MECABRIDGE_UTILS__PROTOCOL__CRC16_HPP_
#define MECABRIDGE_HARDWARE__MECABRIDGE_UTILS__PROTOCOL__CRC16_HPP_

// Wrapper header to expose package-prefixed include path
// "mecabridge_hardware/mecabridge_utils/protocol/crc16.hpp"
#pragma once

#include "mecabridge_utils/protocol/crc16.hpp"

// Inline forwarding function in the global namespace so older test code that
// calls crc16_ccitt_false(...) without qualification still links and compiles.
inline uint16_t crc16_ccitt_false(const uint8_t * data, size_t len)
{
  return ::mecabridge::protocol::crc16_ccitt_false(data, len);
}

#endif // MECABRIDGE_HARDWARE__MECABRIDGE_UTILS__PROTOCOL__CRC16_HPP_
