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

#ifndef MECABRIDGE_UTILS__SERIAL__MOCK_SERIAL_BACKEND_HPP_
#define MECABRIDGE_UTILS__SERIAL__MOCK_SERIAL_BACKEND_HPP_


#pragma once

#include "serial_backend.hpp"

namespace mecabridge
{
namespace serial
{

class MockSerialBackend : public SerialBackend
{
public:
  MockSerialBackend()
  : is_open_(false) {}
  ~MockSerialBackend() override = default;

  bool open(const SerialOptions & opts) override
  {
    (void)opts; // Suppress unused parameter warning
    is_open_ = true;
    return true;
  }

  void close() override
  {
    is_open_ = false;
  }

  bool is_open() const override
  {
    return is_open_;
  }

  int read(uint8_t * buf, size_t len) override
  {
    (void)buf; // Suppress unused parameter warning
    (void)len;
    return 0; // No data available
  }

  int write(const uint8_t * buf, size_t len) override
  {
    (void)buf; // Suppress unused parameter warning
    return static_cast<int>(len); // Pretend all data was written
  }

private:
  bool is_open_;
};

} // namespace serial
} // namespace mecabridge

#endif  // MECABRIDGE_UTILS__SERIAL__MOCK_SERIAL_BACKEND_HPP_
