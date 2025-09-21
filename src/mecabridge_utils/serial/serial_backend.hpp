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

// Primary include guard
#ifndef MECABRIDGE_UTILS__SERIAL__SERIAL_BACKEND_HPP_
#define MECABRIDGE_UTILS__SERIAL__SERIAL_BACKEND_HPP_

#pragma once

#include <algorithm>

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

namespace mecabridge
{
namespace serial
{

struct SerialOptions
{
  std::string device;
  int baud_rate = 115200;
  int read_timeout_ms = 20;
  int write_timeout_ms = 20;
};

class SerialBackend
{
public:
  virtual ~SerialBackend() = default;

  virtual bool open(const SerialOptions & opts) = 0;
  virtual void close() = 0;
  virtual bool is_open() const = 0;

  // Returns bytes read; 0 on timeout; negative on error
  virtual int read(uint8_t * buf, size_t len) = 0;
  // Returns bytes written; negative on error
  virtual int write(const uint8_t * buf, size_t len) = 0;
};

// A simple in-memory loopback stub for tests/integration skeletons
class LoopbackBackend : public SerialBackend
{
public:
  LoopbackBackend() = default;
  bool open(const SerialOptions & opts) override {opts_ = opts; open_ = true; return true;}
  void close() override {open_ = false; rx_.clear(); tx_.clear();}
  bool is_open() const override {return open_;}

  int read(uint8_t * buf, size_t len) override
  {
    if (!open_) {return -1;}
    size_t n = rx_.size();
    if (n == 0) {
      return 0;            // timeout behavior for stub
    }
    size_t to_copy = (len < n) ? len : n;
    std::copy(rx_.begin(), rx_.begin() + to_copy, buf);
    rx_.erase(rx_.begin(), rx_.begin() + to_copy);
    return static_cast<int>(to_copy);
  }

  int write(const uint8_t * buf, size_t len) override
  {
    if (!open_) {return -1;}
    // Echo written bytes to rx_ for loopback
    tx_.insert(tx_.end(), buf, buf + len);
    rx_.insert(rx_.end(), buf, buf + len);
    return static_cast<int>(len);
  }

private:
  SerialOptions opts_{};
  bool open_ = false;
  std::vector<uint8_t> rx_;
  std::vector<uint8_t> tx_;
};

} // namespace serial
} // namespace mecabridge

#endif  // MECABRIDGE_UTILS__SERIAL__SERIAL_BACKEND_HPP_
