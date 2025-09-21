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

#include <chrono>
#include <cstdint>

namespace mecabridge
{
namespace safety
{

class Watchdog
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  explicit Watchdog(std::chrono::milliseconds timeout = std::chrono::milliseconds(150));

  void updateOnValidFrame(TimePoint now);
  bool tripped(TimePoint now);
  void reset();
  void reset(TimePoint now);
  uint16_t flags();

private:
  std::chrono::milliseconds timeout_;
  TimePoint last_valid_frame_time_;
  bool tripped_ = false;
};

} // namespace safety
} // namespace mecabridge
