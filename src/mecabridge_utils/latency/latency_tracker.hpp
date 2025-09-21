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

// Header guard
#ifndef MECABRIDGE_UTILS__LATENCY__LATENCY_TRACKER_HPP_
#define MECABRIDGE_UTILS__LATENCY__LATENCY_TRACKER_HPP_

#pragma once

#include <chrono>
#include <vector>
#include <algorithm>

#include <cstdint>

namespace mecabridge
{
namespace latency
{

using TimePoint = std::chrono::steady_clock::time_point;
using Duration = std::chrono::steady_clock::duration;

/**
 * @brief Tracks round-trip latency by measuring time between command send and state echo
 *
 * This class maintains a sliding window of latency measurements and computes statistics
 * like the 95th percentile (p95) to monitor communication performance.
 */
class LatencyTracker
{
public:
  /**
   * @brief Construct a new Latency Tracker
   *
   * @param window_size Maximum number of samples to keep in sliding window
   */
  explicit LatencyTracker(size_t window_size = 100);

  /**
   * @brief Record when a command with given sequence number was sent
   *
   * @param seq Command sequence number
   * @param send_time Time when command was sent
   */
  void recordCommandSent(uint16_t seq, TimePoint send_time);

  /**
   * @brief Record when a state frame echoed a given sequence number
   *
   * @param seq_echo Sequence number echoed from firmware
   * @param receive_time Time when echo was received
   */
  void recordStateEcho(uint16_t seq_echo, TimePoint receive_time);

  /**
   * @brief Get the current 95th percentile latency
   *
   * @return Duration representing p95 latency, or zero if insufficient samples
   */
  Duration getP95Latency() const;

  /**
   * @brief Get the mean latency
   *
   * @return Duration representing mean latency, or zero if insufficient samples
   */
  Duration getMeanLatency() const;

  /**
   * @brief Get the maximum latency
   *
   * @return Duration representing max latency, or zero if no samples
   */
  Duration getMaxLatency() const;

  /**
   * @brief Get the number of valid latency samples
   *
   * @return Number of completed round-trip measurements
   */
  size_t getSampleCount() const;

  /**
   * @brief Clear all measurements and reset state
   */
  void reset();

private:
  struct PendingCommand
  {
    uint16_t seq;
    TimePoint send_time;
  };

  size_t window_size_;
  std::vector<PendingCommand> pending_commands_;
  std::vector<Duration> latencies_;

  // Remove old pending commands to prevent memory growth
  void cleanupOldPending(TimePoint current_time);

  // Maximum time to keep pending commands (in case echo is lost)
  static constexpr auto MAX_PENDING_TIME = std::chrono::milliseconds(1000);
};

} // namespace latency
} // namespace mecabridge

#endif  // MECABRIDGE_UTILS__LATENCY__LATENCY_TRACKER_HPP_
