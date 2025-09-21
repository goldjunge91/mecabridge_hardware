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

#include "latency_tracker.hpp"

#include <algorithm>

#include <numeric>

namespace mecabridge
{
namespace latency
{

LatencyTracker::LatencyTracker(size_t window_size)
: window_size_(window_size)
{
  latencies_.reserve(window_size_);
  pending_commands_.reserve(50);           // Reasonable limit for pending commands
}

void LatencyTracker::recordCommandSent(uint16_t seq, TimePoint send_time)
{
  // Clean up old pending commands first
  cleanupOldPending(send_time);

  // Add new pending command
  pending_commands_.push_back({seq, send_time});
}

void LatencyTracker::recordStateEcho(uint16_t seq_echo, TimePoint receive_time)
{
  // Clean up old pending commands first
  cleanupOldPending(receive_time);

  // Find matching pending command
  auto it = std::find_if(
    pending_commands_.begin(), pending_commands_.end(),
    [seq_echo](const PendingCommand & cmd)
    {
      return cmd.seq == seq_echo;
    });

  if (it != pending_commands_.end()) {
    // Calculate latency
    Duration latency = receive_time - it->send_time;

    // Add to sliding window
    if (latencies_.size() >= window_size_) {
      // Remove oldest sample to maintain window size
      latencies_.erase(latencies_.begin());
    }
    latencies_.push_back(latency);

    // Remove the matched pending command
    pending_commands_.erase(it);
  }
}

Duration LatencyTracker::getP95Latency() const
{
  if (latencies_.empty()) {
    return Duration::zero();
  }

  // Create a sorted copy to find percentile
  std::vector<Duration> sorted_latencies = latencies_;
  std::sort(sorted_latencies.begin(), sorted_latencies.end());

  // Calculate 95th percentile index
  size_t p95_index = static_cast<size_t>(0.95 * sorted_latencies.size());
  if (p95_index >= sorted_latencies.size()) {
    p95_index = sorted_latencies.size() - 1;
  }

  return sorted_latencies[p95_index];
}

Duration LatencyTracker::getMeanLatency() const
{
  if (latencies_.empty()) {
    return Duration::zero();
  }

  // Calculate mean using accumulate
  auto total = std::accumulate(latencies_.begin(), latencies_.end(), Duration::zero());
  return total / latencies_.size();
}

Duration LatencyTracker::getMaxLatency() const
{
  if (latencies_.empty()) {
    return Duration::zero();
  }

  return *std::max_element(latencies_.begin(), latencies_.end());
}

size_t LatencyTracker::getSampleCount() const
{
  return latencies_.size();
}

void LatencyTracker::reset()
{
  latencies_.clear();
  pending_commands_.clear();
}

void LatencyTracker::cleanupOldPending(TimePoint current_time)
{
  // Remove pending commands older than MAX_PENDING_TIME
  pending_commands_.erase(
    std::remove_if(
      pending_commands_.begin(), pending_commands_.end(),
      [current_time](const PendingCommand & cmd)
      {
        return (current_time - cmd.send_time) > MAX_PENDING_TIME;
      }),
    pending_commands_.end());
}

}     // namespace latency
} // namespace mecabridge
