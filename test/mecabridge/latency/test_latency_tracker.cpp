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

#include "mecabridge_utils/latency/latency_tracker.hpp"

#include <chrono>
#include <thread>
#include <memory>

#include <gtest/gtest.h>


using mecabridge::latency::LatencyTracker;
using mecabridge::latency::Duration;
using namespace std::chrono_literals;  // enable 10ms style literals

class LatencyTrackerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tracker_ = std::make_unique<LatencyTracker>(10);   // Small window for testing
  }

  std::unique_ptr<LatencyTracker> tracker_;
};

TEST_F(LatencyTrackerTest, InitialState)
{
  EXPECT_EQ(tracker_->getSampleCount(), 0);
  EXPECT_EQ(tracker_->getP95Latency(), Duration::zero());
  EXPECT_EQ(tracker_->getMeanLatency(), Duration::zero());
  EXPECT_EQ(tracker_->getMaxLatency(), Duration::zero());
}

TEST_F(LatencyTrackerTest, SingleRoundTrip)
{
  auto start_time = std::chrono::steady_clock::now();
  auto end_time = start_time + 10ms;

  tracker_->recordCommandSent(1, start_time);
  tracker_->recordStateEcho(1, end_time);

  EXPECT_EQ(tracker_->getSampleCount(), 1);
  EXPECT_GE(tracker_->getP95Latency(), 9ms);
  EXPECT_LE(tracker_->getP95Latency(), 11ms);
  EXPECT_EQ(tracker_->getMeanLatency(), tracker_->getP95Latency());
  EXPECT_EQ(tracker_->getMaxLatency(), tracker_->getP95Latency());
}

TEST_F(LatencyTrackerTest, MultipleRoundTrips)
{
  auto base_time = std::chrono::steady_clock::now();

  // Record several round trips with increasing latencies
  for (uint16_t i = 1; i <= 5; ++i) {
    auto latency = std::chrono::milliseconds(i * 5);     // 5ms, 10ms, 15ms, 20ms, 25ms
    tracker_->recordCommandSent(i, base_time + std::chrono::milliseconds(i * 100));
    tracker_->recordStateEcho(i, base_time + std::chrono::milliseconds(i * 100) + latency);
  }

  EXPECT_EQ(tracker_->getSampleCount(), 5);

  // P95 of 5 samples should be the largest or second largest
  auto p95 = tracker_->getP95Latency();
  EXPECT_GE(p95, 20ms);
  EXPECT_LE(p95, 25ms);

  // Mean should be around 15ms
  auto mean = tracker_->getMeanLatency();
  EXPECT_GE(mean, 14ms);
  EXPECT_LE(mean, 16ms);

  // Max should be 25ms
  auto max = tracker_->getMaxLatency();
  EXPECT_GE(max, 24ms);
  EXPECT_LE(max, 26ms);
}

TEST_F(LatencyTrackerTest, SlidingWindow)
{
  auto base_time = std::chrono::steady_clock::now();

  // Fill window beyond capacity (window size is 10)
  for (uint16_t i = 1; i <= 15; ++i) {
    auto latency = std::chrono::milliseconds(i);
    tracker_->recordCommandSent(i, base_time + std::chrono::milliseconds(i * 10));
    tracker_->recordStateEcho(i, base_time + std::chrono::milliseconds(i * 10) + latency);
  }

  // Should only keep last 10 samples
  EXPECT_EQ(tracker_->getSampleCount(), 10);

  // Since we kept the last 10 samples (6ms to 15ms), min should be around 6ms
  auto mean = tracker_->getMeanLatency();
  EXPECT_GE(mean, 9ms);   // (6+7+...+15)/10 = 10.5ms
  EXPECT_LE(mean, 12ms);
}

TEST_F(LatencyTrackerTest, MissingEchos)
{
  auto base_time = std::chrono::steady_clock::now();

  // Send commands but only echo some
  tracker_->recordCommandSent(1, base_time);
  tracker_->recordCommandSent(2, base_time + 10ms);
  tracker_->recordCommandSent(3, base_time + 20ms);

  // Only echo sequence 2
  tracker_->recordStateEcho(2, base_time + 30ms);

  // Should only have one sample
  EXPECT_EQ(tracker_->getSampleCount(), 1);

  auto latency = tracker_->getP95Latency();
  EXPECT_GE(latency, 19ms);   // 30ms - 10ms = 20ms
  EXPECT_LE(latency, 21ms);
}

TEST_F(LatencyTrackerTest, OutOfOrderEchos)
{
  auto base_time = std::chrono::steady_clock::now();

  // Send commands in order
  tracker_->recordCommandSent(1, base_time);
  tracker_->recordCommandSent(2, base_time + 10ms);
  tracker_->recordCommandSent(3, base_time + 20ms);

  // Echo out of order
  tracker_->recordStateEcho(3, base_time + 25ms);   // 5ms latency
  tracker_->recordStateEcho(1, base_time + 30ms);   // 30ms latency
  tracker_->recordStateEcho(2, base_time + 35ms);   // 25ms latency

  EXPECT_EQ(tracker_->getSampleCount(), 3);

  // Max should be 30ms
  auto max = tracker_->getMaxLatency();
  EXPECT_GE(max, 29ms);
  EXPECT_LE(max, 31ms);
}

TEST_F(LatencyTrackerTest, Reset)
{
  auto base_time = std::chrono::steady_clock::now();

  // Add some samples
  tracker_->recordCommandSent(1, base_time);
  tracker_->recordStateEcho(1, base_time + 10ms);

  EXPECT_EQ(tracker_->getSampleCount(), 1);

  // Reset should clear everything
  tracker_->reset();

  EXPECT_EQ(tracker_->getSampleCount(), 0);
  EXPECT_EQ(tracker_->getP95Latency(), Duration::zero());
  EXPECT_EQ(tracker_->getMeanLatency(), Duration::zero());
  EXPECT_EQ(tracker_->getMaxLatency(), Duration::zero());
}

TEST_F(LatencyTrackerTest, SequenceNumberWrapAround)
{
  auto base_time = std::chrono::steady_clock::now();

  // Test with sequence numbers near uint16_t max
  uint16_t seq = 65534;
  tracker_->recordCommandSent(seq, base_time);
  tracker_->recordStateEcho(seq, base_time + 5ms);

  seq = 65535;
  tracker_->recordCommandSent(seq, base_time + 10ms);
  tracker_->recordStateEcho(seq, base_time + 15ms);

  // Wrap around to 0
  seq = 0;
  tracker_->recordCommandSent(seq, base_time + 20ms);
  tracker_->recordStateEcho(seq, base_time + 25ms);

  EXPECT_EQ(tracker_->getSampleCount(), 3);
}

TEST_F(LatencyTrackerTest, PendingCommandCleanup)
{
  auto base_time = std::chrono::steady_clock::now();

  // Send commands but don't echo them
  for (uint16_t i = 1; i <= 5; ++i) {
    tracker_->recordCommandSent(i, base_time - std::chrono::seconds(2));     // Old commands
  }

  // Send a new command much later
  tracker_->recordCommandSent(10, base_time);
  tracker_->recordStateEcho(10, base_time + 5ms);

  // Should only have one sample (old pending commands should be cleaned up)
  EXPECT_EQ(tracker_->getSampleCount(), 1);
}
