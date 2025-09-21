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
using mecabridge::latency::LatencyTracker;
using mecabridge::latency::Duration;

#include <memory>
#include <chrono>
#include <vector>
#include <algorithm>

#include <gtest/gtest.h>
#include <random>


/**
 * @brief Integration test for latency measurement with synthetic timing
 *
 * This test feeds synthetic timestamps and verifies that the computed
 * p95 latency is ≤ 20ms for controlled distributions.
 */
class LatencyMeasurementTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create latency tracker with window size of 50 samples
    latency_tracker_ = std::make_unique<mecabridge::latency::LatencyTracker>(50);

    base_time_ = std::chrono::steady_clock::now();
  }

  /**
   * @brief Simulate sending a command and receiving echo with specific latency
   * @param seq_id Sequence ID for the command
   * @param latency_ms Simulated round-trip latency in milliseconds
   */
  void simulateCommandEcho(uint16_t seq_id, double latency_ms)
  {
    auto send_time = base_time_ + std::chrono::milliseconds(static_cast<int64_t>(seq_id * 20));     // 50Hz
    auto echo_time = send_time + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double, std::milli>(latency_ms));

    // Record command send
    latency_tracker_->recordCommandSent(seq_id, send_time);

    // Record echo received
    latency_tracker_->recordStateEcho(seq_id, echo_time);
  }

  std::unique_ptr<mecabridge::latency::LatencyTracker> latency_tracker_;
  std::chrono::steady_clock::time_point base_time_;
};

TEST_F(LatencyMeasurementTest, ConstantLowLatency)
{
  // Feed 50 samples with constant 5ms latency
  for (uint16_t seq = 1; seq <= 50; ++seq) {
    simulateCommandEcho(seq, 5.0);     // 5ms constant latency
  }

  auto p95_duration = latency_tracker_->getP95Latency();
  ASSERT_GT(p95_duration.count(), 0) << "Should have p95 measurement with 50 samples";

  double p95_ms = std::chrono::duration<double, std::milli>(p95_duration).count();
  EXPECT_LE(p95_ms, 20.0) << "p95 latency should be ≤ 20ms";
  EXPECT_NEAR(p95_ms, 5.0, 1.0) << "p95 should be close to 5ms for constant latency";
}

TEST_F(LatencyMeasurementTest, LowVariabilityDistribution)
{
  // Feed samples with low variability (3-7ms range)
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> latency_dist(3.0, 7.0);

  for (uint16_t seq = 1; seq <= 50; ++seq) {
    double latency = latency_dist(gen);
    simulateCommandEcho(seq, latency);
  }

  auto p95_duration = latency_tracker_->getP95Latency();
  ASSERT_GT(p95_duration.count(), 0) << "Should have p95 measurement";

  double p95_ms = std::chrono::duration<double, std::milli>(p95_duration).count();
  EXPECT_LE(p95_ms, 20.0) << "p95 latency should be ≤ 20ms for low variability";
  EXPECT_LE(p95_ms, 10.0) << "p95 should be reasonable for 3-7ms distribution";
}

TEST_F(LatencyMeasurementTest, NormalDistributionWithinBounds)
{
  // Normal distribution with mean=8ms, stddev=2ms
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> latency_dist(8.0, 2.0);   // mean=8ms, stddev=2ms

  for (uint16_t seq = 1; seq <= 100; ++seq) {
    double latency = std::max(1.0, latency_dist(gen));     // Clamp to minimum 1ms
    simulateCommandEcho(seq, latency);
  }

  auto p95_duration = latency_tracker_->getP95Latency();
  ASSERT_GT(p95_duration.count(), 0) << "Should have p95 measurement";

  double p95_ms = std::chrono::duration<double, std::milli>(p95_duration).count();
  EXPECT_LE(p95_ms, 20.0) << "p95 latency should be ≤ 20ms for normal distribution";

  // For normal distribution with mean=8, stddev=2, p95 should be around 11.3ms
  EXPECT_LE(p95_ms, 15.0) << "p95 should be reasonable for normal(8,2) distribution";
}

TEST_F(LatencyMeasurementTest, PerformanceRequirementValidation)
{
  // Test the specific performance requirement: p95 ≤ 20ms
  // Use a distribution that challenges but should meet the requirement

  std::vector<double> latencies = {
    // Most samples are low latency (1-10ms) - 80% of samples
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0,
    1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5, 8.5, 9.5, 10.5,
    2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0,
    2.5, 3.5, 4.5, 5.5, 6.5, 7.5, 8.5, 9.5, 10.5, 11.5,

    // Some medium latency samples (11-15ms) - 15% of samples
    11.0, 12.0, 13.0, 14.0, 15.0, 12.5, 13.5, 14.5,

    // A few higher latency samples (16-19ms) - 5% of samples
    16.0, 18.0};

  for (size_t i = 0; i < latencies.size(); ++i) {
    simulateCommandEcho(static_cast<uint16_t>(i + 1), latencies[i]);
  }

  auto p95_duration = latency_tracker_->getP95Latency();
  ASSERT_GT(p95_duration.count(), 0) << "Should have p95 measurement";

  double p95_ms = std::chrono::duration<double, std::milli>(p95_duration).count();
  EXPECT_LE(p95_ms, 20.0)
    << "p95 latency (" << p95_ms << "ms) must be ≤ 20ms per performance requirement";
}

TEST_F(LatencyMeasurementTest, BoundaryConditionTesting)
{
  // Test at exactly 20ms boundary
  for (uint16_t seq = 1; seq <= 50; ++seq) {
    simulateCommandEcho(seq, 19.9);     // Just under 20ms
  }

  auto p95_duration = latency_tracker_->getP95Latency();
  ASSERT_GT(p95_duration.count(), 0) << "Should have p95 measurement";

  double p95_ms = std::chrono::duration<double, std::milli>(p95_duration).count();
  EXPECT_LE(p95_ms, 20.0) << "19.9ms constant should meet ≤20ms requirement";
  EXPECT_NEAR(p95_ms, 19.9, 0.5) << "p95 should be close to input value";
}

TEST_F(LatencyMeasurementTest, LatencySpikesHandling)
{
  // Test with occasional latency spikes but overall good performance
  std::vector<double> latencies;

  // 90% low latency (2-8ms)
  for (int i = 0; i < 45; ++i) {
    latencies.push_back(2.0 + (i % 6));     // 2-8ms range
  }

  // 10% spikes (but still within acceptable range)
  for (int i = 0; i < 5; ++i) {
    latencies.push_back(15.0 + i);     // 15-19ms spikes
  }

  // Shuffle to make it realistic
  std::random_device rd;
  std::mt19937 gen(rd());
  std::shuffle(latencies.begin(), latencies.end(), gen);

  for (size_t i = 0; i < latencies.size(); ++i) {
    simulateCommandEcho(static_cast<uint16_t>(i + 1), latencies[i]);
  }

  auto p95_duration = latency_tracker_->getP95Latency();
  ASSERT_GT(p95_duration.count(), 0) << "Should have p95 measurement";

  double p95_ms = std::chrono::duration<double, std::milli>(p95_duration).count();
  EXPECT_LE(p95_ms, 20.0) << "Should handle occasional spikes and still meet requirement";
}

TEST_F(LatencyMeasurementTest, WindowSizeBehavior)
{
  // Test that the sliding window correctly maintains only recent samples

  // Fill window with high latency samples
  for (uint16_t seq = 1; seq <= 50; ++seq) {
    simulateCommandEcho(seq, 18.0);     // High but acceptable latency
  }

  auto p95_high_duration = latency_tracker_->getP95Latency();
  ASSERT_GT(p95_high_duration.count(), 0);
  double p95_high_ms = std::chrono::duration<double, std::milli>(p95_high_duration).count();
  EXPECT_NEAR(p95_high_ms, 18.0, 1.0) << "Initial p95 should reflect high latency";

  // Add 50 more samples with low latency (should push out old samples)
  for (uint16_t seq = 51; seq <= 100; ++seq) {
    simulateCommandEcho(seq, 3.0);     // Low latency
  }

  auto p95_low_duration = latency_tracker_->getP95Latency();
  ASSERT_GT(p95_low_duration.count(), 0);
  double p95_low_ms = std::chrono::duration<double, std::milli>(p95_low_duration).count();
  EXPECT_NEAR(p95_low_ms, 3.0, 1.0) << "New p95 should reflect recent low latency";
  EXPECT_LT(p95_low_ms, p95_high_ms) << "p95 should improve with better recent samples";
}

TEST_F(LatencyMeasurementTest, MissedEchoHandling)
{
  // Test behavior when some commands don't get echo responses

  for (uint16_t seq = 1; seq <= 50; ++seq) {
    if (seq % 10 == 0) {
      // Skip echo for every 10th command (simulate missed echo)
      latency_tracker_->recordCommandSent(seq, base_time_ + std::chrono::milliseconds(seq * 20));
      // No corresponding recordEchoReceived call
    } else {
      // Normal command-echo cycle
      simulateCommandEcho(seq, 5.0);
    }
  }

  auto p95_duration = latency_tracker_->getP95Latency();
  // Should still have measurement from the successful echo cycles
  if (p95_duration.count() > 0) {
    double p95_ms = std::chrono::duration<double, std::milli>(p95_duration).count();
    EXPECT_LE(p95_ms, 20.0) << "Should handle missed echoes gracefully";
    EXPECT_NEAR(p95_ms, 5.0, 1.0) << "p95 should reflect successful measurements";
  }
}

TEST_F(LatencyMeasurementTest, LatencyTrendAnalysis)
{
  // Test gradually increasing latency to verify tracker responds to trends

  for (uint16_t seq = 1; seq <= 50; ++seq) {
    double latency = 2.0 + (seq * 0.2);     // Gradually increase from 2ms to 12ms
    simulateCommandEcho(seq, latency);
  }

  auto p95_duration = latency_tracker_->getP95Latency();
  ASSERT_GT(p95_duration.count(), 0) << "Should track latency trends";

  double p95_ms = std::chrono::duration<double, std::milli>(p95_duration).count();
  EXPECT_LE(p95_ms, 20.0) << "Should meet requirement even with increasing trend";
  EXPECT_GT(p95_ms, 8.0) << "p95 should reflect the higher recent latencies";
}
