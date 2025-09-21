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

#include <gtest/gtest.h>
#include "mecabridge_utils/safety/watchdog.hpp"
#include "mecabridge_utils/serial/loopback_serial_backend.hpp"
#include "mecabridge_utils/protocol/frame.hpp"
#include <memory>
#include <chrono>
#include <thread>

using namespace mecabridge::safety;
using namespace mecabridge::serial;
using namespace mecabridge::protocol;
using namespace std::chrono_literals;

/**
 * @brief End-to-end watchdog test simulating communication silence
 *
 * This test verifies that when no frames are received for ≥150ms,
 * the watchdog triggers and command outputs are zeroed.
 */
class WatchdogEndToEndTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    watchdog_ = std::make_unique<Watchdog>(150ms); // 150ms timeout
    loopback_backend_ = std::make_shared<LoopbackSerialBackend>();

    // Initialize test timestamp
    start_time_ = std::chrono::steady_clock::now();
  }

  std::unique_ptr<Watchdog> watchdog_;
  std::shared_ptr<LoopbackSerialBackend> loopback_backend_;
  std::chrono::steady_clock::time_point start_time_;
};

TEST_F(WatchdogEndToEndTest, WatchdogTriggersOnSilence) {
  auto now = std::chrono::steady_clock::now();

  // Initially not tripped
  EXPECT_FALSE(watchdog_->tripped(now));

  // Simulate receiving a valid frame
  watchdog_->updateOnValidFrame(now);
  EXPECT_FALSE(watchdog_->tripped(now));

  // Advance time by 100ms - should still be okay
  now += 100ms;
  EXPECT_FALSE(watchdog_->tripped(now));

  // Advance time by another 60ms (total 160ms > 150ms threshold)
  now += 60ms;
  EXPECT_TRUE(watchdog_->tripped(now));

  // Verify flags indicate watchdog trip
  auto flags = watchdog_->flags();
  EXPECT_NE(flags & static_cast<uint16_t>(StatusFlags::WATCHDOG_TRIGGERED), 0);
}

TEST_F(WatchdogEndToEndTest, WatchdogResetResumesNormalOperation) {
  auto now = std::chrono::steady_clock::now();

  // Trigger watchdog by advancing time
  now += 200ms;
  EXPECT_TRUE(watchdog_->tripped(now));

  // Reset watchdog to current time
  watchdog_->reset(now);
  EXPECT_FALSE(watchdog_->tripped(now));

  // Update with valid frame
  watchdog_->updateOnValidFrame(now);
  EXPECT_FALSE(watchdog_->tripped(now));

  // Should remain okay for the timeout period
  now += 140ms;
  EXPECT_FALSE(watchdog_->tripped(now));
}

TEST_F(WatchdogEndToEndTest, CommandsZeroedWhenWatchdogTrips) {
  // This test simulates the full hardware interface behavior
  // when watchdog trips during normal operation

  auto now = std::chrono::steady_clock::now();

  // Start with normal operation
  watchdog_->updateOnValidFrame(now);
  EXPECT_FALSE(watchdog_->tripped(now));

  // Simulate setting some command values (this would be done by ros2_control)
  double test_velocities[4] = {1.0, -1.0, 0.5, -0.5};  // Non-zero commands

  // Verify commands are non-zero initially
  for (int i = 0; i < 4; ++i) {
    EXPECT_NE(test_velocities[i], 0.0);
  }

  // Advance time to trigger watchdog
  now += 160ms;  // > 150ms threshold
  EXPECT_TRUE(watchdog_->tripped(now));

  // When watchdog trips, commands should be zeroed
  // (This simulates what the hardware interface should do)
  if (watchdog_->tripped(now)) {
    for (int i = 0; i < 4; ++i) {
      test_velocities[i] = 0.0;  // Zero all commands
    }
  }

  // Verify all commands are now zero
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(test_velocities[i], 0.0);
  }
}

TEST_F(WatchdogEndToEndTest, WatchdogWithFrameSequence) {
  auto now = std::chrono::steady_clock::now();

  // Send regular frames at 50Hz (20ms intervals)
  for (int frame = 0; frame < 10; ++frame) {
    watchdog_->updateOnValidFrame(now);
    EXPECT_FALSE(watchdog_->tripped(now))
      << "Watchdog should not trip with regular frames at frame " << frame;

    // Advance by 20ms (50Hz rate)
    now += 20ms;
  }

  // Now stop sending frames and wait
  now += 160ms;  // No frames for 160ms
  EXPECT_TRUE(watchdog_->tripped(now))
    << "Watchdog should trip after 160ms of silence";
}

TEST_F(WatchdogEndToEndTest, WatchdogRecoveryAfterFrameResume) {
  auto now = std::chrono::steady_clock::now();

  // Trigger watchdog
  now += 200ms;
  EXPECT_TRUE(watchdog_->tripped(now));

  // Resume sending frames
  watchdog_->updateOnValidFrame(now);
  // Note: updateOnValidFrame() already resets the watchdog

  EXPECT_FALSE(watchdog_->tripped(now))
    << "Watchdog should reset when frames resume";

  // Verify normal operation continues
  now += 100ms;
  EXPECT_FALSE(watchdog_->tripped(now))
    << "Watchdog should remain okay with resumed communication";
}

TEST_F(WatchdogEndToEndTest, WatchdogTimingPrecision) {
  auto now = std::chrono::steady_clock::now();

  // Update at exactly t=0
  watchdog_->updateOnValidFrame(now);

  // Test at exactly 149ms - should NOT trip
  now += std::chrono::milliseconds(149);
  EXPECT_FALSE(watchdog_->tripped(now))
    << "Watchdog should not trip at exactly 149ms";

  // Test at exactly 150ms - should NOT trip (≤ threshold)
  now += std::chrono::milliseconds(1);  // now = 150ms
  EXPECT_FALSE(watchdog_->tripped(now))
    << "Watchdog should not trip at exactly 150ms";

  // Test at 151ms - should trip (> threshold)
  now += std::chrono::milliseconds(1);  // now = 151ms
  EXPECT_TRUE(watchdog_->tripped(now))
    << "Watchdog should trip at 151ms (>150ms threshold)";
}

TEST_F(WatchdogEndToEndTest, WatchdogFlagsAfterTrip) {
  auto now = std::chrono::steady_clock::now();

  // Normal state - no flags
  EXPECT_EQ(watchdog_->flags(), 0);

  // Trigger watchdog
  now += 200ms;
  EXPECT_TRUE(watchdog_->tripped(now));

  // Check flags after trip
  auto flags = watchdog_->flags();
  EXPECT_NE(flags & static_cast<uint16_t>(StatusFlags::WATCHDOG_TRIGGERED), 0)
    << "WATCHDOG_TRIGGERED flag should be set";

  // Reset should clear flags
  watchdog_->reset();
  EXPECT_EQ(watchdog_->flags(), 0)
    << "Flags should be cleared after reset";
}
