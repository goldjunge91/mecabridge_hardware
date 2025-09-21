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

#include "mecabridge_hardware/mecabridge_utils/safety/watchdog.hpp"
using mecabridge::safety::Watchdog;
using namespace std::chrono_literals;

#include <thread>

#include <gtest/gtest.h>


TEST(WatchdogTest, TripsAfterTimeout) {
  // Use a shorter timeout for testing purposes
  Watchdog watchdog(std::chrono::milliseconds(50));
  auto start_time = std::chrono::steady_clock::now();

  // Initially, it should not be tripped
  ASSERT_FALSE(watchdog.tripped(start_time));

  // Simulate time passing just under the timeout
  auto slightly_later = start_time + std::chrono::milliseconds(40);
  ASSERT_FALSE(watchdog.tripped(slightly_later));

  // Simulate time passing just over the timeout
  auto much_later = start_time + std::chrono::milliseconds(60);
  ASSERT_TRUE(watchdog.tripped(much_later));
  ASSERT_NE(watchdog.flags(), 0);
}

TEST(WatchdogTest, UpdateResetsTimeout) {
  Watchdog watchdog(std::chrono::milliseconds(50));
  auto start_time = std::chrono::steady_clock::now();

  // It should not be tripped initially
  ASSERT_FALSE(watchdog.tripped(start_time));

  // Simulate some time passing
  auto time1 = start_time + std::chrono::milliseconds(30);
  ASSERT_FALSE(watchdog.tripped(time1));

  // Update the watchdog, resetting the timer
  watchdog.updateOnValidFrame(time1);

  // Simulate more time passing, but not enough to trip from time1
  auto time2 = time1 + std::chrono::milliseconds(30);
  ASSERT_FALSE(watchdog.tripped(time2));

  // Simulate enough time passing from time1 to trip
  auto time3 = time1 + std::chrono::milliseconds(60);
  ASSERT_TRUE(watchdog.tripped(time3));
}

TEST(WatchdogTest, ResetWorks) {
  Watchdog watchdog(std::chrono::milliseconds(50));
  auto start_time = std::chrono::steady_clock::now();

  // Trip the watchdog
  auto trip_time = start_time + std::chrono::milliseconds(60);
  ASSERT_TRUE(watchdog.tripped(trip_time));
  ASSERT_NE(watchdog.flags(), 0);

  // Reset the watchdog
  watchdog.reset();

  // It should not be tripped immediately after reset
  ASSERT_FALSE(watchdog.tripped(std::chrono::steady_clock::now()));
  ASSERT_EQ(watchdog.flags(), 0);
}
