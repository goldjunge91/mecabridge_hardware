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

#ifndef MECABRIDGE_HARDWARE__FEATURE_CONFIG_HPP_
#define MECABRIDGE_HARDWARE__FEATURE_CONFIG_HPP_

#ifndef MECABRIDGE_ENABLE_ESCS
#define MECABRIDGE_ENABLE_ESCS 1
#endif

#ifndef MECABRIDGE_ENABLE_DIAGNOSTICS
#define MECABRIDGE_ENABLE_DIAGNOSTICS 1
#endif

// Derived configuration
#if MECABRIDGE_ENABLE_SERVOS
constexpr bool kEnableServos = true;
constexpr int kServoInterfaces = 2;  // positional + continuous
#else
constexpr bool kEnableServos = false;
constexpr int kServoInterfaces = 0;
#endif

#if MECABRIDGE_ENABLE_ESCS
constexpr bool kEnableESCs = true;
constexpr int kESCInterfaces = 2;    // 2 ESC channels
#else
constexpr bool kEnableESCs = false;
constexpr int kESCInterfaces = 0;
#endif

#if MECABRIDGE_ENABLE_DIAGNOSTICS
constexpr bool kEnableDiagnostics = true;
#else
constexpr bool kEnableDiagnostics = false;
#endif

// Total interface counts
constexpr int kWheelInterfaces = 4;  // Always enabled
constexpr int kTotalInterfaces = kWheelInterfaces + kServoInterfaces + kESCInterfaces;

// Interface index mappings
constexpr int kWheelStartIndex = 0;
constexpr int kServoStartIndex = kWheelStartIndex + kWheelInterfaces;
constexpr int kESCStartIndex = kServoStartIndex + kServoInterfaces;
#endif  // MECABRIDGE_HARDWARE__FEATURE_CONFIG_HPP_
