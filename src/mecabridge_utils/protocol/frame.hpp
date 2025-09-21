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

#include <cstdint>
#include <cstddef>

// Include CRC helper used by tests/helpers
#include "crc16.hpp"

namespace mecabridge
{
namespace protocol
{

// Protocol constants
static constexpr uint8_t START_BYTE = 0xAA;
static constexpr uint8_t PROTOCOL_VERSION = 1;

// Frame IDs
enum class FrameId : uint8_t
{
  COMMAND = 0x01,
  STATE = 0x02,
  HEARTBEAT = 0x03,         // Optional
  VERSION_INFO = 0x04,      // Optional
  ERROR = 0x05              // Optional
};

// Error codes
enum class ErrorCode : uint8_t
{
  OK = 0,
  CRC_FAIL = 1,
  MALFORMED_FRAME = 2,
  UNKNOWN_FRAME_ID = 3,
  VERSION_MISMATCH = 4,
  WATCHDOG_TIMEOUT = 5,
  NOT_IMPLEMENTED = 255
};

// Status flags bitfield
enum class StatusFlags : uint16_t
{
  WATCHDOG_TRIGGERED = 0x0001,
  POSITIONAL_SERVO_LIMIT_HIT = 0x0002,
  CRC_ERROR_SINCE_LAST = 0x0004,
  MALFORMED_ERROR_SINCE_LAST = 0x0008,
  ENCODER_STALE = 0x0010,
  RESERVED_FUTURE = 0x0020
};

// Safety flags for limit clamping
enum class SafetyFlags : uint16_t
{
  SERVO_POSITION_LIMIT = 0x0001,
  SERVO_VELOCITY_LIMIT = 0x0002,
  ESC_LIMIT = 0x0004,
  WHEEL_VELOCITY_LIMIT = 0x0008,
  RESERVED_1 = 0x0010,
  RESERVED_2 = 0x0020,
  RESERVED_3 = 0x0040,
  RESERVED_4 = 0x0080
};

// Command frame payload structure
struct CommandFramePayload
{
  float wheel_vel_rad_s[4];         // FL, FR, RL, RR order
  float servo_pos_rad;              // Positional servo target
  float servo_cont_vel_norm;        // Continuous servo velocity (-1..1)
  float esc_norm[2];                // ESC normalized [-1..1]
  uint16_t seq;                     // Sequence number
  uint8_t protocol_version;         // Protocol version
  uint8_t reserved_flags;           // Future use (0)
} __attribute__((packed));

static_assert(sizeof(CommandFramePayload) == 36, "CommandFramePayload must be 36 bytes");

// State frame payload structure
struct StateFramePayload
{
  uint32_t encoder_counts[4];       // Raw absolute counters
  uint16_t dt_ms;                   // Elapsed ms since previous state frame
  float servo_pos_rad;              // Current servo position
  float servo_cont_vel_norm;        // Echo last commanded
  float esc_norm[2];                // Echo last commanded normalized
  uint16_t seq_echo;                // Last accepted COMMAND seq
  uint16_t flags;                   // Status flags bitfield
  uint8_t error_code;               // Recent error cause
  uint8_t protocol_version;         // Protocol version of the firmware
} __attribute__((packed));

static_assert(sizeof(StateFramePayload) == 40, "StateFramePayload must be 40 bytes");

// Generic frame structure for parsing
struct Frame
{
  uint8_t start_byte;
  FrameId frame_id;
  uint8_t len;
  uint8_t payload[64];    // Max payload size
  uint16_t crc16;
};

// Parse result
enum class ParseResult
{
  SUCCESS,
  INSUFFICIENT_DATA,
  INVALID_START_BYTE,
  INVALID_FRAME_ID,
  INVALID_LENGTH,
  CRC_MISMATCH,
  VERSION_MISMATCH,
  UNKNOWN_ERROR
};

// Parsed frame information
struct ParsedFrame
{
  FrameId frame_id;
  uint8_t payload_len;
  const uint8_t * payload_data;
  ParseResult result;
};

// Byte span for input data
struct ByteSpan
{
  const uint8_t * data;
  size_t size;

  ByteSpan(const uint8_t * d, size_t s)
  : data(d), size(s) {}
};

// Function declarations
ErrorCode encodeCommand(
  const CommandFramePayload & command, uint8_t * buffer, size_t buffer_size,
  size_t & bytes_written);
ErrorCode encodeState(
  const StateFramePayload & state, uint8_t * buffer, size_t buffer_size,
  size_t & bytes_written);
ParseResult tryParseFrame(ByteSpan input, ParsedFrame & result);
bool verify_version(uint8_t firmware_version);

// Helper functions
bool validateCommandPayload(CommandFramePayload & command);
void clampCommandValues(CommandFramePayload & command, uint16_t & flags_set);

} // namespace protocol
} // namespace mecabridge
