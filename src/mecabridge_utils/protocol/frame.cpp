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

#include "frame.hpp"
#include "crc16.hpp"

#include <algorithm>

#include <cstring>
#include <cmath>

namespace mecabridge
{
namespace protocol
{

ErrorCode encodeCommand(
  const CommandFramePayload & command, uint8_t * buffer, size_t buffer_size,
  size_t & bytes_written)
{
  // Frame structure: START_BYTE + FRAME_ID + LEN + PAYLOAD + CRC16
  // Total size: 1 + 1 + 1 + 36 + 2 = 41 bytes
  const size_t total_frame_size = 41;

  if (buffer == nullptr || buffer_size < total_frame_size) {
    bytes_written = 0;
    return ErrorCode::MALFORMED_FRAME;
  }

  CommandFramePayload command_copy = command;
  command_copy.protocol_version = PROTOCOL_VERSION;

  size_t offset = 0;

  // START_BYTE
  buffer[offset++] = START_BYTE;

  // FRAME_ID
  buffer[offset++] = static_cast<uint8_t>(FrameId::COMMAND);

  // LEN (payload size)
  buffer[offset++] = sizeof(CommandFramePayload);

  // PAYLOAD - copy the struct directly (little-endian)
  std::memcpy(&buffer[offset], &command_copy, sizeof(CommandFramePayload));
  offset += sizeof(CommandFramePayload);

  // Calculate CRC over FRAME_ID + LEN + PAYLOAD (exclude START_BYTE and CRC bytes)
  uint16_t crc = crc16_ccitt_false(&buffer[1], offset - 1);

  // CRC16 (MSB first as per protocol)
  buffer[offset++] = static_cast<uint8_t>((crc >> 8) & 0xFF);    // MSB
  buffer[offset++] = static_cast<uint8_t>(crc & 0xFF);           // LSB

  bytes_written = offset;
  return ErrorCode::OK;
}

ErrorCode encodeState(
  const StateFramePayload & state, uint8_t * buffer, size_t buffer_size,
  size_t & bytes_written)
{
  // Frame structure: START_BYTE + FRAME_ID + LEN + PAYLOAD + CRC16
  // Total size: 1 + 1 + 1 + 40 + 2 = 45 bytes
  const size_t total_frame_size = 45;

  if (buffer == nullptr || buffer_size < total_frame_size) {
    bytes_written = 0;
    return ErrorCode::MALFORMED_FRAME;
  }

  StateFramePayload state_copy = state;
  state_copy.protocol_version = PROTOCOL_VERSION;

  size_t offset = 0;

  // START_BYTE
  buffer[offset++] = START_BYTE;

  // FRAME_ID
  buffer[offset++] = static_cast<uint8_t>(FrameId::STATE);

  // LEN (payload size)
  buffer[offset++] = sizeof(StateFramePayload);

  // PAYLOAD - copy the struct directly (little-endian)
  std::memcpy(&buffer[offset], &state_copy, sizeof(StateFramePayload));
  offset += sizeof(StateFramePayload);

  // Calculate CRC over FRAME_ID + LEN + PAYLOAD (exclude START_BYTE and CRC bytes)
  uint16_t crc = crc16_ccitt_false(&buffer[1], offset - 1);

  // CRC16 (MSB first as per protocol)
  buffer[offset++] = static_cast<uint8_t>((crc >> 8) & 0xFF);    // MSB
  buffer[offset++] = static_cast<uint8_t>(crc & 0xFF);           // LSB

  bytes_written = offset;
  return ErrorCode::OK;
}

ParseResult tryParseFrame(ByteSpan input, ParsedFrame & result)
{
  // Initialize result
  result.result = ParseResult::UNKNOWN_ERROR;
  result.payload_data = nullptr;
  result.payload_len = 0;

  // Minimum frame size: START_BYTE + FRAME_ID + LEN + CRC16 = 5 bytes
  if (input.size < 5) {
    result.result = ParseResult::INSUFFICIENT_DATA;
    return ParseResult::INSUFFICIENT_DATA;
  }

  // Check START_BYTE
  if (input.data[0] != START_BYTE) {
    result.result = ParseResult::INVALID_START_BYTE;
    return ParseResult::INVALID_START_BYTE;
  }

  // Extract FRAME_ID
  uint8_t frame_id_byte = input.data[1];
  if (frame_id_byte != static_cast<uint8_t>(FrameId::COMMAND) &&
    frame_id_byte != static_cast<uint8_t>(FrameId::STATE) &&
    frame_id_byte != static_cast<uint8_t>(FrameId::HEARTBEAT) &&
    frame_id_byte != static_cast<uint8_t>(FrameId::VERSION_INFO) &&
    frame_id_byte != static_cast<uint8_t>(FrameId::ERROR))
  {
    result.result = ParseResult::INVALID_FRAME_ID;
    return ParseResult::INVALID_FRAME_ID;
  }
  result.frame_id = static_cast<FrameId>(frame_id_byte);

  // Extract LEN
  uint8_t payload_len = input.data[2];

  // Validate frame length bounds
  if (payload_len > 64) {    // Reasonable upper bound
    result.result = ParseResult::INVALID_LENGTH;
    return ParseResult::INVALID_LENGTH;
  }

  // Check if we have enough data for complete frame
  size_t expected_total_size = 3 + payload_len + 2;    // header + payload + crc
  if (input.size < expected_total_size) {
    result.result = ParseResult::INSUFFICIENT_DATA;
    return ParseResult::INSUFFICIENT_DATA;
  }

  // Extract payload pointer
  result.payload_data = &input.data[3];
  result.payload_len = payload_len;

  // Verify CRC
  // CRC is calculated over FRAME_ID + LEN + PAYLOAD
  const uint8_t * crc_data = &input.data[1];   // Start from FRAME_ID
  size_t crc_data_len = 2 + payload_len;       // FRAME_ID + LEN + payload
  uint16_t calculated_crc = crc16_ccitt_false(crc_data, crc_data_len);

  // Extract received CRC (MSB first)
  size_t crc_offset = 3 + payload_len;
  uint16_t received_crc = (static_cast<uint16_t>(input.data[crc_offset]) << 8) |
    static_cast<uint16_t>(input.data[crc_offset + 1]);

  if (calculated_crc != received_crc) {
    result.result = ParseResult::CRC_MISMATCH;
    return ParseResult::CRC_MISMATCH;
  }

  // Version check for STATE frames: ensure firmware protocol_version matches
  if (result.frame_id == FrameId::STATE) {
    if (result.payload_len != sizeof(StateFramePayload)) {
      result.result = ParseResult::INVALID_LENGTH;
      return ParseResult::INVALID_LENGTH;
    }

    const auto * state = reinterpret_cast<const StateFramePayload *>(result.payload_data);
    if (state->protocol_version != PROTOCOL_VERSION) {
      result.result = ParseResult::VERSION_MISMATCH;
      return ParseResult::VERSION_MISMATCH;
    }
  }

  // Success
  result.result = ParseResult::SUCCESS;
  return ParseResult::SUCCESS;
}

bool verify_version(uint8_t firmware_version)
{
  return firmware_version == PROTOCOL_VERSION;
}

bool validateCommandPayload(CommandFramePayload & command)
{
  bool valid = true;

  // Check wheel velocities (reasonable bounds)
  for (int i = 0; i < 4; ++i) {
    if (std::isnan(command.wheel_vel_rad_s[i]) || std::isinf(command.wheel_vel_rad_s[i])) {
      valid = false;
    }
  }

  // Check servo position (should be finite)
  if (std::isnan(command.servo_pos_rad) || std::isinf(command.servo_pos_rad)) {
    valid = false;
  }

  // Check servo continuous velocity (normalized)
  if (std::isnan(command.servo_cont_vel_norm) || std::isinf(command.servo_cont_vel_norm)) {
    valid = false;
  }

  // Check ESC values
  for (int i = 0; i < 2; ++i) {
    if (std::isnan(command.esc_norm[i]) || std::isinf(command.esc_norm[i])) {
      valid = false;
    }
  }

  return valid;
}

void clampCommandValues(CommandFramePayload & command, uint16_t & flags_set)
{
  flags_set = 0;

  // Clamp servo continuous velocity to [-1.0, 1.0]
  if (command.servo_cont_vel_norm > 1.0f) {
    command.servo_cont_vel_norm = 1.0f;
    flags_set |= static_cast<uint16_t>(StatusFlags::POSITIONAL_SERVO_LIMIT_HIT);
  } else if (command.servo_cont_vel_norm < -1.0f) {
    command.servo_cont_vel_norm = -1.0f;
    flags_set |= static_cast<uint16_t>(StatusFlags::POSITIONAL_SERVO_LIMIT_HIT);
  }

  // Clamp ESC values to [-1.0, 1.0]
  for (int i = 0; i < 2; ++i) {
    if (command.esc_norm[i] > 1.0f) {
      command.esc_norm[i] = 1.0f;
      flags_set |= static_cast<uint16_t>(StatusFlags::POSITIONAL_SERVO_LIMIT_HIT);
    } else if (command.esc_norm[i] < -1.0f) {
      command.esc_norm[i] = -1.0f;
      flags_set |= static_cast<uint16_t>(StatusFlags::POSITIONAL_SERVO_LIMIT_HIT);
    }
  }

  // Note: Wheel velocities and servo position limits depend on configuration
  // For now, we don't clamp them as limits are application-specific
  // In a full implementation, these would be configured via YAML config
}

} // namespace protocol
} // namespace mecabridge
