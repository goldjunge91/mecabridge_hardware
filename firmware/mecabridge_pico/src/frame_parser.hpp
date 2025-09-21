#ifndef DRIVE_ARDUINO__FIRMWARE__MECABRIDGE_PICO__FRAME_PARSER_HPP_
#define DRIVE_ARDUINO__FIRMWARE__MECABRIDGE_PICO__FRAME_PARSER_HPP_


#pragma once

#include <cstdint>
#include <cstddef>

namespace mecabridge
{

// Protocol constants from data-model.md
constexpr uint8_t START_BYTE = 0xAA;
constexpr uint8_t PROTOCOL_VERSION = 1;

// Frame IDs
enum class FrameId : uint8_t
{
  COMMAND = 0x01,
  STATE = 0x02,
  HEARTBEAT = 0x03,
  VERSION_INFO = 0x04,
  ERROR = 0x05
};

// Error codes
enum class ErrorCode : uint8_t
{
  OK = 0,
  CRC_FAIL = 1,
  MALFORMED_FRAME = 2,
  UNKNOWN_FRAME_ID = 3,
  VERSION_MISMATCH = 4,
  WATCHDOG_TIMEOUT = 5
};

// Flag bits
namespace Flags
{
constexpr uint16_t WATCHDOG_TRIGGERED = (1 << 0);
constexpr uint16_t POSITIONAL_SERVO_LIMIT_HIT = (1 << 1);
constexpr uint16_t CRC_ERROR_SINCE_LAST = (1 << 2);
constexpr uint16_t MALFORMED_ERROR_SINCE_LAST = (1 << 3);
constexpr uint16_t ENCODER_STALE = (1 << 4);
}

// Command frame payload (36 bytes)
struct CommandFramePayload
{
  float wheel_vel_rad_s[4];        // 16 bytes: FL, FR, RL, RR
  float servo_pos_rad;             // 4 bytes
  float servo_cont_vel_norm;       // 4 bytes
  float esc_norm[2];               // 8 bytes
  uint16_t seq;                    // 2 bytes
  uint8_t protocol_version;        // 1 byte
  uint8_t reserved_flags;          // 1 byte
} __attribute__((packed));

// State frame payload (40 bytes)
struct StateFramePayload
{
  uint32_t encoder_counts[4];      // 16 bytes
  uint16_t dt_ms;                  // 2 bytes
  float servo_pos_rad;             // 4 bytes
  float servo_cont_vel_norm;       // 4 bytes
  float esc_norm[2];               // 8 bytes
  uint16_t seq_echo;               // 2 bytes
  uint16_t flags;                  // 2 bytes
  uint8_t error_code;              // 1 byte
  uint8_t protocol_version;        // 1 byte
} __attribute__((packed));

// Frame structure
struct Frame
{
  uint8_t start_byte;
  uint8_t frame_id;
  uint8_t len;
  union {
    CommandFramePayload command;
    StateFramePayload state;
    uint8_t raw_payload[64];      // Max payload size
  };
  uint16_t crc16;
} __attribute__((packed));

// Parser state
enum class ParseState
{
  WAITING_START,
  WAITING_FRAME_ID,
  WAITING_LENGTH,
  READING_PAYLOAD,
  WAITING_CRC_HIGH,
  WAITING_CRC_LOW,
  FRAME_COMPLETE
};

// Parser result
enum class ParseResult
{
  NEED_MORE_DATA,
  FRAME_COMPLETE,
  FRAME_ERROR,
  CRC_MISMATCH
};

// CRC16/CCITT-FALSE calculator
class CRC16
{
public:
  CRC16()
  : crc_(0xFFFF) {}

  void reset() {crc_ = 0xFFFF;}
  void update(uint8_t byte);
  void update(const uint8_t * data, size_t len);
  uint16_t finalize() const {return crc_;}

  // Static function for one-shot calculation
  static uint16_t calculate(const uint8_t * data, size_t len);

private:
  uint16_t crc_;
};

// Frame parser
class FrameParser
{
public:
  FrameParser();

  // Parse incoming bytes, returns result and updates frame if complete
  ParseResult parseByte(uint8_t byte, Frame & frame);

  // Reset parser state
  void reset();

  // Get current error state
  ErrorCode getLastError() const {return last_error_;}

private:
  ParseState state_;
  Frame current_frame_;
  size_t payload_bytes_read_;
  size_t expected_payload_len_;
  CRC16 crc_calculator_;
  ErrorCode last_error_;

  bool isValidFrameId(uint8_t frame_id);
  bool isValidPayloadLength(uint8_t frame_id, uint8_t len);
};

// Frame encoder
class FrameEncoder
{
public:
  // Encode command frame
  static size_t encodeCommand(
    const CommandFramePayload & payload, uint8_t * buffer,
    size_t buffer_size);

  // Encode state frame
  static size_t encodeState(
    const StateFramePayload & payload, uint8_t * buffer,
    size_t buffer_size);

private:
  static size_t encodeFrame(
    FrameId frame_id, const void * payload, size_t payload_size,
    uint8_t * buffer, size_t buffer_size);
};

} // namespace mecabridge
#endif  // DRIVE_ARDUINO__FIRMWARE__MECABRIDGE_PICO__FRAME_PARSER_HPP_
