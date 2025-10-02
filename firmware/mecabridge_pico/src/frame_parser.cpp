#include "frame_parser.hpp"

#include <cstring>

namespace mecabridge
{

// CRC16/CCITT-FALSE implementation
void CRC16::update(uint8_t byte)
{
  crc_ ^= (uint16_t)byte << 8;
  for (int i = 0; i < 8; ++i) {
    if (crc_ & 0x8000) {
      crc_ = (crc_ << 1) ^ 0x1021;
    } else {
      crc_ <<= 1;
    }
  }
}

void CRC16::update(const uint8_t * data, size_t len)
{
  for (size_t i = 0; i < len; ++i) {
    update(data[i]);
  }
}

uint16_t CRC16::calculate(const uint8_t * data, size_t len)
{
  CRC16 calculator;
  calculator.update(data, len);
  return calculator.finalize();
}

// FrameParser implementation
FrameParser::FrameParser()
: state_(ParseState::WAITING_START)
  , payload_bytes_read_(0)
  , expected_payload_len_(0)
  , last_error_(ErrorCode::OK)
{
  memset(&current_frame_, 0, sizeof(current_frame_));
}

void FrameParser::reset()
{
  state_ = ParseState::WAITING_START;
  payload_bytes_read_ = 0;
  expected_payload_len_ = 0;
  last_error_ = ErrorCode::OK;
  crc_calculator_.reset();
  memset(&current_frame_, 0, sizeof(current_frame_));
}

bool FrameParser::isValidFrameId(uint8_t frame_id)
{
  return frame_id == static_cast<uint8_t>(FrameId::COMMAND) ||
         frame_id == static_cast<uint8_t>(FrameId::STATE) ||
         frame_id == static_cast<uint8_t>(FrameId::HEARTBEAT) ||
         frame_id == static_cast<uint8_t>(FrameId::VERSION_INFO) ||
         frame_id == static_cast<uint8_t>(FrameId::ERROR);
}

bool FrameParser::isValidPayloadLength(uint8_t frame_id, uint8_t len)
{
  switch (static_cast<FrameId>(frame_id)) {
    case FrameId::COMMAND:
      return len == sizeof(CommandFramePayload);
    case FrameId::STATE:
      return len == sizeof(StateFramePayload);
    case FrameId::HEARTBEAT:
      return len == 0;       // Heartbeat has no payload
    case FrameId::VERSION_INFO:
      return len <= 16;       // Flexible version info
    case FrameId::ERROR:
      return len <= 32;       // Flexible error info
    default:
      return false;
  }
}

ParseResult FrameParser::parseByte(uint8_t byte, Frame & frame)
{
  switch (state_) {
    case ParseState::WAITING_START:
      if (byte == START_BYTE) {
        reset();
        current_frame_.start_byte = byte;
        state_ = ParseState::WAITING_FRAME_ID;
      }
      return ParseResult::NEED_MORE_DATA;

    case ParseState::WAITING_FRAME_ID:
      current_frame_.frame_id = byte;
      if (!isValidFrameId(byte)) {
        last_error_ = ErrorCode::UNKNOWN_FRAME_ID;
        reset();
        return ParseResult::FRAME_ERROR;
      }
      crc_calculator_.update(byte);
      state_ = ParseState::WAITING_LENGTH;
      return ParseResult::NEED_MORE_DATA;

    case ParseState::WAITING_LENGTH:
      current_frame_.len = byte;
      if (!isValidPayloadLength(current_frame_.frame_id, byte)) {
        last_error_ = ErrorCode::MALFORMED_FRAME;
        reset();
        return ParseResult::FRAME_ERROR;
      }
      expected_payload_len_ = byte;
      crc_calculator_.update(byte);

      if (expected_payload_len_ == 0) {
        state_ = ParseState::WAITING_CRC_HIGH;
      } else {
        state_ = ParseState::READING_PAYLOAD;
      }
      return ParseResult::NEED_MORE_DATA;

    case ParseState::READING_PAYLOAD:
      current_frame_.raw_payload[payload_bytes_read_] = byte;
      crc_calculator_.update(byte);
      payload_bytes_read_++;

      if (payload_bytes_read_ >= expected_payload_len_) {
        state_ = ParseState::WAITING_CRC_HIGH;
      }
      return ParseResult::NEED_MORE_DATA;

    case ParseState::WAITING_CRC_HIGH:
      current_frame_.crc16 = (uint16_t)byte << 8;
      state_ = ParseState::WAITING_CRC_LOW;
      return ParseResult::NEED_MORE_DATA;

    case ParseState::WAITING_CRC_LOW: {
      current_frame_.crc16 |= byte;

      // Verify CRC
      uint16_t calculated_crc = crc_calculator_.finalize();
      if (calculated_crc != current_frame_.crc16) {
        last_error_ = ErrorCode::CRC_FAIL;
        reset();
        return ParseResult::CRC_MISMATCH;
      }

      // Frame is complete and valid
      frame = current_frame_;
      reset();
      return ParseResult::FRAME_COMPLETE;
    }

    default:
      reset();
      return ParseResult::FRAME_ERROR;
  }
}

// FrameEncoder implementation
size_t FrameEncoder::encodeCommand(
  const CommandFramePayload & payload, uint8_t * buffer,
  size_t buffer_size)
{
  return encodeFrame(FrameId::COMMAND, &payload, sizeof(payload), buffer, buffer_size);
}

size_t FrameEncoder::encodeState(
  const StateFramePayload & payload, uint8_t * buffer,
  size_t buffer_size)
{
  return encodeFrame(FrameId::STATE, &payload, sizeof(payload), buffer, buffer_size);
}

size_t FrameEncoder::encodeFrame(
  FrameId frame_id, const void * payload, size_t payload_size,
  uint8_t * buffer, size_t buffer_size)
{
  // Calculate required buffer size: start_byte(1) + frame_id(1) + len(1) + payload + crc(2)
  size_t required_size = 3 + payload_size + 2;

  if (buffer_size < required_size) {
    return 0;     // Buffer too small
  }

  size_t offset = 0;

  // Start byte
  buffer[offset++] = START_BYTE;

  // Frame ID
  buffer[offset++] = static_cast<uint8_t>(frame_id);

  // Length
  buffer[offset++] = static_cast<uint8_t>(payload_size);

  // Payload
  if (payload_size > 0) {
    memcpy(&buffer[offset], payload, payload_size);
    offset += payload_size;
  }

  // Calculate CRC over frame_id + len + payload
  uint16_t crc = CRC16::calculate(&buffer[1], offset - 1);

  // CRC (MSB first)
  buffer[offset++] = (crc >> 8) & 0xFF;
  buffer[offset++] = crc & 0xFF;

  return offset;
}

} // namespace mecabridge
