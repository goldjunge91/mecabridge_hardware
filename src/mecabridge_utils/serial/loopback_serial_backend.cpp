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

#include "loopback_serial_backend.hpp"
#include "mecabridge_utils/protocol/frame.hpp"

#include <algorithm>

#include <cstring>

namespace mecabridge
{
namespace serial
{

LoopbackSerialBackend::LoopbackSerialBackend()
: is_open_(false)
{
}

bool LoopbackSerialBackend::open(const SerialOptions & opts)
{
  (void)opts; // Suppress unused parameter warning
  std::lock_guard<std::mutex> lock(mutex_);
  is_open_ = true;
  return true;
}

void LoopbackSerialBackend::close()
{
  std::lock_guard<std::mutex> lock(mutex_);
  is_open_ = false;
  // Clear queues
  std::queue<uint8_t> empty;
  input_queue_.swap(empty);
  last_command_frame_.clear();
}

bool LoopbackSerialBackend::is_open() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return is_open_;
}

int LoopbackSerialBackend::read(uint8_t * buf, size_t len)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_open_) {
    return -1;
  }

  size_t bytes_read = 0;
  while (bytes_read < len && !input_queue_.empty()) {
    buf[bytes_read++] = input_queue_.front();
    input_queue_.pop();
  }

  return static_cast<int>(bytes_read);
}

int LoopbackSerialBackend::write(const uint8_t * buf, size_t len)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_open_) {
    return -1;
  }

  // Store the last command frame
  last_command_frame_.assign(buf, buf + len);

  // Process command frame and generate loopback state frame
  processCommandFrame(buf, len);

  return static_cast<int>(len);
}

void LoopbackSerialBackend::injectStateFrame(const uint8_t * frame_data, size_t frame_size)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (size_t i = 0; i < frame_size; ++i) {
    input_queue_.push(frame_data[i]);
  }
}

void LoopbackSerialBackend::clearInput()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::queue<uint8_t> empty;
  input_queue_.swap(empty);
}

std::vector<uint8_t> LoopbackSerialBackend::getLastCommandFrame() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_command_frame_;
}

void LoopbackSerialBackend::processCommandFrame(const uint8_t * data, size_t size)
{
  // Try to parse the command frame
  mecabridge::protocol::ByteSpan input(data, size);
  mecabridge::protocol::ParsedFrame parsed_frame;

  auto parse_result = mecabridge::protocol::tryParseFrame(input, parsed_frame);
  if (parse_result != mecabridge::protocol::ParseResult::SUCCESS) {
    return; // Invalid frame, don't generate response
  }

  if (parsed_frame.frame_id != mecabridge::protocol::FrameId::COMMAND) {
    return; // Not a command frame
  }

  // Extract command payload
  if (parsed_frame.payload_len != sizeof(mecabridge::protocol::CommandFramePayload)) {
    return; // Invalid payload size
  }

  const auto * cmd_payload =
    reinterpret_cast<const mecabridge::protocol::CommandFramePayload *>(parsed_frame.
    payload_data);

  // Create a corresponding state frame
  mecabridge::protocol::StateFramePayload state_payload = {};

  // Echo the sequence number
  state_payload.seq_echo = cmd_payload->seq;

  // Protocol version will be set during encoding (not copied from command)

  // Simulate encoder counts (convert wheel velocities to position deltas)
  state_payload.dt_ms = 20; // Simulate 20ms update rate
  for (int i = 0; i < 4; ++i) {
    // Simulate encoder counts based on wheel velocity
    // This is a simplified simulation - real firmware would integrate velocity
    float velocity = cmd_payload->wheel_vel_rad_s[i];
    state_payload.encoder_counts[i] = static_cast<uint32_t>(velocity * 100.0f); // Arbitrary scaling
  }

  // Echo servo and ESC commands
  state_payload.servo_pos_rad = cmd_payload->servo_pos_rad;
  state_payload.servo_cont_vel_norm = cmd_payload->servo_cont_vel_norm;
  state_payload.esc_norm[0] = cmd_payload->esc_norm[0];
  state_payload.esc_norm[1] = cmd_payload->esc_norm[1];

  // Set status flags (no errors in simulation)
  state_payload.flags = 0;
  state_payload.error_code = 0;

  // Encode the state frame
  uint8_t state_frame_buffer[128];
  size_t bytes_written = 0;
  auto encode_result =
    mecabridge::protocol::encodeState(
    state_payload, state_frame_buffer,
    sizeof(state_frame_buffer), bytes_written);

  if (encode_result == mecabridge::protocol::ErrorCode::OK) {
    // Add the state frame to input queue for reading
    for (size_t i = 0; i < bytes_written; ++i) {
      input_queue_.push(state_frame_buffer[i]);
    }
  }
}

} // namespace serial
} // namespace mecabridge
