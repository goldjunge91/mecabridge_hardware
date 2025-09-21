#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "hardware/timer.h"
#include "frame_parser.hpp"
#include "actuators.hpp"

#include <stdio.h>

#include <cstring>


// Watchdog configuration
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 150;
constexpr uint32_t STATE_FRAME_INTERVAL_MS = 20;  // 50Hz state frame rate

// Global state
FrameParser parser;
ActuatorController actuators;
uint64_t last_valid_command_time = 0;
uint64_t last_state_frame_time = 0;
uint16_t current_seq_echo = 0;
uint16_t current_flags = 0;
ErrorCode current_error_code = ErrorCode::OK;
bool watchdog_tripped = false;

// Statistics for encoder timing simulation
uint32_t fake_encoder_counts[4] = {0, 0, 0, 0};

void processCommandFrame(const Frame & frame)
{
  if (frame.len != sizeof(CommandFramePayload)) {
    current_error_code = ErrorCode::MALFORMED_FRAME;
    current_flags |= Flags::MALFORMED_ERROR_SINCE_LAST;
    return;
  }

  // Update last valid command time
  last_valid_command_time = time_us_64() / 1000;    // Convert to ms

  // Clear watchdog if it was tripped
  if (watchdog_tripped) {
    watchdog_tripped = false;
    current_flags &= ~Flags::WATCHDOG_TRIGGERED;
  }

  // Apply command to actuators
  uint16_t actuator_flags = 0;
  actuators.applyCommand(frame.command, actuator_flags);
  current_flags |= actuator_flags;

  // Update sequence echo
  current_seq_echo = frame.command.seq;

  // Clear previous error if command was processed successfully
  if (current_error_code != ErrorCode::WATCHDOG_TIMEOUT) {
    current_error_code = ErrorCode::OK;
  }
}

void checkWatchdog()
{
  uint64_t current_time = time_us_64() / 1000;    // Convert to ms

  if (current_time - last_valid_command_time > WATCHDOG_TIMEOUT_MS) {
    if (!watchdog_tripped) {
      watchdog_tripped = true;
      current_flags |= Flags::WATCHDOG_TRIGGERED;
      current_error_code = ErrorCode::WATCHDOG_TIMEOUT;

      // Set all actuators to safe state
      actuators.setAllNeutral();
    }
  }
}

void sendStateFrame()
{
  StateFramePayload state;
  memset(&state, 0, sizeof(state));

  // Set protocol version
  state.protocol_version = PROTOCOL_VERSION;

  // Get current actuator state
  actuators.getCurrentState(state);

  // Simulate encoder counts (increment for demonstration)
  for (int i = 0; i < 4; i++) {
    fake_encoder_counts[i] += 10 * i;      // Different increment per wheel
    state.encoder_counts[i] = fake_encoder_counts[i];
  }

  // Set timing info
  uint64_t current_time = time_us_64() / 1000;
  state.dt_ms = static_cast<uint16_t>(current_time - last_state_frame_time);
  last_state_frame_time = current_time;

  // Set sequence and status
  state.seq_echo = current_seq_echo;
  state.flags = current_flags;
  state.error_code = static_cast<uint8_t>(current_error_code);

  // Encode and send frame
  uint8_t buffer[64];
  size_t frame_size = FrameEncoder::encodeState(state, buffer, sizeof(buffer));

  if (frame_size > 0) {
    for (size_t i = 0; i < frame_size; i++) {
      putchar(buffer[i]);
    }
    fflush(stdout);
  }

  // Clear one-shot flags after sending
  current_flags &= ~(Flags::CRC_ERROR_SINCE_LAST | Flags::MALFORMED_ERROR_SINCE_LAST);
}

void processIncomingData()
{
  int c = getchar_timeout_us(0);    // Non-blocking read
  if (c == PICO_ERROR_TIMEOUT) {
    return;
  }

  Frame frame;
  ParseResult result = parser.parseByte(static_cast<uint8_t>(c), frame);

  switch (result) {
    case ParseResult::FRAME_COMPLETE:
      if (frame.frame_id == static_cast<uint8_t>(FrameId::COMMAND)) {
        processCommandFrame(frame);
      }
      break;

    case ParseResult::CRC_MISMATCH:
      current_flags |= Flags::CRC_ERROR_SINCE_LAST;
      current_error_code = ErrorCode::CRC_FAIL;
      break;

    case ParseResult::FRAME_ERROR:
      current_flags |= Flags::MALFORMED_ERROR_SINCE_LAST;
      if (parser.getLastError() == ErrorCode::UNKNOWN_FRAME_ID) {
        current_error_code = ErrorCode::UNKNOWN_FRAME_ID;
      } else {
        current_error_code = ErrorCode::MALFORMED_FRAME;
      }
      break;

    case ParseResult::NEED_MORE_DATA:
      // Continue processing
      break;
  }
}

int main()
{
  stdio_init_all();

  // Initialize actuators
  if (!actuators.init()) {
    printf("Failed to initialize actuators\n");
    return -1;
  }

  // Initialize timing
  uint64_t current_time = time_us_64() / 1000;
  last_valid_command_time = current_time;
  last_state_frame_time = current_time;

  printf("MecaBridge firmware started\n");

  while (true) {
    // Process incoming serial data
    processIncomingData();

    // Check watchdog
    checkWatchdog();

    // Send state frame at regular intervals
    uint64_t current_time = time_us_64() / 1000;
    if (current_time - last_state_frame_time >= STATE_FRAME_INTERVAL_MS) {
      sendStateFrame();
    }

    // Small delay to prevent busy-waiting
    sleep_ms(1);
  }

  return 0;
}
