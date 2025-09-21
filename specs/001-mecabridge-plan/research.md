# Research: MecaBridge Hardware Interface

Date: 2025-09-19  
Branch: 001-mecabridge-plan  
Spec: `src/drive_arduino/specs/001-mecabridge-plan/spec.md`

## 1. Goals Recap
Provide a ros2-control SystemInterface + Pico firmware bridging 4 mecanum wheel motors, 2 servos, 2 ESC channels over a deterministic serial protocol with safety watchdog and low latency.

## 2. Key Decisions
| Decision | Rationale | Alternatives Considered |
|----------|-----------|-------------------------|
| Create dedicated ament_cmake package `mecabridge_hardware` with sibling firmware directory | Keeps HAL isolated per Constitution Principle 1 and matches plan/tasks structure | Extend existing `drive_arduino` package (risk of coupling, harder reuse) |
| SystemInterface vs Actuator/Multiple Components | One logical communication channel (serial) for multi-DOF set → SystemInterface fits | Separate Actuator components per wheel (more plugin overhead) |
| Use mecanum_drive_controller | Standard controller, avoids custom kinematics | Custom twist → wheel mapper |
| Velocity interface for wheels | Matches controller expectations | Position (would require integration differentiation) |
| CRC-16/CCITT-FALSE | Explicitly specified in spec, common, efficient | CRC-32 (larger payload, unnecessary), Fletcher-16 (less common) |
| Host scales ESC normalized [-1,1] → PWM | Centralize calibration in YAML; firmware simpler | Firmware scaling (duplicate config + complexity) |
| YAML single source of truth | Prevent divergence between URDF and driver | Split YAML + parameters (sync drift risk) |
| Watchdog SAFE_CUTOFF_MS=150ms | Spec; balances responsiveness & noise tolerance | 100ms (risk false stops), 200ms (slower fail-safe) |
| p95 latency budget 20ms | Spec; ensures responsive teleop | 10ms (tight on USB + ROS), 30ms (sluggish feel) |
| Deterministic joint ordering | Stable mapping for controllers/tests | Dynamic discovery (ordering nondeterministic) |
| Compile-time feature toggles | Allows trimming firmware & tests for minimal builds | Runtime toggles (increase code path, RAM) |

## 3. ros2_control Integration Notes
- Hardware plugin must export class via `PLUGINLIB_EXPORT_CLASS(mecabridge_hardware::MecaBridgeHardware, hardware_interface::SystemInterface)`.
- Lifecycle methods: `on_init() -> return_type`, `export_state_interfaces()`, `export_command_interfaces()`, `read()`, `write()`, optional `prepare_command_mode_switch()`.
- Ensure non-blocking or bounded blocking in `read()`/`write()`. Use ring buffer for serial reads; parse frames in `read()`; accumulate partial frames.
- Maintain time of last valid frame; if now - last_valid > SAFE_CUTOFF_MS set zero commands (and set safety flag for next state frame).

## 4. Frame Protocol Outline (Pre-Contract)
```
Byte 0:  START_BYTE (0xAA)
Byte 1:  FRAME_ID (enum) e.g. 0x01=COMMAND, 0x02=STATE, 0x03=HEARTBEAT, 0x04=VERSION, 0x05=ERROR
Byte 2:  LEN (payload length N)
Bytes 3..(3+N-1): PAYLOAD
Bytes end-2..end-1: CRC16 (MSB first)
```
State Payload (draft fields):
- wheel velocities or encoder counts (decide representation – counts allow host differentiation) → choose counts (uint32 per wheel) + delta time ms (uint16)
- servo positional angle (float32 rad), continuous servo reported normalized velocity or measured? (report last commanded + optional tach if available; initial spec requires state) → last commanded for v1
- ESC outputs (float32 normalized) x2
- watchdog status + limit flags (bitfield uint16)
- heartbeat counter (uint16 rollover)

Command Payload (draft fields):
- wheel target velocities (float32 rad/s) x4
- positional servo desired angle (float32 rad)
- continuous servo target normalized velocity (float32 -1..1)
- ESC normalized commands (float32 -1..1) x2
- sequence / heartbeat counter (uint16)

Justification: Using float32 for velocities simplifies conversion; encoders in state frame retain precision for odometry.

## 5. Safety & Watchdog Strategy
- Host always sends COMMAND frames at control loop rate (e.g. 50-100Hz). If no actuator changes, still send heartbeat flag via sequence counter.
- Firmware resets `last_command_time` upon valid COMMAND or HEARTBEAT frame.
- If (now - last_command_time) > SAFE_CUTOFF_MS: set all PWM = safe (0), set watchdog flag bit, include event code in ERROR or next STATE.
- Host upon detecting missed state intervals may log warning (FR-015 logging minimal: encode flag in state).

## 6. Latency Measurement Approach
- Host timestamp at `/cmd_vel` receipt (rclcpp::Time) stored in ring.
- When controller writes commands, record serialization send time.
- Firmware echoes sequence/heartbeat counter in STATE frame; host matches to original timestamp to compute RTT; subtract half of average state period to approximate one-way or accept RTT as proxy.
- p95 computed over sliding window (e.g. 200 samples). Test harness asserts <= 20ms.

## 7. YAML Configuration Schema (Draft)
```yaml
mecabridge_hardware:
  serial_port: /dev/ttyACM0
  baud_rate: 115200            # optional if USB CDC; else ignored
  state_publish_rate_hz: 50
  wheel_radius: 0.048          # meters
  wheel_separation_x: 0.20     # half-distance front-back? define clearly in docs
  wheel_separation_y: 0.18
  encoder_ticks_per_rev: 1024
  wheels:
    front_left:  encoder_index: 0
    front_right: encoder_index: 1
    rear_left:   encoder_index: 2
    rear_right:  encoder_index: 3
  servos:
    positional:
      name: pan_servo
      min_rad: 0.0
      max_rad: 3.14159
    continuous:
      name: rot_servo
      max_velocity_rad_s: 10.0   # optional for sanity clamp
  escs:
    left:
      name: esc_left
      esc_min_pwm: 1000
      esc_max_pwm: 2000
      esc_deadband: 0            # optional µs
    right:
      name: esc_right
      esc_min_pwm: 1000
      esc_max_pwm: 2000
  features:
    enable_servos: true
    enable_escs: true
    enable_diagnostics: false
```

## 8. Bitfield / Flags (Draft)
```
Bit 0: watchdog_triggered
Bit 1: positional_servo_limit_hit
Bit 2: frame_crc_error_since_last
Bit 3: frame_malformed_error_since_last
Bit 4: encoder_stale_flag
Bit 5: reserved_future
...
```

## 9. Open Points (Resolved / Clarified Already)
- Test mode deferred (FR-016) → No design needed in v1.
- Servo clamping: Host clamps before sending; firmware secondarily clamps for safety, sets flag.
- ESC scaling: host converts normalized to PWM; firmware receives direct PWM value; keep both normalized and pwm? → send PWM only to minimize bytes.

## 10. Risks & Mitigations
| Risk | Impact | Mitigation |
|------|--------|-----------|
| Serial fragmentation / partial frames | Dropped commands / jitter | Robust parser with state machine + timeouts |
| Latency > 20ms due to USB scheduling | Fails FR-017 | Keep payload small, optimize parsing, consider increasing state rate to reduce buffering |
| CRC implementation mismatch | Discarding all frames | Add unit test vectors vs known CRC examples |
| Encoder overflow (uint32 wrap) | Wrong velocity | Use delta counts with uint32; host handles wrap-around safely |
| Clock drift for latency metrics | Mis-measured p95 | Use host-only RTT and adjust; document method |
| YAML misconfiguration | Startup failure | Validate all ranges and names at on_init() with descriptive errors |

## 11. References
- ros2_control docs (architecture, hardware component lifecycle)
- mecanum_drive_controller user doc
- CRC-16/CCITT-FALSE reference polynomial 0x1021

## 12. Completion
All unknowns in spec are resolved; ready to proceed to Phase 1 design artifacts.
