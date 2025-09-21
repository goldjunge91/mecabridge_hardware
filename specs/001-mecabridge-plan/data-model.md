# Data Model: MecaBridge

Date: 2025-09-19  
Branch: 001-mecabridge-plan  

## 1. Entities Overview
| Entity | Purpose | Key Fields |
|--------|---------|------------|
| Joint | Logical actuator state/command mapping | name, type, command_value, state_value, limits |
| Frame | Serialized packet exchanged over serial | start_byte, frame_id, len, payload[], crc16 |
| CommandFramePayload | Outbound command semantics | wheel_vel_rad_s[4], servo_pos_rad, servo_cont_vel_norm, esc_norm[2], seq, flags |
| StateFramePayload | Inbound state semantics | encoder_counts[4], dt_ms, servo_pos_rad, servo_cont_vel_norm, esc_norm[2], flags, seq_echo, error_code |
| Watchdog | Safety timing supervision | last_valid_time, cutoff_ms, tripped_flag |
| Config | Parsed YAML source of truth | serial_port, state_rate, geometry, joint names, ticks_per_rev, servo limits, esc calibration, feature toggles |
| CRC16 | Integrity calculator | poly, init, update(), finalize() |
| FlagsBitfield | Aggregate status / safety bits | bits (uint16) |

## 2. Joint Types & Interfaces
| Type | Command Interface | State Interface | Notes |
|------|-------------------|-----------------|-------|
| wheel | velocity (rad/s) | velocity (rad/s) | Derived velocity from encoder delta; command direct target |
| servo_positional | position (rad) | position (rad) | Clamped to [min_rad, max_rad]; limit flag set if clamped |
| servo_continuous | velocity (rad/s) | velocity (rad/s) | Host may clamp by max_velocity_rad_s |
| esc | velocity (normalized -1..1) | velocity (normalized -1..1) | Host converts to PWM, state echoes normalized |

## 3. Flags Bitfield (uint16)
| Bit | Name | Description |
|-----|------|-------------|
| 0 | watchdog_triggered | Watchdog safe stop active until next valid command |
| 1 | positional_servo_limit_hit | Last command clamped |
| 2 | crc_error_since_last | At least one CRC failure since previous state frame |
| 3 | malformed_error_since_last | Length / framing error observed |
| 4 | encoder_stale | At least one encoder failed to update this cycle |
| 5 | reserved_future | Reserved |
| 6-15 | unused | Future expansion |

## 4. Error Codes (enum uint8)
| Code | Meaning |
|------|---------|
| 0 | OK (no recent error) |
| 1 | CRC_FAIL |
| 2 | MALFORMED_FRAME |
| 3 | UNKNOWN_FRAME_ID |
| 4 | VERSION_MISMATCH |
| 5 | WATCHDOG_TIMEOUT |

## 5. Frame IDs (enum uint8)
| ID | Name |
|----|------|
| 0x01 | COMMAND |
| 0x02 | STATE |
| 0x03 | HEARTBEAT (optional separate frame if no command changes) |
| 0x04 | VERSION_INFO |
| 0x05 | ERROR (optional dedicated error frame) |

Initial implementation may only use COMMAND + STATE; heartbeat semantics satisfied by periodic COMMAND frames. Keep enum reserved for forward compatibility.

## 6. Command Frame (Binary Layout)
All multi-byte numeric fields little-endian (LE) for MCU simplicity.
```
Offset  Size  Field
0       1     START_BYTE (0xAA)
1       1     FRAME_ID (0x01)
2       1     LEN (payload bytes)
3       4*4   wheel_vel_rad_s[4] (float32 LE)
19      4     servo_pos_rad (float32 LE)
23      4     servo_cont_vel_norm (float32 LE)
27      4*2   esc_norm[2] (float32 LE)
35      2     seq (uint16 LE)
37      1     reserved_flags (future)
38      2     CRC16 (MSB first)  <-- computed over FRAME_ID..last payload byte (exclude START_BYTE & CRC)
```
LEN currently = 35 (payload bytes from offset 3 to 37 inclusive minus header/trailer) → verify exact count: wheel(16)+servo_pos(4)+servo_cont(4)+esc(8)+seq(2)+flags(1)=35.

## 7. State Frame (Binary Layout)
```
Offset  Size  Field
0       1     START_BYTE (0xAA)
1       1     FRAME_ID (0x02)
2       1     LEN
3       4*4   encoder_counts[4] (uint32 LE)
19      2     dt_ms (uint16 LE)  # elapsed time since last state frame
21      4     servo_pos_rad (float32 LE)
25      4     servo_cont_vel_norm (float32 LE)
29      4*2   esc_norm[2] (float32 LE)
37      2     seq_echo (uint16 LE)
39      2     flags (uint16 LE)
41      1     error_code (uint8)
42      2     CRC16
```
Payload LEN = encoder(16)+dt(2)+servo_pos(4)+servo_cont(4)+esc(8)+seq_echo(2)+flags(2)+error(1)=39.

## 8. CRC Algorithm
CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflect in/out, xorout 0x0000. Process bytes FRAME_ID..end payload inclusive (exclude START_BYTE and the two CRC bytes). Test vectors to be added in contracts.

## 9. YAML Config Mapping → Runtime
| YAML Key | Runtime Field |
|----------|---------------|
| serial_port | Config.serial_port |
| state_publish_rate_hz | Config.state_rate_hz |
| wheel_radius | geometry.wheel_radius |
| wheel_separation_x / y | geometry.center_offset_x/y |
| encoder_ticks_per_rev | ticks_per_rev |
| wheels.front_left.encoder_index | wheel[0].encoder_index |
| servos.positional.min_rad/max_rad | servo_positional.limits |
| servos.continuous.max_velocity_rad_s | servo_continuous.max_vel |
| escs.left.esc_min_pwm | esc_left.min_pwm |
| escs.left.esc_max_pwm | esc_left.max_pwm |
| escs.left.esc_deadband | esc_left.deadband |
| features.enable_* | feature toggles |

## 10. Validation Rules
- All joint names unique.
- wheel radius > 0, ticks_per_rev > 0.
- servo min_rad < max_rad.
- ESC min_pwm < max_pwm, both > 0.
- state_publish_rate_hz within [10, 200].
- Feature toggles boolean.
- If enable_escs=false then esc related config optional.

## 11. Derived Calculations
- Wheel linear velocity = wheel_vel_rad_s * wheel_radius.
- Odometry handled by controller; driver only supplies accurate wheel velocities (derived from encoder deltas / dt_ms).
- PWM for ESC = map(normalized, [-1,1]) → [min_pwm, max_pwm] with optional deadband suppression around 0.

## 12. Sequence Handling
`seq` increments per COMMAND frame (uint16 wrap). Firmware echoes latest valid `seq` in `seq_echo`. Host detects missed COMMAND acceptance if gap >1.

## 13. Deterministic Ordering
Index mapping fixed: [front_left, front_right, rear_left, rear_right] for wheel arrays and encoder_counts.

## 14. Memory Footprint (Approx)
- Command payload 35 bytes + header(3)+CRC(2)=40 bytes total.
- State payload 39 bytes + header(3)+CRC(2)=44 bytes total.
Target < 64 bytes each satisfying low latency & minimal USB overhead.

## 15. Lifecycle Notes
- on_init(): parse YAML, build joint arrays, open serial.
- read(): parse frames; update joint states; apply safety flag logic.
- write(): build command frame; compute CRC; write serial; update last_write_time.

## 16. Future Extension (Deferred)
- Synthetic encoder injection (test mode) would add new FRAME_ID and payload variant.

All data structures now defined for contract generation.
