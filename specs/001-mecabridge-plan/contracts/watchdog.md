# Contract: Safety Watchdog

Related FR: FR-009, FR-014, FR-015, FR-020

## 1. Purpose
Ensure all actuators enter a safe (zero) state within SAFE_CUTOFF_MS (150 ms) after last valid COMMAND (or HEARTBEAT) frame.

## 2. Timing
- HEARTBEAT_MS expectation: ≤ 50 ms between valid commands during normal operation.
- SAFE_CUTOFF_MS: 150 ms (hard upper bound).

## 3. Trigger Conditions
Watchdog is considered TRIPPED if `now - last_valid_frame_time > SAFE_CUTOFF_MS`.

## 4. Actions on Trip
- Set all wheel velocities target = 0.
- Set servo_continuous velocity = 0.
- Maintain positional servo at last safe position (no forced move).
- Set ESC outputs to neutral (normalized 0 → midpoint PWM mapping).
- Set flags.watchdog_triggered=1.
- Set error_code=WATCHDOG_TIMEOUT if no previous higher-precedence error.

## 5. Recovery
A valid COMMAND frame (CRC OK, parsed) clears watchdog_triggered flag on next STATE frame emission.

## 6. Ordering & Determinism
If multiple faults occur simultaneously (CRC fail + timeout), precedence order for error_code:
1. WATCHDOG_TIMEOUT (if tripped)
2. CRC_FAIL
3. MALFORMED_FRAME
4. UNKNOWN_FRAME_ID
5. OK

Flags may show multiple aggregated bits, but single error_code chosen by precedence.

## 7. Test Cases
1. Normal Operation: periodic commands every 40 ms → watchdog never trips over 5 s.
2. Silence: stop commands; after 160 ms state frame reports watchdog_triggered=1 and error_code=WATCHDOG_TIMEOUT.
3. Recovery: send valid command at t=170 ms → next state clears flag.
4. Concurrent CRC Errors: send series of bad CRC frames only → watchdog triggers; error_code=WATCHDOG_TIMEOUT (takes precedence).

## 8. Implementation Notes
- Use steady clock (monotonic) on host / firmware.
- Firmware executes check in main control loop before applying command outputs.

Stable for v1. Changes altering timing constants require justification & semantic version bump.
