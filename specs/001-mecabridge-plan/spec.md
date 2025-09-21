# Feature Specification: Create MecaBridge

**Feature Branch**: `001-mecabridge-plan`  
**Created**: 2025-09-19  
**Status**: Draft  
**Input**: User description: "MecaBridge, a hardware bridge for a mecanum-wheel robot. It should connect a Raspberry Pi 4B (running ROS 2 and ros2_control) with a Raspberry Pi Pico (handling low-level PWM and encoder logic) over a simple serialn this initial phase, let’s call it “Create MecaBridge.”\nThe system should support:\nFour brushed DC motors with encoders mounted as mecanum wheels.\nTwo servos (one 180° positional, one 360° continuous rotation).\nTwo ESCs for additional motor controllers.\nThe Pico will receive command frames from the Pi 4B and send back state frames. All frames should include a start byte, ID, payload length, payload, and CRC checksum.\nOn the Pi 4B side, the ROS 2 control framework should manage:\nA mecanum drive controller that listens for geometry_msgs/Twist on /cmd_vel and outputs wheel velocity commands.\nA joint_state_broadcaster that publishes encoder feedback and servo/ESC states.\nA forward command controller for the servos.\nA forward command controller for the ESCs.\nConfiguration should come from a single YAML source of truth, defining wheel geometry, joint names, encoder counts, servo limits, and ESC ranges.\n\nOn the Pico side:\n\nThe firmware should parse serial frames, drive PWM outputs, and collect encoder counts.\n\nThere should be clear separation between modules for motors, servos, and ESCs.\nFeature toggles should be controlled at compile time using #define/#ifndef.\nWhen you first power on MecaBridge, it should immediately establish the USB connection and start exchanging heartbeat frames to verify connectivity. If the Pi 4B disconnects, the Pico should stop all motors and servos for safety.\nThis first version does not need advanced logging or calibration routines—just the core command/state bridge with safety stop behavior and modular structure for later extension."

---

## Execution Flow (main)
```
1. Parse user description from Input
   → If empty: ERROR "No feature description provided"
2. Extract key concepts from description
   → Identify: actors, actions, data, constraints
3. For each unclear aspect:
   → Mark with [NEEDS CLARIFICATION: specific question]
4. Fill User Scenarios & Testing section
   → If no clear user flow: ERROR "Cannot determine user scenarios"
5. Generate Functional Requirements
   → Each requirement must be testable
   → Mark ambiguous requirements
6. Identify Key Entities (if data involved)
7. Run Review Checklist
   → If any [NEEDS CLARIFICATION]: WARN "Spec has uncertainties"
   → If implementation details found: ERROR "Remove tech details"
8. Return: SUCCESS (spec ready for planning)
```

---

## ⚡ Quick Guidelines
- ✅ Focus on WHAT users need and WHY
- ❌ Avoid HOW to implement (no tech stack, APIs, code structure) EXCEPT baseline hardware/ROS interface context intrinsic to the feature
- 👥 Written for stakeholders defining capability of platform HAL + control integration

### Section Requirements
- **Mandatory sections**: Must be completed for every feature
- **Optional sections**: Include only when relevant to the feature
- When a section doesn't apply, remove it entirely (don't leave as "N/A")

### For AI Generation
Ambiguities are explicitly marked below.

---

## User Scenarios & Testing *(mandatory)*

### Primary User Story
An operator deploys a mecanum mobile base using the MecaBridge HAL. A ROS 2 navigation or teleop node publishes `/cmd_vel` and the robot drives smoothly via the mecanum drive kinematics. Encoder feedback, servo angles, and ESC throttle states are published to the ROS 2 graph. If the USB serial link is interrupted, all motion outputs stop within a bounded safety timeout.

### Acceptance Scenarios
1. **Given** the system is powered and the USB link is active, **When** a `/cmd_vel` Twist with non-zero linear.x and angular.z is published, **Then** appropriate four wheel velocity commands are sent to the Pico and wheels rotate accordingly (simulation or HIL) while joint states reflect motion.
2. **Given** an active session, **When** the `/cmd_vel` topic stops updating (no new messages within the watchdog window), **Then** the Pico commands all motors and ESCs to zero output.
3. **Given** servo command topics publish target positions (180° servo) and speed/throttle for continuous/ESC joints, **When** valid values are within configured limits, **Then** those values are reflected in outbound frames and joint state feedback.
4. **Given** the USB cable is unplugged or Pi resets, **When** heartbeat frames cease, **Then** the Pico halts PWM outputs and reports a fault state on reconnection.
5. **Given** configuration YAML defines geometry and joint names, **When** the system starts, **Then** controllers load successfully and no parameter resolution errors occur.

### Edge Cases
- Serial framing error / bad CRC (CRC-16/CCITT-FALSE failure).
- Frame loss or partial frame at startup (incomplete buffer flush during bring-up).
- Servo limit exceeded → command hard-clamped to YAML min/max; state frame sets `limit_hit=1`.
- Rapid reconnect cycles (USB flaps) – watchdog must not falsely trigger within HEARTBEAT_MS jitter yet still enforce SAFE_CUTOFF_MS.
- One encoder non-responsive (stale count) – publish stale flag / derived velocity = 0.

## Constants & Protocol Parameters
| Name                     | Value              | Description                                                  |
| ------------------------ | ------------------ | ------------------------------------------------------------ |
| START_BYTE               | 0xAA               | First byte for every frame                                   |
| PROTOCOL_VERSION         | 0x01               | Firmware / protocol compatibility identifier                 |
| HEARTBEAT_MS             | 50                 | Expected max interval between valid heartbeat/command frames |
| SAFE_CUTOFF_MS           | 150                | Upper bound before forced safe stop (outputs zeroed)         |
| MAX_E2E_LATENCY_MS (p95) | 20                 | Max 95th percentile end-to-end command latency               |
| CRC                      | CRC-16/CCITT-FALSE | Poly 0x1021, init 0xFFFF, no reflect, final XOR 0x0000       |
| ESC_RANGE                | [-1.0, 1.0]        | Normalized throttle / velocity range                         |

All timing, range, and protocol constants MUST be enforced in tests (unit + integration).

## Requirements *(mandatory)*

### Functional Requirements
- **FR-001**: System MUST support four mecanum wheel DC motor joints with encoder feedback.
- **FR-002**: System MUST expose a mecanum drive controller consuming `/cmd_vel` and producing wheel velocity commands.
- **FR-003**: System MUST publish all joint states (wheels, servos, ESCs) via `joint_state_broadcaster` at configured rate.
- **FR-004**: System MUST support two servo joints: one positional (position interface, radians) and one continuous rotation (velocity interface, rad/s mapped internally to normalized speed).
- **FR-005**: System MUST support two ESC joints and the continuous rotation servo using a normalized velocity/throttle interface in range [-1.0, 1.0] (dimensionless). YAML MUST define per ESC: `esc_min_pwm`, `esc_max_pwm` (µs) and optional `esc_deadband`. Host scales normalized value → PWM µs before serialization; Pico receives final PWM only.
- **FR-006**: System MUST maintain a single YAML configuration defining wheel geometry (wheel radius, wheel separation, roller angle), joint names, encoder ticks per revolution, servo min/max, ESC min/max, ESC calibration parameters.
- **FR-007**: Pico firmware MUST parse and validate serial command frames: START_BYTE, FRAME_ID, LEN, PAYLOAD, CRC (CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, final XOR 0x0000). Invalid CRC frames are discarded.
- **FR-008**: Pico firmware MUST emit state frames at a fixed frequency containing encoder counts, servo positions, ESC outputs, heartbeat counter, limit flags, and error codes.
- **FR-009**: System MUST implement a heartbeat/watchdog: absence of any valid command or heartbeat frame for > SAFE_CUTOFF_MS (150 ms) SHALL force all motor/servo/ESC outputs to zero.
- **FR-010**: System MUST provide compile-time feature toggles for (a) servos, (b) ESCs, (c) extended diagnostics.
- **FR-011**: System MUST discard malformed (length mismatch), CRC-failed, or invalid-ID frames. If within a single HEARTBEAT_MS interval only invalid frames are received, all outputs SHALL be set to safe (zero). The next state frame MUST include `frame_error_code` (enum) referencing the last error cause.
- **FR-012**: System MUST provide a unique protocol version/id so that mismatched firmware is detectable.
- **FR-013**: System MUST ensure deterministic order of joints in state publications matching configuration.
- **FR-014**: System MUST zero all outputs no later than SAFE_CUTOFF_MS (150 ms) after last valid heartbeat/command.
- **FR-015**: System MUST log (or encode via state frame flag) at least one safety event when watchdog triggers.
- **FR-016 (Deferred)**: Dynamic test mode to inject synthetic encoder counts is postponed; not implemented in v1 (tracked under Backlog).
- **FR-017**: System MUST keep end-to-end latency (timestamped `/cmd_vel` receipt → PWM update) ≤ 20 ms at 95th percentile: host serialization ≤ 5 ms; transport + parse < 10 ms; PWM apply < 5 ms. Tests MUST assert this distribution (p95) and flag violations.
- **FR-018**: Joint interfaces MUST match ros2-control conventions: wheels = velocity (rad/s); positional servo = position (rad); continuous servo = velocity (rad/s); ESCs = velocity (dimensionless normalized [-1.0, 1.0], scaled to PWM by host).
- **FR-019**: System MUST provide reproducible colcon tests validating frame encode/decode round trip and watchdog timeout behavior.
- **FR-020**: System MUST document safety behavior, protocol format (including CRC, frame layout), and configuration schema.

### Backlog / Deferred (Post-v1)
- FR-016 implementation (synthetic encoder injection test mode).
- Extended logging & calibration routines.
- Runtime reconfiguration of PWM calibration without firmware rebuild.

### Key Entities *(include if feature involves data)*
- **Joint**: Abstract representation of an actuator with a name, type (wheel, servo_positional, servo_continuous, esc), and interface(s).
- **Frame**: Serial packet with fields {start_byte, frame_id, len, payload[], crc} and semantic subtypes (command, state, heartbeat, version, error).
- **EncoderState**: Counts per wheel, converted to velocity using ticks per revolution and time delta.
- **Heartbeat**: Monotonic counter or timestamp to prove liveness.
- **SafetyWatchdog**: Timer tracking last good command or heartbeat.
- **Config**: Parsed YAML structure with geometry, joint definitions, electrical/limit parameters.

---

## Review & Acceptance Checklist
*GATE: Automated checks run during main() execution*

### Content Quality
- [ ] No unapproved implementation detail creep beyond hardware/control contract surface
- [ ] Focused on capabilities & constraints
- [ ] All mandatory sections completed

### Requirement Completeness
- [ ] No unresolved clarification markers (confirmed none remain)
- [ ] Requirements testable & unambiguous
- [ ] Success criteria measurable (latency, timeout, frame format)
- [ ] Scope clearly bounded (no calibration, advanced logging in v1)
- [ ] Dependencies & assumptions identified (CRC algorithm, watchdog timeout, ranges)

---

## Execution Status
*Updated by main() during processing*

- [ ] User description parsed
- [ ] Key concepts extracted
- [ ] Ambiguities marked
- [ ] User scenarios defined
- [ ] Requirements generated
- [ ] Entities identified
- [ ] Review checklist passed

---
