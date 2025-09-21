# Implementation Plan: MecaBridge Hardware Interface

**Branch**: `001-mecabridge-plan` | **Date**: 2025-09-19 | **Spec**: `src/drive_arduino/specs/001-mecabridge-plan/spec.md`
**Input**: Feature specification from `src/drive_arduino/specs/001-mecabridge-plan/spec.md`

## Summary
Primary goal: Implement a C++ ros2-control SystemInterface (`MecaBridgeHardware`) and matching Pico firmware providing a safe, low‑latency serial bridge for a mecanum mobile base with 4 DC wheel motors (velocity), 1 positional servo (position), 1 continuous servo (velocity), and 2 ESC channels (normalized velocity). End‑to‑end (Twist receipt → PWM update) p95 latency ≤ 20 ms, watchdog enforced safe stop ≤ 150 ms after last valid heartbeat. Single authoritative YAML config defines geometry, joint names, encoder ticks, servo & ESC calibration. Deterministic frame protocol (START_BYTE 0xAA, frame_id, len, payload, CRC‑16/CCITT‑FALSE) with explicit error handling and safety flags.

## Technical Context
**Language/Version**: C++17 (ROS 2 Humble) + C++17 (Pico SDK)  
**Primary Dependencies**: ros2_control, ros2_controllers (mecanum_drive_controller, forward_command_controller, joint_state_broadcaster), rclcpp, pluginlib, serial backend (e.g. termios or libserial), Pico SDK (pico_stdlib, hardware_pwm, hardware_pio)  
**Storage**: N/A (runtime parameters only)  
**Testing**: ament_cmake + GTest (host), possibly microcontroller-side unit tests (optional), integration tests using loopback / fake serial, latency measurement harness  
**Target Platform**: Raspberry Pi 4B (Ubuntu 22.04, ROS 2 Humble) + Raspberry Pi Pico MCU  
**Project Type**: Single HAL package (ament_cmake) + firmware directory  
**Performance Goals**: p95 end‑to‑end command latency ≤ 20 ms; watchdog trigger ≤ 150 ms; stable 50–100 Hz state publish rate  
**Constraints**: Deterministic joint ordering, CRC integrity, single YAML config, compile-time feature toggles, no dynamic memory spikes in realtime loop  
**Scale/Scope**: Single robot base (v1), extensible for future modules (test mode deferred)  

## Constitution Check
*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Principle 1 (Modularity): Single package `mecabridge_hardware` + isolated firmware → PASS.
Principle 2 (ROS 2 & ros2-control Compatibility): Using SystemInterface with canonical velocity/position interfaces, no custom control loop duplication → PASS.
Principle 3 (Safety & Fail-Safe Operation): Watchdog ≤150 ms, startup outputs zero, CRC validation, explicit safe state on error → PASS.
Principle 4 (Test-First & HIL): Plan includes unit tests (frame encode/decode, watchdog), integration latency tests; HIL future harness documented → PASS (pending artifact creation).
Principle 5 (Versioned & Stable Contracts): Protocol, YAML schema, joint interface names documented under contracts/; semantic versioning via package.xml + CHANGELOG → PASS.

No violations requiring Complexity Tracking.

**Based on Constitution v1.1.0**

## Execution Flow (/plan command scope)
```
1. Load feature spec from Input path
   → If not found: ERROR "No feature spec at {path}"
2. Fill Technical Context (scan for NEEDS CLARIFICATION)
   → Detect Project Type from context (web=frontend+backend, mobile=app+api)
   → Set Structure Decision based on project type
3. Fill the Constitution Check section based on the content of the constitution document.
4. Evaluate Constitution Check section below
   → If violations exist: Document in Complexity Tracking
   → If no justification possible: ERROR "Simplify approach first"
   → Update Progress Tracking: Initial Constitution Check
5. Execute Phase 0 → research.md
   → If NEEDS CLARIFICATION remain: ERROR "Resolve unknowns"
6. Execute Phase 1 → contracts, data-model.md, quickstart.md, agent-specific template file (e.g., `CLAUDE.md` for Claude Code, `.github/copilot-instructions.md` for GitHub Copilot, `GEMINI.md` for Gemini CLI, `QWEN.md` for Qwen Code or `AGENTS.md` for opencode).
7. Re-evaluate Constitution Check section
   → If new violations: Refactor design, return to Phase 1
   → Update Progress Tracking: Post-Design Constitution Check
8. Plan Phase 2 → Describe task generation approach (DO NOT create tasks.md)
9. STOP - Ready for /tasks command
```

**IMPORTANT**: The /plan command STOPS at step 7. Phases 2-4 are executed by other commands:
- Phase 2: /tasks command creates tasks.md
- Phase 3-4: Implementation execution (manual or via tools)

## Project Structure

### Documentation (this feature)
```
specs/[###-feature]/
├── plan.md              # This file (/plan command output)
├── research.md          # Phase 0 output (/plan command)
├── data-model.md        # Phase 1 output (/plan command)
├── quickstart.md        # Phase 1 output (/plan command)
├── contracts/           # Phase 1 output (/plan command)
└── tasks.md             # Phase 2 output (/tasks command - NOT created by /plan)
```

### Source Code (repository root)
```
src/
├── mecabridge_hardware/
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── include/
│   │   └── mecabridge_hardware/        # SystemInterface headers + protocol contracts
│   ├── src/
│   │   ├── mecabridge_hardware/        # ros2_control implementation + protocol lib
│   │   └── mecabridge_utils/           # shared helpers (CRC, YAML validation, watchdog)
│   ├── test/
│   │   └── mecabridge/                 # GTest + integration harnesses
│   ├── config/                         # YAML templates & sample configs
│   └── launch/                         # bringup + test launches
firmware/
└── mecabridge_pico/
    ├── CMakeLists.txt
    ├── include/
    ├── src/                            # Pico firmware modules (serial, pwm, watchdog)
    └── tests/
```

**Structure Decision**: Create standalone `mecabridge_hardware` ROS 2 package plus sibling Pico firmware directory (`firmware/mecabridge_pico`); existing packages remain untouched.

## Phase 0: Outline & Research
(Completed – see `research.md`)

## Phase 1: Design & Contracts
(Completed – see `data-model.md`, `contracts/`, `quickstart.md`)

## Phase 2: Task Planning Approach
*This section describes what the /tasks command will do - DO NOT execute during /plan*

**Task Generation Strategy**:
- For each FR → at least one test & implementation task.
- Frame protocol tests (encode/decode, CRC, watchdog) marked high priority.
- Protocol version handshake (FR-012) gets its own validation + mismatch test task.
- Latency harness tasks generate measurement script before hardware loop.

**Ordering Strategy**:
1. Protocol library + CRC tests
2. YAML config loader & validation
3. Hardware interface skeleton (exports only)
4. Protocol version handshake + mismatch reporting (FR-012)
5. State parsing + command serialization
6. Watchdog logic integration
7. ros2_control integration tests (joint registration)
8. Latency measurement harness
9. Feature toggles & flags
10. ESC/servo scaling
11. Documentation finalization

**Parallelizable**: CRC tests, YAML schema tests, frame encode/decode tests can proceed in parallel after skeleton.

## Complexity Tracking
(No deviations – table omitted)

## Progress Tracking
**Phase Status**:
- [x] Phase 0: Research complete (/plan command)
- [x] Phase 1: Design complete (/plan command)
- [ ] Phase 2: Task planning complete (/plan command - describe approach only)
- [ ] Phase 3: Tasks generated (/tasks command)
- [ ] Phase 4: Implementation complete
- [ ] Phase 5: Validation passed

**Gate Status**:
- [x] Initial Constitution Check: PASS
- [x] Post-Design Constitution Check: PASS
- [x] All NEEDS CLARIFICATION resolved
- [ ] Complexity deviations documented (none needed)

---
*Based on Constitution v1.1.0 - See `/memory/constitution.md`*
