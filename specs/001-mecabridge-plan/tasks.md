# Tasks: MecaBridge Hardware Interface (Feature 001)

Branch: `001-mecabridge-plan`  
Spec: `/specs/001-mecabridge-plan/spec.md`  
Plan: `/specs/001-mecabridge-plan/plan.md`  
Contracts: `/specs/001-mecabridge-plan/contracts/`  

Legend:

- [P] = Can be executed in parallel with other [P] tasks (different files / no ordering dependency)
- FR-XXX references Functional Requirements in spec
- CI Gate Tags: [SAFETY], [LATENCY], [PROTOCOL], [CONFIG], [ROS2-CONTROL]

Ordering enforces: Test-first → Core protocol → Config → Skeletons → Logic → Integration → Performance → Docs

## Dependency Graph (High-Level)

```
T001 → T002 → T003 → T004 → T005 → T005a → T006 → T008 → T010 → T014
                                 └────────→ T007 → T009 → T011 → T012 → T013
Parallel clusters: {T001,T002 strict}; {T003,T004 CRC/layout tests parallel}; {T016..T019 docs/polish}; Firmware tasks (T015–T019) start after T005 (protocol logic) but before T005a completion (handshake tests can arrive later).
```

## Tasks

### Setup & Scaffolding

[X] T001. Create protocol library skeleton [PROTOCOL]

- Files: `src/mecabridge_hardware/src/mecabridge_utils/protocol/{frame.hpp, frame.cpp, crc16.hpp, crc16.cpp}` (host package root `src/mecabridge_hardware/` per plan structure)
- Define enums (FrameId, ErrorCode), constants (START_BYTE, etc.), struct layouts (CommandFrame, StateFrame) mirroring `data-model.md` & `frame_protocol.md`.
- Provide function stubs: `encodeCommand(...)`, `tryParseFrame(ByteSpan, ParsedFrame&)`, `crc16_ccitt_false(data,len)`.
- No logic yet (return NOT_IMPLEMENTED).
- Add placeholder CMake target `mecabridge_protocol`.

[X] T002. Add GTest target & basic build integration [PROTOCOL]

- Files: `src/mecabridge_hardware/test/mecabridge/protocol/test_protocol_empty.cpp`
- Verify library compiles & headers include cleanly.
- CI passes with one trivial test (EXPECT_TRUE(true)).

### Protocol Tests First

[X] T003. Implement CRC16 tests & function [P] [PROTOCOL]

- Files: `src/mecabridge_hardware/test/mecabridge/protocol/test_crc16.cpp`, update `src/mecabridge_hardware/src/mecabridge_utils/protocol/crc16.cpp`.
- Add test vector "123456789" → 0x29B1; randomized fuzz of small buffers.
- Implement `crc16_ccitt_false` fully.

[X] T004. Implement frame size & layout encode/decode round-trip tests [P] [PROTOCOL]

- File: `src/mecabridge_hardware/test/mecabridge/protocol/test_frame_roundtrip.cpp`
- Tests: encode command sets LEN=35, total bytes=40; encode state sets LEN=39, total=44; decode returns identical fields; CRC validated.

[X] T005. Implement frame encoding & parsing logic [PROTOCOL]

- Fill `encodeCommand`, `encodeState`, parser state machine (one-shot buffer parse for now; iterative improvement later).
- Enforce clamp rules & set internal flags for out-of-range.

[X] T005a. Protocol version & handshake logic + mismatch path [PROTOCOL][SAFETY]

- Implement explicit `PROTOCOL_VERSION` field injection & extraction; add lightweight handshake routine (`bool verify_version(uint8_t firmware_version)`), returning false on mismatch.
- On mismatch: set internal error code VERSION_MISMATCH; watchdog stays armed; driver initialization fails fast.
- Add host unit test `src/mecabridge_hardware/test/mecabridge/protocol/test_protocol_handshake.cpp` creating a synthetic frame with wrong version.
- Preconditions: T005 completed (basic encode/decode available).

[X] T006. Add error handling tests (CRC fail, truncated, unknown ID) [PROTOCOL]

- File: `src/mecabridge_hardware/test/mecabridge/protocol/test_frame_errors.cpp`
- Generate corrupted frames; expect parser status codes & flags set per contract.

[X] T007. Add watchdog logic unit tests (host-side simulation) [SAFETY]

- File: `src/mecabridge_hardware/test/mecabridge/safety/test_watchdog.cpp`
- Simulate time progression; ensure trigger at >150ms silence.

[X] T008. Implement host-side watchdog integration helper [SAFETY]

- Files: `src/mecabridge_hardware/src/mecabridge_utils/safety/watchdog.hpp/.cpp`
- API: `void updateOnValidFrame(Time now)`, `bool tripped(Time now)`, `void reset()`, `uint16_t flags()`.

### Configuration Loader

[X] T009. Implement YAML configuration loader (parse + validate) [CONFIG]

- Files: `src/mecabridge_hardware/src/mecabridge_utils/config/config.hpp/.cpp`
- Use rclcpp::Node parameters or direct yaml-cpp (choose minimal dependency; if unavailable use rclcpp::Parameter for loaded YAML string).
- Validation rules from `yaml_config.md` (errors throw exception with message).

[X] T010. Add configuration unit tests [P] [CONFIG]

- File: `src/mecabridge_hardware/test/mecabridge/config/test_config.cpp`
- Valid config test; each validation rule negative test; ordering determinism check.

### ros2_control Hardware Interface Skeleton

[X] T011. Create `MecaBridgeHardware` skeleton class [ROS2-CONTROL]

- Files: `src/mecabridge_hardware/include/mecabridge_hardware/mecabridge_hardware.hpp`, `src/mecabridge_hardware/src/mecabridge_hardware/mecabridge_hardware.cpp`
- Inherit `hardware_interface::SystemInterface`.
- Implement methods returning OK/ERROR with TODO comments.
- Export plugin via pluginlib macro & update `package.xml` + `CMakeLists.txt`.

[X] T012. Register state & command interfaces (wheels, servos, ESCs) [ROS2-CONTROL]

- Flesh out `export_state_interfaces()` and `export_command_interfaces()` using config ordering.
- Add unit test using controller manager `ros2_control` test harness verifying interface count & names.

[X] T013. Integrate protocol library into hardware interface [PROTOCOL][ROS2-CONTROL]

- Implement `read()` frame parsing → internal state arrays.
- Implement `write()` building command frame with current command values & seq increment.
- Watchdog check before writing; zero commands if tripped.

[X] T014. Add latency measurement instrumentation [LATENCY]

- Files: `src/mecabridge_hardware/src/mecabridge_utils/latency/latency_tracker.hpp/.cpp`, `src/mecabridge_hardware/test/mecabridge/latency/test_latency_tracker.cpp`
- Track send vs echoed seq time; compute sliding window p95; unit test with synthetic timestamps.

### Pico Firmware Skeleton & Logic

[X] T015. Create Pico firmware skeleton project [P] [FIRMWARE]

- Directory: `firmware/mecabridge_pico/`
- CMakeLists, main.cpp with loop: init USB serial, placeholder parse loop calling stub parser.

[X] T016. Implement Pico CRC + frame parser state machine [FIRMWARE][PROTOCOL]

- Files: `firmware/mecabridge_pico/src/frame_parser.{hpp,cpp}`
- Reuse algorithm spec; include tests if feasible (host-side cross compile optional).

[X] T017. Implement Pico command execution + PWM abstraction [FIRMWARE]

- Files: `firmware/mecabridge_pico/src/actuators.{hpp,cpp}`
- Stub: apply wheel velocities (placeholder), servo, ESC neutral mapping.

[X] T018. Implement Pico state frame emission + watchdog enforcement [FIRMWARE][SAFETY]

- Include encoder count placeholders; heartbeat/seq echo; flags + error_code precedence.

[X] T019. Firmware build/test script [P] [FIRMWARE]

- Script: `firmware/mecabridge_pico/build.sh` building UF2; verify size < target.

### Integration & End-to-End

[X] T020. Serial loopback / fake transport test (host only) [INTEGRATION][PROTOCOL]

- Mock serial class feeding STATE frames; ensure driver updates joint states.

[X] T021. ros2_control controller integration test (spawn mecanum + forward controllers) [INTEGRATION][ROS2-CONTROL]

- Launch file under `test/` verifying interfaces & command flow to protocol encode.

[X] T022. Watchdog end-to-end test (silence) [INTEGRATION][SAFETY]

- Simulate no frames; assert zeroed commands within ≤150 ms.

[X] T023. Latency measurement test (synthetic timing) [INTEGRATION][LATENCY]

- Feed timestamps; assert computed p95 ≤ 20 ms for synthetic distribution.

[X] T024. ESC & servo limit clamp tests [INTEGRATION][SAFETY]

- Over-limit commands → clamped + flag bits set.

### Feature Toggles & Scaling

[X] T025. Implement compile-time feature toggles (macros) [P] [CONFIG]

- Macros: MECABRIDGE_ENABLE_SERVOS, MECABRIDGE_ENABLE_ESCS, MECABRIDGE_ENABLE_DIAGNOSTICS.
- Adjust interface export and frame encode selectively.

T026. Tests for disabled feature builds [CONFIG]

- Build variant disabling ESCs & servos; tests ensure omitted interfaces & reduced frame encode.

T027. Implement ESC PWM scaling + optional deadband [PROTOCOL]

- Convert normalized [-1,1] → PWM µs; account for deadband; unit tests with boundary values.

T028. Continuous servo velocity scaling/clamp [PROTOCOL]

- Clamp by max_velocity_rad_s; test clamping & flag semantics.

### Protocol Version & Safety Flag Tests

T033. Protocol version field presence & integration tests [P] [PROTOCOL]

- Extend existing frame encode tests to assert version byte; add dedicated `src/mecabridge_hardware/test/mecabridge/protocol/test_protocol_version.cpp` verifying both host encode & Pico decode (synthetic) honor constant `PROTOCOL_VERSION`.
- Add integration sub-test in T020 or separate to simulate mismatch (alter expected version) confirming initialization abort.
- Complements handshake logic from T005a; together they fully satisfy FR-012.

- Extend `test_watchdog.cpp` or new `test_watchdog_flag.cpp` (under `src/mecabridge_hardware/test/mecabridge/safety/`) to assert that when watchdog triggers (host silence > SAFE_CUTOFF_MS) the next state frame (simulated) includes SAFETY_EVENT / WATCHDOG_TRIPPED flag bit and error code precedence matches contract (FR-015).
- Ensure firmware side task list (T018) encodes this bit; host side (T022) asserts receipt.

### Documentation & Polish

T029. Add CHANGELOG entry + package.xml version bump (0.1.0) [P] [DOCS]

- Document stability surfaces.

T030. Update README (hardware interface usage, safety, config schema) [P] [DOCS]

- Include YAML sample & watchdog explanation.

T031. Add performance & safety test invocation docs to `quickstart.md` [P] [DOCS]

- Section on running latency & watchdog tests.

T032. Constitution Check audit before merge [P] [GATE]

- Verify principles satisfied; note any deviations.

## Parallel Execution Guidance

- After T005: T006, T007, T008, T009 can proceed in parallel (distinct files).
- Firmware tasks T015, T016, T017, T018 depend on protocol spec (T005) but can progress once encoding fixed.
- Toggle-related tasks (T025–T028) wait until core integration (T013/T014) passes.
- Documentation tasks (T029–T031) can start once interfaces stable (post T013/T021).

## FR Coverage Matrix (Summary)

| FR  | Tasks                                                   |
| --- | ------------------------------------------------------- |
| 001 | T011, T012, T013, T021                                  |
| 002 | T021                                                    |
| 003 | T012, T021                                              |
| 004 | T012, T024                                              |
| 005 | T027, T028, T024, T013                                  |
| 006 | T009, T010, T012                                        |
| 007 | T003, T004, T005, T006, T016                            |
| 008 | T018, T021                                              |
| 009 | T007, T008, T018, T022                                  |
| 010 | T025, T026                                              |
| 011 | T006, T016, T024                                        |
| 012 | T001, T005, T005a, T033                                 |
| 013 | T012, T009                                              |
| 014 | T007, T018, T022                                        |
| 015 | T018 (flag encode), T022, T034                          |
| 016 | Deferred (no tasks)                                     |
| 017 | T014, T023                                              |
| 018 | T012, T027, T028                                        |
| 019 | All protocol & integration tests (T003–T007, T020–T024) |
| 020 | T029, T030, T031                                        |

## Completion Criteria

## Completion Criteria
All tasks T001–T034 (plus insertion T005a) completed, tests green, latency & watchdog integration tests within specified constraints, Constitution Check (T032) passes with no unaddressed violations. FR-012 explicitly validated via T005a + T033; FR-015 via T018 + T022 + T034.

