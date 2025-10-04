<!-- TODO: need update -->
<!-- 
# MecaBridge Hardware Interface

ROS 2 (ros2_control) hardware interface and Pico firmware bridge for a mecanum-base robot with four DC wheels, two servos, and two ESC channels. The project targets a Raspberry Pi 4B (controller manager) connected to a Raspberry Pi Pico (low-level PWM and encoder IO) over a deterministic serial link.

## 1. Scope & Deliverables
- Host-side `MecaBridgeHardware` ros2_control SystemInterface loaded via pluginlib
- Pico firmware providing PWM, encoder, servo, and ESC handling with watchdog safety
- Shared deterministic frame protocol (START_BYTE 0xAA, CRC-16/CCITT-FALSE, 150 ms watchdog window)
- Single YAML configuration source describing geometry, joints, encoder ticks, servo limits, ESC calibration, and feature toggles
- Validation assets: unit tests for CRC/protocol/watchdog, ros2_control integration tests, latency measurement harness (p95 <= 20 ms)

Additional design artifacts live under `specs/001-mecabridge-plan/`.

## 2. Architecture Overview
### 2.1 Host (Raspberry Pi 4B)
- `controller_manager` drives the read -> update -> write loop
- `mecanum_drive_controller` consumes `/cmd_vel` (`geometry_msgs/Twist` or `TwistStamped`) and outputs wheel velocity commands
- `joint_state_broadcaster` publishes wheel, servo, and ESC state interfaces
- Optional forward command controllers expose servo and ESC set-points directly when required
- `MecaBridgeHardware` plugin handles serial communication, state decoding, command encoding, configuration loading, safety, and latency metrics

### 2.2 Firmware (Raspberry Pi Pico)
- PWM generation for TB6612FNG motor driver outputs
- Servo and ESC signal generation with calibration mapping
- Quadrature encoder capture (PIO or equivalent) delivering 32-bit counts per wheel
- Serial transport implementing command/state frames, sequence acknowledgement, CRC verification, and watchdog-triggered safe stop
- Modular source layout (`motors.cpp`, `enc.cpp`, `esc_servo.cpp`, protocol handling TBD) with compile-time feature toggles (`MECABRIDGE_ENABLE_SERVOS`, etc.)

## 3. Communication Protocol
- **Frame layout**: `START_BYTE (0xAA) | FRAME_ID | LENGTH | PAYLOAD | CRC16`
- **Command payload (35 bytes)**: wheel velocities (4 x float32 rad/s), positional servo angle (float32 rad), continuous servo velocity (float32 normalized), ESC outputs (2 x float32 normalized), sequence number (uint16), reserved flags (uint8)
- **State payload (39 bytes)**: encoder counts (4 x uint32), delta time (uint16 ms), servo angle (float32), continuous servo velocity echo (float32), ESC echoes (2 x float32), accepted sequence (uint16), status flags (uint16), error code (uint8)
- **Error codes**: `0 OK`, `1 CRC_FAIL`, `2 MALFORMED_FRAME`, `3 UNKNOWN_FRAME_ID`, `4 VERSION_MISMATCH`, `5 WATCHDOG_TIMEOUT`
- **Status flags** (uint16 bitfield): watchdog triggered, positional servo clamp, CRC error since last, malformed frame since last, encoder stale, reserved future bits

Version handshakes and protocol evolution guidelines are covered in `specs/001-mecabridge-plan/data-model.md`.

## 4. Configuration Schema
All runtime configuration is loaded from a single YAML file referenced in the URDF hardware block.

```yaml
mecabridge_hardware:
  serial_port: /dev/ttyACM0
  baud_rate: 115200
  state_publish_rate_hz: 50
  wheel_radius: 0.048
  wheel_separation_x: 0.20
  wheel_separation_y: 0.18
  encoder_ticks_per_rev: 1024
  wheels:
    front_left:  { encoder_index: 0, joint_name: front_left_wheel_joint }
    front_right: { encoder_index: 1, joint_name: front_right_wheel_joint }
    rear_left:   { encoder_index: 2, joint_name: rear_left_wheel_joint }
    rear_right:  { encoder_index: 3, joint_name: rear_right_wheel_joint }
  servos:
    positional:
      joint_name: pan_servo_joint
      min_rad: -3.14159
      max_rad: 3.14159
    continuous:
      joint_name: rot_servo_joint
      max_velocity_rad_s: 10.0
  escs:
    left:
      joint_name: esc_left_joint
      esc_min_pwm: 1000
      esc_max_pwm: 2000
      esc_deadband: 50
    right:
      joint_name: esc_right_joint
      esc_min_pwm: 1000
      esc_max_pwm: 2000
      esc_deadband: 50
  features:
    enable_servos: true
    enable_escs: true
    enable_diagnostics: false
```

Validation rules (unique joint names, positive radii, etc.) are listed in `specs/001-mecabridge-plan/data-model.md`.

## 5. ros2_control Integration
Embed the hardware in URDF/Xacro:

```xml
<ros2_control name="MecaBridgeSystem" type="system">
  <hardware>
    <plugin>mecabridge_hardware/MecaBridgeHardware</plugin>
    <param name="config_file">package://mecabridge_hardware/config/mecabridge.yaml</param>
  </hardware>
  <joint name="front_left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

Controller example (`config/controllers.yaml`):

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    mecanum_controller:
      type: mecanum_drive_controller/MecanumDriveController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    servo_forward_controller:
      type: forward_command_controller/ForwardCommandController
      joints: [pan_servo_joint]
    misc_forward_controller:
      type: forward_command_controller/ForwardCommandController
      joints: [rot_servo_joint, esc_left_joint, esc_right_joint]

mecanum_controller:
  ros__parameters:
    front_left_wheel_command_joint_name: front_left_wheel_joint
    front_right_wheel_command_joint_name: front_right_wheel_joint
    rear_left_wheel_command_joint_name: rear_left_wheel_joint
    rear_right_wheel_command_joint_name: rear_right_wheel_joint
    kinematics:
      wheels_radius: 0.048
      sum_of_robot_center_projection_on_X_Y_axis: 0.38
    base_frame_id: base_link
    odom_frame_id: odom
    enable_odom_tf: true
```
Launch flow (pseudo): load robot description, start `ros2_control_node`, spawn broadcasters/controllers, start latency diagnostics if required.

## 6. Firmware Bring-Up
1. Build Pico firmware (`firmware/mecabridge_pico/`) with matching `PROTOCOL_VERSION`.
2. Flash via USB; firmware should initiate serial heartbeats immediately.
3. Confirm watchdog safe-stop behaviour: on host disconnect (>150 ms gap) all PWM outputs go to zero and flags indicate `WATCHDOG_TRIGGERED`.

Current sources (`term.cpp`, `motors.cpp`, etc.) still expose a command-line harness; upcoming work replaces this with the deterministic frame handler (see TODO list below).

## 7. Safety, Diagnostics & Performance
- Watchdog timeout: 150 ms between valid frames -> safe stop + flag bit
- CRC-16/CCITT-FALSE guarantees frame integrity; CRC failures increment status flags and raise error code 1
- Diagnostic publisher (planned) reports battery and communication health using ROS diagnostics
- Latency budget: <= 20 ms p95 from `/cmd_vel` reception to PWM update; integration tests measure RTT via sequence echo

## 8. Testing Strategy
- **Unit tests**: CRC vectors, frame encode/decode, error-path coverage, watchdog timing (see `specs/001-mecabridge-plan/tasks.md` tasks T003-T008)
- **Integration tests**: ros2_control interface registration, configuration validation, safety flag propagation, protocol version handshake (tasks T011-T024)
- **Performance tests**: synthetic latency harness ensuring p95 <= 20 ms, ESC/servo clamp verification

Test ordering is defined in `specs/001-mecabridge-plan/tasks.md` to preserve a test-first workflow.

## 9. Repository Layout
```
src/mecabridge_hardware/
  include/mecabridge_hardware/    # SystemInterface headers + protocol contracts
  src/mecabridge_hardware/        # ros2_control implementation and utilities
  src/mecabridge_utils/           # protocol, CRC, safety helpers
  test/mecabridge/                # GTest + integration harnesses
  config/                         # YAML templates & sample configs
  launch/                         # bringup + diagnostic launches

firmware/mecabridge_pico/
  src/                            # Pico firmware modules (PWM, encoder, protocol)
  include/                        # Firmware headers (if added)
  tests/                          # Firmware unit/integration tests (planned)
```

## 10. Current Status & Priority TODOs
- Spec and design artifacts (Phase 0/1) completed; execution phases (task generation, implementation, validation) pending (see `specs/001-mecabridge-plan/plan.md`)
- Protocol utilities (`frame.cpp`, `crc16.cpp`) implemented with unit tests
- `MecaBridgeHardware` plugin scaffolding, YAML loader, safety integration, and ros2_control wiring still outstanding (tasks T005a onward)
- Pico firmware currently runs terminal demo (`firmware/mecabridge_pico/src/term.cpp`); needs refactor to real protocol loop with watchdog and sequence handling
- Pluginlib export and package docs to be finalized once hardware implementation lands

Refer to `specs/001-mecabridge-plan/tasks.md` for the authoritative task list and gating criteria.

## 11. References
- Design artifacts & contracts: `specs/001-mecabridge-plan/`
- ROS control docs: <https://control.ros.org/>
- Local ROS 2 control docs: `.docs/ros2_control/`
- micro-ROS on Raspberry Pi Pico resources (see Implementierung_raspberry pi.md for annotated links)

---

# MecaBridge Hardware Interface (Deutsch)

ROS 2 (ros2_control) Hardware-Interface und Pico-Firmware-Bruecke fuer einen Mecanum-Roboter mit vier DC-Raedern, zwei Servos und zwei ESC-Kanaelen. Zielplattform ist ein Raspberry Pi 4B (Controller-Manager), der ueber eine deterministische serielle Verbindung mit einem Raspberry Pi Pico (PWM- und Encoder-I/O) gekoppelt ist.

## 1. Umfang & Deliverables
- Hostseitiges ros2_control SystemInterface `MecaBridgeHardware`, geladen via pluginlib
- Pico-Firmware mit PWM-, Encoder-, Servo- und ESC-Ansteuerung inklusive Watchdog-Sicherheit
- Gemeinsames deterministisches Frame-Protokoll (START_BYTE 0xAA, CRC-16/CCITT-FALSE, 150 ms Watchdog-Fenster)
- Zentrale YAML-Konfiguration fuer Geometrie, Joints, Encoder-Ticks, Servo-Limits, ESC-Kalibrierung und Feature-Toggles
- Validierungsartefakte: Unit-Tests fuer CRC/Protokoll/Watchdog, ros2_control Integrationstests, Latenz-Mess-Harness (p95 <= 20 ms)

Weitere Designartefakte finden sich unter `specs/001-mecabridge-plan/`.

## 2. Architekturueberblick
### 2.1 Host (Raspberry Pi 4B)
- `controller_manager` uebernimmt den read -> update -> write Zyklus
- `mecanum_drive_controller` verarbeitet `/cmd_vel` (`geometry_msgs/Twist` oder `TwistStamped`) und erzeugt Radgeschwindigkeiten
- `joint_state_broadcaster` veroeffentlicht Rad-, Servo- und ESC-Zustaende
- Optionale Forward Command Controller stellen direkte Sollwertschnittstellen fuer Servos und ESCs bereit
- Das `MecaBridgeHardware` Plugin kuemmert sich um serielle Kommunikation, Zustandsdekodierung, Kommando-Encoding, Konfigurations-Loading, Safety-Handling und Latenzmetriken

### 2.2 Firmware (Raspberry Pi Pico)
- PWM-Erzeugung fuer den TB6612FNG-Motortreiber
- Servo- und ESC-Signale mit Kalibrierungsmapping
- Quadratur-Encoder-Erfassung (PIO o.ae.) mit 32-bit Zaehlern pro Rad
- Serielle Transportschicht fuer Command/State Frames, Sequenzbestaetigung, CRC-Pruefung und Watchdog-Safe-Stop
- Modulare Quellstruktur (`motors.cpp`, `enc.cpp`, `esc_servo.cpp`, Protokoll-Handler TBD) mit Compile-Time-Feature-Toggles (`MECABRIDGE_ENABLE_SERVOS`, etc.)

## 3. Kommunikationsprotokoll
- **Frame-Aufbau**: `START_BYTE (0xAA) | FRAME_ID | LENGTH | PAYLOAD | CRC16`
- **Command-Payload (35 Bytes)**: Radgeschwindigkeiten (4 x float32 rad/s), positionsservo-Winkel (float32 rad), kontinuierlicher Servo (float32 normiert), ESC-Werte (2 x float32 normiert), Sequenznummer (uint16), reservierte Flags (uint8)
- **State-Payload (39 Bytes)**: Encoder-Zaehler (4 x uint32), Delta-Zeit (uint16 ms), Servo-Winkel (float32), Servo-velocity-Echo (float32), ESC-Echos (2 x float32), angenommene Sequenz (uint16), Status-Flags (uint16), Fehlercode (uint8)
- **Fehlercodes**: `0 OK`, `1 CRC_FAIL`, `2 MALFORMED_FRAME`, `3 UNKNOWN_FRAME_ID`, `4 VERSION_MISMATCH`, `5 WATCHDOG_TIMEOUT`
- **Status-Flags** (uint16 Bitfeld): Watchdog ausgeloest, Servo-Clamp, CRC-Fehler seit letztem Frame, Malformed-Frame, Encoder stale, Reserven

Hinweise zu Versionshandshake und Protokollerweiterung stehen in `specs/001-mecabridge-plan/data-model.md`.

## 4. Konfigurationsschema
Alle Laufzeitparameter werden aus einer YAML-Datei geladen, die im URDF-Hardwareblock referenziert wird (siehe Beispiel im englischen Abschnitt). Validierungsregeln (eindeutige Joint-Namen, positive Radien usw.) sind in `specs/001-mecabridge-plan/data-model.md` dokumentiert.

## 5. ros2_control Integration
- Hardware-Plugin im URDF/Xacro einbinden (`<plugin>mecabridge_hardware/MecaBridgeHardware</plugin>` plus `config_file` Parameter)
- Controller-Parameter in `config/controllers.yaml` (mecanum_controller, joint_state_broadcaster, Forward-Controller)
- Launchflow: Robot Description laden, `ros2_control_node` starten, Controller spawnen, optionale Latenzdiagnostik aktivieren

## 6. Firmware-Inbetriebnahme
1. Pico-Firmware unter `firmware/mecabridge_pico/` mit passender `PROTOCOL_VERSION` bauen
2. Via USB flashen; Firmware sollte sofort Heartbeat-Frames senden
3. Watchdog pruefen: Host-Verbindung >150 ms unterbrechen, alle PWM-Signale muessen auf 0 gehen und das Flag `WATCHDOG_TRIGGERED` setzen

Derzeit laeuft noch das Terminal-Harness (`term.cpp`); naechster Schritt ist der vollstaendige Frame-Handler mit Watchdog- und Sequenzlogik.

## 7. Safety, Diagnostik & Performance
- Watchdog-Timeout: 150 ms zwischen gueltigen Frames -> Safe Stop + Flagbit
- CRC-16/CCITT-FALSE garantiert Integritaet; Fehler inkrementieren Statusflags und setzen Fehlercode 1
- Geplante Diagnosen: Battery- und Kommunikationsstatus via ROS diagnostics
- Latenz-Ziel: <= 20 ms p95 vom `/cmd_vel`-Eingang bis zur PWM-Aktualisierung (Messung ueber Sequenz-Echo)

## 8. Teststrategie
- **Unit-Tests**: CRC-Vektoren, Frame Encode/Decode, Fehlerpfade, Watchdog-Timing (Tasks T003-T008)
- **Integrationstests**: ros2_control Interface-Registrierung, Konfigurationsvalidierung, Safety-Flag Propagation, Protokoll-Version (Tasks T011-T024)
- **Performance-Tests**: Latenz-Harness (p95 <= 20 ms), ESC/Servo-Clamping

## 9. Repository-Struktur
```
src/mecabridge_hardware/
  include/mecabridge_hardware/
  src/mecabridge_hardware/
  src/mecabridge_utils/
  test/mecabridge/
  config/
  launch/

firmware/mecabridge_pico/
  src/
  include/
  tests/
```

## 10. Aktueller Status & ToDos
- Spezifikation und Design (Phase 0/1) fertig; Umsetzung, Tests und Gate-Pruefungen stehen aus (`specs/001-mecabridge-plan/plan.md`)
- Protokoll-Hilfsfunktionen vorhanden; restliche Tasks ab T005a offen (`specs/001-mecabridge-plan/tasks.md`)
- Pico-Firmware muss auf Protokollbetrieb umgestellt werden (Watchdog, Sequenzen, CRC)
- pluginlib-Export und Dokumentation abschliessen, sobald das Hardware-Plugin implementiert ist

## 11. Referenzen
- Design & Contracts: `specs/001-mecabridge-plan/`
- ROS control Dokumentation: <https://control.ros.org/>
- Lokale ROS 2 control Doku: `.docs/ros2_control/`
- micro-ROS Ressourcen: `Implementierung_raspberry pi.md`



docker compose exec -T ros2_drive_dev bash -c "colcon build --packages-select drive_arduino mecabridge_hardware && colcon test --packages-select mecabridge_hardware && colcon test-result --all --verbose" *> test_results_5.log -->