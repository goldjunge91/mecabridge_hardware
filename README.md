# MecaBridge Hardware Interface Documentation

## Overview

The MecaBridge Hardware Interface is a ROS 2 `SystemInterface` implementation that provides a bridge between high-level
ROS 2 control systems and low-level Raspberry Pi Pico firmware for robotics platforms. The system supports multiple
drive configurations (differential, mecanum, four-wheel) with deterministic serial communication and safety-first design
principles.

### Key Features

- **Multi-Drive Support**: Differential, Mecanum, and Four-Wheel drive configurations
- **Safety-First Design**: Watchdog-enforced failsafes with ≤150ms timeout
- **Deterministic Communication**: CRC-16/CCITT-FALSE validated serial protocol
- **ROS 2 Integration**: Full `hardware_interface::SystemInterface` compliance
- **Error Recovery**: Automatic connection recovery and error handling
- **Testing Support**: Mock injection for comprehensive unit testing

## Architecture

### Component Hierarchy

```
MecaBridgeHardwareInterface (SystemInterface)
├── MecaBridgeSerialProtocol (Serial Protocol Layer)
├── MecaBridgeDriveConfig (Parameter Management)
└── Wheel[] (Joint State Management)
```

### Core Components

#### MecaBridgeHardwareInterface

The main ROS 2 hardware interface that implements the `SystemInterface` lifecycle. This class manages:

- Lifecycle Management: Initialization, activation, deactivation
- State/Command Interfaces: Exporting joint states and accepting velocity commands
- Read/Write Operations: Synchronizing with the hardware at the configured loop rate
- Error Handling: Connection recovery and error tracking
- Drive Type Abstraction: Runtime switching between drive configurations

**Key Methods:**

- `on_init()`: Hardware initialization and parameter validation
- `export_state_interfaces()`: Expose joint positions, velocities, efforts
- `export_command_interfaces()`: Accept velocity commands for joints
- `read()`: Read encoder data and update joint states
- `write()`: Send velocity commands to motors

#### MecaBridgeSerialProtocol

Handles all serial communication with the Raspberry Pi Pico firmware. Features include:

- **Connection Management**: Auto-detection of serial ports, reconnection logic
- **Command Transmission**: Motor velocity commands for different drive types
- **Encoder Reading**: Optional encoder data retrieval
- **Error Statistics**: Tracking of communication errors and reconnection attempts
- **Testing Support**: Mock serial injection for unit tests

**Supported Commands:**

- `setDifferentialMotors()`: "V {left} {right}\n"
- `setFourMotors()`: "M {fl} {fr} {rl} {rr}\n"
- `readDifferentialEncoders()`: "E\n" → "left right"
- `readFourEncoders()`: "E\n" → "fl fr rl rr"

#### MecaBridgeDriveConfig

Configuration management structure containing all hardware parameters:

- **Drive Configuration**: Drive type, encoder presence, wheel names
- **Communication Parameters**: Serial device, baud rate, timeout
- **Velocity Limits**: Maximum linear and angular velocities
- **Drive-Specific Parameters**: Wheel base, track width, mixing factors
- **Encoder Parameters**: Counts per revolution, wheel radius

#### Wheel

Represents individual wheel joints with state and command tracking:

- **State Variables**: Position, velocity, effort, encoder counts
- **Command Variables**: Velocity setpoint
- **Calculations**: Encoder angle conversion, rads per count

## Communication Protocol

### Frame Structure

The system uses a binary protocol with the following frame format:

```
[START_BYTE][FRAME_ID][LEN][PAYLOAD...][CRC16]
```

- **START_BYTE**: 0xAA (170)
- **FRAME_ID**: Command (0x01), State (0x02), Heartbeat (0x03), etc.
- **LEN**: Payload length in bytes
- **PAYLOAD**: Frame-specific data (up to 64 bytes)
- **CRC16**: CRC-16/CCITT-FALSE checksum

### Command Frame Payload (36 bytes)

```cpp
struct CommandFramePayload {
    float wheel_vel_rad_s[4];       // FL, FR, RL, RR order
    float servo_pos_rad;            // Positional servo target
    float servo_cont_vel_norm;      // Continuous servo velocity (-1..1)
    float esc_norm[2];              // ESC normalized [-1..1]
    uint16_t seq;                   // Sequence number
    uint8_t protocol_version;       // Protocol version
    uint8_t reserved_flags;         // Future use (0)
};
```

### State Frame Payload (40 bytes)

```cpp
struct StateFramePayload {
    uint32_t encoder_counts[4];     // Raw absolute counters
    uint16_t dt_ms;                 // Elapsed ms since previous state frame
    float servo_pos_rad;            // Current servo position
    float servo_cont_vel_norm;      // Echo last commanded
    float esc_norm[2];              // Echo last commanded normalized
    uint16_t seq_echo;              // Last accepted COMMAND seq
    uint16_t flags;                 // Status flags bitfield
    uint8_t error_code;             // Recent error cause
    uint8_t protocol_version;       // Protocol version of the firmware
};
```

### CRC-16 Implementation

- **Polynomial**: 0x1021
- **Initial Value**: 0xFFFF
- **No Reflection**: Input and output not reflected
- **No XorOut**: Final XOR value is 0x0000
- **Test Vector**: "123456789" → 0x29B1

## Configuration

### YAML Configuration Structure

```yaml
mecabridge_hardware_node:
  ros__parameters:
    # Drive configuration
    drive_type: "differential"  # Options: differential, four_wheel, mecanum
    has_encoders: false

    # Communication
    device: "/dev/ttyUSB0"
    baud_rate: 115200
    timeout: 50

    # Velocity limits
    max_lin_vel: 0.3  # m/s
    max_ang_vel: 1.0  # rad/s

    # Drive-specific parameters
    wheel_base: 0.3    # For mecanum drive
    track_width: 0.3   # For mecanum drive
    mix_factor: 0.5    # For differential drive

    # Encoder parameters (if has_encoders: true)
    enc_counts_per_rev: 1920
    wheel_radius: 0.05
```

### Drive Type Configuration

#### Differential Drive

- **Joints**: `left_wheel`, `right_wheel`
- **Commands**: Linear and angular velocity mixing
- **Encoders**: Optional left/right encoder reading

#### Four Wheel Drive

- **Joints**: `front_left_wheel`, `front_right_wheel`, `rear_left_wheel`, `rear_right_wheel`
- **Commands**: Independent wheel velocities
- **Encoders**: Individual wheel encoder reading

#### Mecanum Drive

- **Joints**: `front_left_wheel`, `front_right_wheel`, `rear_left_wheel`, `rear_right_wheel`
- **Commands**: Mecanum kinematics transformation
- **Encoders**: Individual wheel encoder reading

## Safety Features

### Watchdog System

- **Timeout**: ≤150ms between valid frames
- **Behavior**: Zero velocities on timeout
- **Recovery**: Automatic when valid frames resume
- **Status**: Reported in state frame flags

### Error Hierarchy

1. **WATCHDOG_TIMEOUT**: Communication loss detected
2. **CRC_FAIL**: Message integrity validation failed
3. **MALFORMED_FRAME**: Invalid frame structure
4. **VERSION_MISMATCH**: Protocol version incompatibility

### Connection Recovery

- **Auto-Detection**: Scans common serial port candidates
- **Reconnection Logic**: Attempts recovery on communication failures
- **Error Statistics**: Tracks connection, read, and write errors
- **Graceful Degradation**: Continues operation when possible

## Usage Examples

### Launch Configuration

```python
# launch/mecabridge_differential.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecabridge_hardware',
            executable='mecabridge_hardware_node',
            name='mecabridge_hardware',
            parameters=[{
                'drive_type': 'differential',
                'device': '/dev/ttyUSB0',
                'baud_rate': 115200,
                'max_lin_vel': 0.3,
                'max_ang_vel': 1.0
            }]
        )
    ])
```

### Controller Configuration

```yaml
# config/mecabridge_differential_controller.yaml
controller_manager:
  ros__parameters:
    update_rate: 20  # Hz

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

diff_drive_controller:
  ros__parameters:
    left_wheel_names: ['left_wheel']
    right_wheel_names: ['right_wheel']
    wheels_per_side: 1
    wheel_separation: 0.3
    wheel_radius: 0.05
```

### Testing

#### Unit Tests

```bash
# Run all unit tests
colcon test --packages-select mecabridge_hardware --ctest-args -R test_mecabridge

# Run specific test
colcon test --packages-select mecabridge_hardware --ctest-args -R test_mecabridge_comms
```

#### Integration Tests

```bash
# Hardware-in-the-loop testing
ros2 launch mecabridge_hardware test_mecabridge_hardware.launch.py

# Mock testing
colcon test --packages-select mecabridge_hardware --ctest-args -R test_mecabridge_integration
```

## API Reference

### MecaBridgeHardwareInterface API

#### Public Methods

- `hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info)`
    - Initializes hardware with ROS parameters
    - Returns SUCCESS on successful initialization

- `std::vector<hardware_interface::StateInterface> export_state_interfaces()`
    - Returns state interfaces for all configured joints
    - Includes position, velocity, and effort for each wheel

- `std::vector<hardware_interface::CommandInterface> export_command_interfaces()`
    - Returns command interfaces for velocity control
    - One command interface per wheel joint

- `hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period)`
    - Reads current joint states from hardware
    - Updates encoder positions and velocities

- `hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period)`
    - Sends velocity commands to hardware
    - Converts rad/s to motor commands

#### Private Methods

- `bool attemptConnectionRecovery()`
    - Attempts to recover from communication failures
    - Returns true if recovery successful

- `int convertVelocityToMotorCommand(double wheel_vel_rad_s)`
    - Converts wheel velocity to motor command value
    - Scales and clamps to valid motor range

### MecaBridgeSerialProtocol API

#### Public Methods

- `void setup(const std::string& serial_device, int32_t baud_rate, int32_t timeout_ms)`
    - Configures serial connection parameters

- `void sendPing()`
    - Sends ping command for Arduino synchronization

- `void setDifferentialMotors(int left_val, int right_val)`
    - Sends velocity commands for differential drive

- `void setFourMotors(int fl, int fr, int rl, int rr)`
    - Sends velocity commands for four-wheel drive

- `bool connected() const`
    - Returns current connection status

- `bool attemptReconnection()`
    - Attempts to reconnect to serial device

- `void getConnectionStats(int& write_errors, int& read_errors, int& reconnection_attempts) const`
    - Retrieves error statistics

### MecaBridgeDriveConfig API

#### Configuration Parameters

- `DriveType drive_type`: Drive configuration type
- `bool has_encoders`: Whether encoders are present
- `std::string device`: Serial device path
- `int baud_rate`: Serial baud rate
- `int timeout`: Serial timeout in milliseconds
- `float max_lin_vel`: Maximum linear velocity (m/s)
- `float max_ang_vel`: Maximum angular velocity (rad/s)

## Error Handling

### Error Codes

- `OK = 0`: No error
- `CRC_FAIL = 1`: CRC validation failed
- `MALFORMED_FRAME = 2`: Invalid frame structure
- `UNKNOWN_FRAME_ID = 3`: Unrecognized frame ID
- `VERSION_MISMATCH = 4`: Protocol version mismatch
- `WATCHDOG_TIMEOUT = 5`: Watchdog timer expired

### Status Flags

- `WATCHDOG_TRIGGERED = 0x0001`: Watchdog has tripped
- `POSITIONAL_SERVO_LIMIT_HIT = 0x0002`: Servo position limit reached
- `CRC_ERROR_SINCE_LAST = 0x0004`: CRC error occurred since last frame
- `MALFORMED_ERROR_SINCE_LAST = 0x0008`: Malformed frame since last
- `ENCODER_STALE = 0x0010`: Encoder data is stale

## Testing and Validation

### Unit Testing

The system includes comprehensive unit tests with mock injection:

- **Mock Serial**: `MockSerial` class for testing without hardware
- **Protocol Tests**: Frame parsing, CRC validation, error handling
- **Communication Tests**: Command transmission, encoder reading
- **Hardware Interface Tests**: ROS interface compliance, state management

### Integration Testing

- **Hardware-in-the-Loop**: Tests with actual Pico firmware
- **ROS Integration**: Full ROS 2 control stack testing
- **Performance Testing**: Latency and throughput validation

## Best Practices

### Development

1. **Namespace Convention**: All classes use `mecabridge_hardware` namespace
2. **Error Handling**: Always check return values and handle errors gracefully
3. **Testing Mode**: Use `#ifdef TESTING_MODE` for mock injection
4. **Documentation**: Update this documentation when adding new features

### Deployment

1. **Serial Permissions**: Ensure user has access to serial ports
2. **Parameter Validation**: Validate all configuration parameters
3. **Monitoring**: Monitor error statistics and connection status
4. **Backup Communication**: Have fallback communication methods

### Safety

1. **Watchdog Monitoring**: Always monitor watchdog status
2. **Velocity Limits**: Respect configured velocity limits
3. **Emergency Stop**: Implement emergency stop functionality
4. **Fail-Safe Behavior**: Define safe behavior for all error conditions

## Troubleshooting

### Common Issues

1. **Serial Connection Failed**
    - Check device permissions: `sudo usermod -a -G dialout $USER`
    - Verify device path and baud rate
    - Check USB connection and Pico firmware

2. **Watchdog Timeouts**
    - Verify loop rate matches firmware expectations
    - Check serial communication latency
    - Monitor CPU usage and timing

3. **Encoder Data Issues**
    - Verify encoder configuration in firmware
    - Check encoder wiring and power
    - Validate encoder counts per revolution

4. **Velocity Control Problems**
    - Verify joint names match between hardware and controller
    - Check velocity limits and scaling
    - Monitor command transmission

### Debug Tools

- **Error Statistics**: Use `getConnectionStats()` for diagnostics
- **Logging**: Enable ROS logging for detailed information
- **Mock Testing**: Use mock serial for isolated testing
- **Protocol Analysis**: Monitor serial traffic for protocol issues

## Future Enhancements

- **Advanced Kinematics**: Support for custom drive configurations
- **Sensor Integration**: IMU, GPS, and other sensor support
- **Advanced Safety**: Multi-level safety systems and redundancy
- **Performance Optimization**: Reduced latency and improved throughput
- **Configuration Tools**: GUI configuration and validation tools

---

*This documentation is maintained alongside the codebase. Please update it when making significant changes to the
system.*

# MecaBridge Hardware Interface Dokumentation

## Übersicht

Das MecaBridge Hardware Interface ist eine ROS 2 `SystemInterface`-Implementierung, die eine Brücke zwischen hochlevel
ROS 2-Steuerungssystemen und niederlevel Raspberry Pi Pico-Firmware für Robotik-Plattformen bereitstellt. Das System
unterstützt mehrere Antriebskonfigurationen (Differential, Mecanum, Vier-Rad) mit deterministischer serieller
Kommunikation und sicherheitsorientierten Design-Prinzipien.

### Hauptmerkmale

- **Multi-Antriebs-Unterstützung**: Differential-, Mecanum- und Vier-Rad-Antriebskonfigurationen
- **Sicherheitsorientiertes Design**: Watchdog-gesicherte Fail-Safes mit ≤150ms Timeout
- **Deterministische Kommunikation**: CRC-16/CCITT-FALSE validiertes serielles Protokoll
- **ROS 2 Integration**: Vollständige `hardware_interface::SystemInterface`-Compliance
- **Fehlerwiederherstellung**: Automatische Verbindungs- und Fehlerbehandlung
- **Test-Unterstützung**: Mock-Injection für umfassende Unit-Tests

## Architektur

### Komponentenhierarchie

```
MecaBridgeHardwareInterface (SystemInterface)
├── MecaBridgeSerialProtocol (Serial Protocol Layer)
├── MecaBridgeDriveConfig (Parameter Management)
└── Wheel[] (Joint State Management)
```

### Kernkomponenten

#### MecaBridgeHardwareInterface

Die Haupt-ROS 2 Hardware-Schnittstelle, die den `SystemInterface`-Lebenszyklus implementiert. Diese Klasse verwaltet:

- Lebenszyklus-Verwaltung: Initialisierung, Aktivierung, Deaktivierung
- State/Command Interfaces: Export von Joint-Zuständen und Annahme von Geschwindigkeitsbefehlen
- Read/Write-Operationen: Synchronisation mit der Hardware bei konfigurierter Schleifenrate
- Fehlerbehandlung: Verbindungs-Wiederherstellung und Fehler-Tracking
- Antriebs-Typ-Abstraktion: Laufzeit-Umschaltung zwischen Antriebskonfigurationen

**Wichtige Methoden:**

- `on_init()`: Hardware-Initialisierung und Parameter-Validierung
- `export_state_interfaces()`: Belichtet Joint-Positionen, -Geschwindigkeiten, -Efforts
- `export_command_interfaces()`: Nimmt Geschwindigkeitsbefehle für Joints an
- `read()`: Liest Encoder-Daten und aktualisiert Joint-Zustände
- `write()`: Sendet Geschwindigkeitsbefehle an Motoren

#### MecaBridgeSerialProtocol

Verwaltet die gesamte serielle Kommunikation mit der Raspberry Pi Pico-Firmware. Features umfassen:

- **Verbindungs-Verwaltung**: Auto-Erkennung serieller Ports, Reconnection-Logik
- **Befehls-Übertragung**: Motor-Geschwindigkeitsbefehle für verschiedene Antriebstypen
- **Encoder-Lesung**: Optionale Encoder-Daten-Abfrage
- **Fehler-Statistiken**: Tracking von Kommunikationsfehlern und Reconnection-Versuchen
- **Test-Unterstützung**: Mock-Serial-Injection für Unit-Tests

**Unterstützte Befehle:**

- `setDifferentialMotors()`: "V {left} {right}\n"
- `setFourMotors()`: "M {fl} {fr} {rl} {rr}\n"
- `readDifferentialEncoders()`: "E\n" → "left right"
- `readFourEncoders()`: "E\n" → "fl fr rl rr"

#### MecaBridgeDriveConfig

Konfigurations-Verwaltungsstruktur mit allen Hardware-Parametern:

- **Antriebs-Konfiguration**: Antriebstyp, Encoder-Anwesenheit, Rad-Namen
- **Kommunikations-Parameter**: Serielles Gerät, Baudrate, Timeout
- **Geschwindigkeits-Limits**: Maximale lineare und angulare Geschwindigkeiten
- **Antriebsspezifische Parameter**: Rad-Basis, Spurbreite, Mischfaktoren
- **Encoder-Parameter**: Counts pro Umdrehung, Rad-Radius

#### Wheel

Repräsentiert individuelle Rad-Joints mit State- und Command-Tracking:

- **State-Variablen**: Position, Geschwindigkeit, Effort, Encoder-Counts
- **Command-Variablen**: Geschwindigkeits-Sollwert
- **Berechnungen**: Encoder-Winkel-Konvertierung, rads pro Count

## Kommunikationsprotokoll

### Frame-Struktur

Das System verwendet ein binäres Protokoll mit folgendem Frame-Format:

```
[START_BYTE][FRAME_ID][LEN][PAYLOAD...][CRC16]
```

- **START_BYTE**: 0xAA (170)
- **FRAME_ID**: Command (0x01), State (0x02), Heartbeat (0x03), etc.
- **LEN**: Payload-Länge in Bytes
- **PAYLOAD**: Frame-spezifische Daten (bis zu 64 Bytes)
- **CRC16**: CRC-16/CCITT-FALSE Checksumme

### Command Frame Payload (36 Bytes)

```cpp
struct CommandFramePayload {
    float wheel_vel_rad_s[4];       // FL, FR, RL, RR Reihenfolge
    float servo_pos_rad;            // Positioneller Servo-Zielwert
    float servo_cont_vel_norm;      // Kontinuierlicher Servo-Geschwindigkeit (-1..1)
    float esc_norm[2];              // ESC normalisiert [-1..1]
    uint16_t seq;                   // Sequenznummer
    uint8_t protocol_version;       // Protokoll-Version
    uint8_t reserved_flags;         // Zukünftige Verwendung (0)
};
```

### State Frame Payload (40 Bytes)

```cpp
struct StateFramePayload {
    uint32_t encoder_counts[4];     // Roh absolute Zähler
    uint16_t dt_ms;                 // Verstrichene ms seit vorherigem State-Frame
    float servo_pos_rad;            // Aktuelle Servo-Position
    float servo_cont_vel_norm;      // Echo letzter befohlener
    float esc_norm[2];              // Echo letzter befohlener normalisiert
    uint16_t seq_echo;              // Letzte akzeptierte COMMAND seq
    uint16_t flags;                 // Status-Flags Bitfeld
    uint8_t error_code;             // Jüngste Fehlerursache
    uint8_t protocol_version;       // Protokoll-Version der Firmware
};
```

### CRC-16 Implementierung

- **Polynom**: 0x1021
- **Initialwert**: 0xFFFF
- **Keine Reflexion**: Input und Output nicht reflektiert
- **Kein XorOut**: Finale XOR-Wert ist 0x0000
- **Test-Vektor**: "123456789" → 0x29B1

## Konfiguration

### YAML-Konfigurationsstruktur

```yaml
mecabridge_hardware_node:
  ros__parameters:
    # Antriebs-Konfiguration
    drive_type: "differential"  # Optionen: differential, four_wheel, mecanum
    has_encoders: false

    # Kommunikation
    device: "/dev/ttyUSB0"
    baud_rate: 115200
    timeout: 50

    # Geschwindigkeits-Limits
    max_lin_vel: 0.3  # m/s
    max_ang_vel: 1.0  # rad/s

    # Antriebsspezifische Parameter
    wheel_base: 0.3    # Für Mecanum-Antrieb
    track_width: 0.3   # Für Mecanum-Antrieb
    mix_factor: 0.5    # Für Differential-Antrieb

    # Encoder-Parameter (falls has_encoders: true)
    enc_counts_per_rev: 1920
    wheel_radius: 0.05
```

### Antriebs-Typ-Konfiguration

#### Differential-Antrieb

- **Joints**: `left_wheel`, `right_wheel`
- **Befehle**: Lineare und angulare Geschwindigkeits-Mischung
- **Encoder**: Optionale Links/Rechts Encoder-Lesung

#### Vier-Rad-Antrieb

- **Joints**: `front_left_wheel`, `front_right_wheel`, `rear_left_wheel`, `rear_right_wheel`
- **Befehle**: Unabhängige Rad-Geschwindigkeiten
- **Encoder**: Individuelle Rad-Encoder-Lesung

#### Mecanum-Antrieb

- **Joints**: `front_left_wheel`, `front_right_wheel`, `rear_left_wheel`, `rear_right_wheel`
- **Befehle**: Mecanum-Kinematik-Transformation
- **Encoder**: Individuelle Rad-Encoder-Lesung

## Sicherheitsfeatures

### Watchdog-System

- **Timeout**: ≤150ms zwischen gültigen Frames
- **Verhalten**: Null-Geschwindigkeiten bei Timeout
- **Wiederherstellung**: Automatisch wenn gültige Frames wiederaufgenommen werden
- **Status**: Berichtet in State-Frame-Flags

### Fehlerhierarchie

1. **WATCHDOG_TIMEOUT**: Kommunikationsverlust erkannt
2. **CRC_FAIL**: Nachrichtenintegritäts-Validierung fehlgeschlagen
3. **MALFORMED_FRAME**: Ungültige Frame-Struktur
4. **VERSION_MISMATCH**: Protokoll-Versions-Inkompatibilität

### Verbindungs-Wiederherstellung

- **Auto-Erkennung**: Scannt gängige serielle Port-Kandidaten
- **Reconnection-Logik**: Versucht Wiederherstellung bei Kommunikationsfehlern
- **Fehler-Statistiken**: Verfolgt Verbindung, Lese- und Schreibfehler
- **Graceful Degradation**: Setzt Betrieb fort wenn möglich

## Verwendungsbeispiele

### Launch-Konfiguration

```python
# launch/mecabridge_differential.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecabridge_hardware',
            executable='mecabridge_hardware_node',
            name='mecabridge_hardware',
            parameters=[{
                'drive_type': 'differential',
                'device': '/dev/ttyUSB0',
                'baud_rate': 115200,
                'max_lin_vel': 0.3,
                'max_ang_vel': 1.0
            }]
        )
    ])
```

### Controller-Konfiguration

```yaml
# config/mecabridge_differential_controller.yaml
controller_manager:
  ros__parameters:
    update_rate: 20  # Hz

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

diff_drive_controller:
  ros__parameters:
    left_wheel_names: ['left_wheel']
    right_wheel_names: ['right_wheel']
    wheels_per_side: 1
    wheel_separation: 0.3
    wheel_radius: 0.05
```

### Tests

#### Unit-Tests

```bash
# Alle Unit-Tests ausführen
colcon test --packages-select mecabridge_hardware --ctest-args -R test_mecabridge

# Spezifischen Test ausführen
colcon test --packages-select mecabridge_hardware --ctest-args -R test_mecabridge_comms
```

#### Integration-Tests

```bash
# Hardware-in-the-Loop-Tests
ros2 launch mecabridge_hardware test_mecabridge_hardware.launch.py

# Mock-Tests
colcon test --packages-select mecabridge_hardware --ctest-args -R test_mecabridge_integration
```

## API-Referenz

### MecaBridgeHardwareInterface API

#### Öffentliche Methoden

- `hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info)`
    - Initialisiert Hardware mit ROS-Parametern
    - Gibt SUCCESS bei erfolgreicher Initialisierung zurück

- `std::vector<hardware_interface::StateInterface> export_state_interfaces()`
    - Gibt State-Interfaces für alle konfigurierten Joints zurück
    - Beinhaltet Position, Geschwindigkeit und Effort für jedes Rad

- `std::vector<hardware_interface::CommandInterface> export_command_interfaces()`
    - Gibt Command-Interfaces für Geschwindigkeitssteuerung zurück
    - Ein Command-Interface pro Rad-Joint

- `hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period)`
    - Liest aktuelle Joint-Zustände von der Hardware
    - Aktualisiert Encoder-Positionen und -Geschwindigkeiten

- `hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period)`
    - Sendet Geschwindigkeitsbefehle an die Hardware
    - Konvertiert rad/s zu Motor-Befehlen

#### Private Methoden

- `bool attemptConnectionRecovery()`
    - Versucht Wiederherstellung von Kommunikationsfehlern
    - Gibt true zurück wenn Wiederherstellung erfolgreich

- `int convertVelocityToMotorCommand(double wheel_vel_rad_s)`
    - Konvertiert Rad-Geschwindigkeit zu Motor-Befehlswert
    - Skaliert und begrenzt auf gültigen Motor-Bereich

### MecaBridgeSerialProtocol API

#### Methoden

- `void setup(const std::string& serial_device, int32_t baud_rate, int32_t timeout_ms)`
    - Konfiguriert serielle Verbindungs-Parameter

- `void sendPing()`
    - Sendet Ping-Befehl für Arduino-Synchronisation

- `void setDifferentialMotors(int left_val, int right_val)`
    - Sendet Geschwindigkeitsbefehle für Differential-Antrieb

- `void setFourMotors(int fl, int fr, int rl, int rr)`
    - Sendet Geschwindigkeitsbefehle für Vier-Rad-Antrieb

- `bool connected() const`
    - Gibt aktuellen Verbindungsstatus zurück

- `bool attemptReconnection()`
    - Versucht Reconnection zum seriellen Gerät

- `void getConnectionStats(int& write_errors, int& read_errors, int& reconnection_attempts) const`
    - Ruft Fehler-Statistiken ab

### MecaBridgeDriveConfig API

#### Konfigurations-Parameter

- `DriveType drive_type`: Antriebs-Konfigurationstyp
- `bool has_encoders`: Ob Encoder vorhanden sind
- `std::string device`: Serieller Geräte-Pfad
- `int baud_rate`: Serielle Baudrate
- `int timeout`: Serieller Timeout in Millisekunden
- `float max_lin_vel`: Maximale lineare Geschwindigkeit (m/s)
- `float max_ang_vel`: Maximale angulare Geschwindigkeit (rad/s)

## Fehlerbehandlung

### Fehler-Codes

- `OK = 0`: Kein Fehler
- `CRC_FAIL = 1`: CRC-Validierung fehlgeschlagen
- `MALFORMED_FRAME = 2`: Ungültige Frame-Struktur
- `UNKNOWN_FRAME_ID = 3`: Unerkannte Frame-ID
- `VERSION_MISMATCH = 4`: Protokoll-Versions-Fehlanpassung
- `WATCHDOG_TIMEOUT = 5`: Watchdog-Timer abgelaufen

### Status-Flags

- `WATCHDOG_TRIGGERED = 0x0001`: Watchdog wurde ausgelöst
- `POSITIONAL_SERVO_LIMIT_HIT = 0x0002`: Servo-Positions-Limit erreicht
- `CRC_ERROR_SINCE_LAST = 0x0004`: CRC-Fehler seit letztem Frame aufgetreten
- `MALFORMED_ERROR_SINCE_LAST = 0x0008`: Fehlerhafter Frame seit letztem
- `ENCODER_STALE = 0x0010`: Encoder-Daten sind veraltet

## Tests und Validierung

### Unit-Testing

Das System beinhaltet umfassende Unit-Tests mit Mock-Injection:

- **Mock Serial**: `MockSerial`-Klasse für Tests ohne Hardware
- **Protokoll-Tests**: Frame-Parsing, CRC-Validierung, Fehlerbehandlung
- **Kommunikations-Tests**: Befehls-Übertragung, Encoder-Lesung
- **Hardware Interface-Tests**: ROS-Interface-Compliance, State-Verwaltung

### Integration-Testing

- **Hardware-in-the-Loop**: Tests mit echter Pico-Firmware
- **ROS Integration**: Vollständige ROS 2 Control-Stack-Tests
- **Performance-Testing**: Latenz- und Durchsatz-Validierung

## Best Practices

### Entwicklung

1. **Namespace-Konvention**: Alle Klassen verwenden `mecabridge_hardware` Namespace
2. **Fehlerbehandlung**: Immer Return-Werte prüfen und Fehler graceful behandeln
3. **Test-Modus**: `#ifdef TESTING_MODE` für Mock-Injection verwenden
4. **Dokumentation**: Diese Dokumentation bei neuen Features aktualisieren

### Deployment

1. **Serielle Berechtigungen**: Sicherstellen dass User Zugriff auf serielle Ports hat
2. **Parameter-Validierung**: Alle Konfigurations-Parameter validieren
3. **Monitoring**: Fehler-Statistiken und Verbindungsstatus überwachen
4. **Backup-Kommunikation**: Fallback-Kommunikationsmethoden vorhalten

### Sicherheit

1. **Watchdog-Monitoring**: Immer Watchdog-Status überwachen
2. **Geschwindigkeits-Limits**: Konfigurierte Geschwindigkeits-Limits einhalten
3. **Notstopp**: Notstopp-Funktionalität implementieren
4. **Fail-Safe-Verhalten**: Sicheres Verhalten für alle Fehlerfälle definieren

## Troubleshooting

### Häufige Probleme

1. **Serielle Verbindung fehlgeschlagen**
    - Geräte-Berechtigungen prüfen: `sudo usermod -a -G dialout $USER`
    - Geräte-Pfad und Baudrate verifizieren
    - USB-Verbindung und Pico-Firmware prüfen

2. **Watchdog-Timeouts**
    - Schleifenrate mit Firmware-Erwartungen verifizieren
    - Serielle Kommunikations-Latenz prüfen
    - CPU-Auslastung und Timing überwachen

3. **Encoder-Daten-Probleme**
    - Encoder-Konfiguration in Firmware verifizieren
    - Encoder-Verkabelung und Stromversorgung prüfen
    - Encoder-Counts pro Umdrehung validieren

4. **Geschwindigkeitssteuerungs-Probleme**
    - Joint-Namen zwischen Hardware und Controller abgleichen
    - Geschwindigkeits-Limits und Skalierung prüfen
    - Befehls-Übertragung überwachen

### Debug-Tools

- **Fehler-Statistiken**: `getConnectionStats()` für Diagnose verwenden
- **Logging**: ROS-Logging für detaillierte Informationen aktivieren
- **Mock-Testing**: Mock-Serial für isolierte Tests verwenden
- **Protokoll-Analyse**: Seriellen Traffic für Protokoll-Probleme überwachen

## Zukünftige Erweiterungen

- **Erweiterte Kinematik**: Unterstützung für kundenspezifische Antriebskonfigurationen
- **Sensor-Integration**: IMU, GPS und andere Sensor-Unterstützung
- **Erweiterte Sicherheit**: Mehrstufige Sicherheits-Systeme und Redundanz
- **Performance-Optimierung**: Reduzierte Latenz und verbesserter Durchsatz
- **Konfigurations-Tools**: GUI-Konfiguration und -Validierung

---

*Diese Dokumentation wird neben dem Codebase gepflegt. Bitte aktualisieren Sie sie bei wesentlichen Änderungen am
System.*
