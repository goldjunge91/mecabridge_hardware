# mecabridge Hardware Interface Configuration Guide

This directory contains configuration files for the mecabridge hardware interface supporting multiple drive types and
configurations.

## Configuration Files Overview

### Controller Configurations

- `mecabridge_differential_controller.yaml` - Basic differential drive controller
- `mecabridge_differential_enhanced.yaml` - Enhanced differential drive with full documentation
- `mecabridge_mecanum_controller.yaml` - Basic mecanum drive controller
- `mecabridge_mecanum_enhanced.yaml` - Enhanced mecanum drive with full documentation
- `mecabridge_four_wheel_controller.yaml` - Four wheel independent drive controller

### Hardware Interface Parameters

- `mecabridge_hardware_params.yaml` - Complete hardware interface parameter documentation
- `mecabridge_example_configs.yaml` - Example configurations for all drive types

## Supported Drive Types

### 1. Differential Drive (`drive_type: "differential"`)

- **Description**: Two-wheel differential drive robot
- **Wheels**: Left and right wheels
- **Controller**: `diff_drive_controller/DiffDriveController`
- **Use Cases**: Simple mobile robots, educational platforms

### 2. Mecanum Drive (`drive_type: "mecanum"`)

- **Description**: Four-wheel omnidirectional drive with mecanum wheels
- **Wheels**: Front-left, front-right, rear-left, rear-right
- **Controller**: `mecanum_drive_controller/MecanumDriveController`
- **Use Cases**: Omnidirectional robots, warehouse automation

### 3. Four Wheel Independent (`drive_type: "four_wheel"`)

- **Description**: Four wheels with independent control (no specific kinematics)
- **Wheels**: Front-left, front-right, rear-left, rear-right
- **Controller**: `forward_command_controller/ForwardCommandController`
- **Use Cases**: Custom drive algorithms, research platforms

## Key Parameters

### Drive Configuration

```yaml
drive_type: "differential"  # "differential", "mecanum", "four_wheel"
has_encoders: false         # true if Arduino provides encoder feedback
```

### Serial Communication

```yaml
device: ""           # Serial port path, empty for auto-detection
baud_rate: 115200    # Communication speed
timeout: 50          # Timeout in milliseconds
loop_rate: 20.0      # Control frequency in Hz
```

### Velocity Limits

```yaml
max_lin_vel: 0.3     # Maximum linear velocity (m/s)
max_ang_vel: 1.0     # Maximum angular velocity (rad/s)
```

### Wheel Joint Names

Must match the joint names defined in your robot's URDF:

**Differential Drive:**

```yaml
left_wheel_name: "left_wheel"
right_wheel_name: "right_wheel"
```

**Mecanum/Four Wheel:**

```yaml
front_left_wheel_name: "front_left_wheel"
front_right_wheel_name: "front_right_wheel"
rear_left_wheel_name: "rear_left_wheel"
rear_right_wheel_name: "rear_right_wheel"
```

### Encoder Parameters (if `has_encoders: true`)

```yaml
enc_counts_per_rev: 1920  # Encoder counts per wheel revolution
wheel_radius: 0.05        # Wheel radius in meters
```

### Mecanum-Specific Parameters

```yaml
wheel_base: 0.3      # Distance between front and rear axles (m)
track_width: 0.3     # Distance between left and right wheels (m)
```

## Usage Examples

### Launch Differential Drive

```bash
# Basic differential drive
ros2 launch mecabridge_hardware mecabridge_differential.launch.py

# With specific serial device
ros2 launch mecabridge_hardware mecabridge_differential_enhanced.launch.py device:=/dev/ttyUSB0

# With encoder support
ros2 launch mecabridge_hardware mecabridge_differential_enhanced.launch.py has_encoders:=true
```

### Launch Mecanum Drive

```bash
# Basic mecanum drive
ros2 launch mecabridge_hardware mecabridge_mecanum.launch.py

# With mock hardware for testing
ros2 launch mecabridge_hardware mecabridge_mecanum.launch.py use_mock_hardware:=true
```

### Launch Four Wheel Independent

```bash
# Four wheel independent control
ros2 launch mecabridge_hardware mecabridge_four_wheel.launch.py

# With custom baud rate
ros2 launch mecabridge_hardware mecabridge_four_wheel.launch.py baud_rate:=57600
```

## Serial Port Auto-Detection

When `device` is empty or not specified, the hardware interface will automatically try these ports in order:

1. `/dev/ttyUSB0`
2. `/dev/ttyUSB1`
3. `/dev/ttyACM0`

## Communication Protocol

### Motor Commands

- **Differential**: `"V {left_value} {right_value}\n"`
- **Four Motor**: `"M {fl} {fr} {rl} {rr}\n"`
- **Values**: Integer range -100 to 100

### Encoder Reading (if enabled)

- **Command**: `"E\n"`
- **Differential Response**: `"{left_count} {right_count}\n"`
- **Four Motor Response**: `"{fl_count} {fr_count} {rl_count} {rr_count}\n"`

### Initialization

- Send `"PING\n"` on connection for Arduino synchronization
- Wait 2 seconds after connection before sending commands

## Troubleshooting

### Common Issues

1. **Serial Connection Failed**
    - Check device permissions: `sudo chmod 666 /dev/ttyUSB0`
    - Verify Arduino is connected and powered
    - Try different baud rates

2. **No Movement**
    - Check velocity limits in controller configuration
    - Verify joint names match URDF
    - Check Arduino firmware compatibility

3. **Erratic Movement**
    - Adjust `mix_factor` for differential drive
    - Check wheel geometry parameters for mecanum
    - Verify encoder parameters if using encoders

### Debug Commands

```bash
# List available controllers
ros2 control list_controllers

# Check hardware interfaces
ros2 control list_hardware_interfaces

# Monitor command topics
ros2 topic echo /diff_cont/cmd_vel_unstamped
ros2 topic echo /mecanum_cont/cmd_vel

# Check joint states
ros2 topic echo /joint_states
```

## Custom Configuration

To create a custom configuration:

1. Copy one of the example configurations
2. Modify parameters for your robot
3. Update joint names to match your URDF
4. Adjust velocity limits and geometry
5. Test with mock hardware first: `use_mock_hardware:=true`

## Integration with Existing Systems

The mecabridge hardware interface is designed to be compatible with:

- Standard ros2_control controllers
- Existing launch files and configurations
- Navigation stack (nav2)
- Teleoperation packages
- Custom control algorithms