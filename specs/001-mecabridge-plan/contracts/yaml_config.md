# Contract: YAML Configuration Schema

Related FR: FR-006, FR-013, FR-018, FR-020

## 1. Overview
Single YAML file defines all runtime parameters for hardware interface and protocol scaling.

## 2. Top-Level Structure
```yaml
mecabridge_hardware:
  serial_port: /dev/ttyACM0
  baud_rate: 115200              # optional (USB CDC may ignore)
  state_publish_rate_hz: 50      # 10..200
  wheel_radius: 0.048            # meters > 0
  wheel_separation_x: 0.20       # meters > 0 (front-back half distance)
  wheel_separation_y: 0.18       # meters > 0 (left-right half distance)
  encoder_ticks_per_rev: 1024    # >0
  wheels:
    front_left:  { encoder_index: 0, joint_name: front_left_wheel_joint }
    front_right: { encoder_index: 1, joint_name: front_right_wheel_joint }
    rear_left:   { encoder_index: 2, joint_name: rear_left_wheel_joint }
    rear_right:  { encoder_index: 3, joint_name: rear_right_wheel_joint }
  servos:
    positional:
      joint_name: pan_servo_joint
      min_rad: 0.0
      max_rad: 3.14159
    continuous:
      joint_name: rot_servo_joint
      max_velocity_rad_s: 10.0   # optional clamp
  escs:
    left:
      joint_name: esc_left_joint
      esc_min_pwm: 1000
      esc_max_pwm: 2000
      esc_deadband: 0            # optional
    right:
      joint_name: esc_right_joint
      esc_min_pwm: 1000
      esc_max_pwm: 2000
  features:
    enable_servos: true
    enable_escs: true
    enable_diagnostics: false
```

## 3. Validation Rules
| Field | Rule | Error Message |
|-------|------|---------------|
| serial_port | non-empty | "serial_port missing" |
| state_publish_rate_hz | 10<=n<=200 | "state_publish_rate_hz out of range" |
| wheel_radius | >0 | "wheel_radius must be >0" |
| wheel_separation_x/y | >0 | "wheel separation must be >0" |
| encoder_ticks_per_rev | >0 | "encoder_ticks_per_rev must be >0" |
| wheel encoder_index | unique, 0..3 | "duplicate or invalid encoder_index" |
| wheel joint_name | unique | "duplicate joint_name" |
| servo positional min_rad<max_rad | true | "servo positional min>=max" |
| esc_min_pwm < esc_max_pwm | true | "esc pwm range invalid" |
| normalized values at runtime | clamp [-1,1] | N/A (runtime warning) |

## 4. Deterministic Ordering
Driver builds arrays in order [front_left, front_right, rear_left, rear_right] irrespective of YAML key ordering; wheel keys must exist.

## 5. Mapping to URDF
URDF `<ros2_control>` joint names must match YAML joint_name fields for controllers to bind. Mismatch → initialization failure.

## 6. Change Policy
Additive fields → MINOR bump. Renames/removals → MAJOR bump.

Schema stable for v1.
