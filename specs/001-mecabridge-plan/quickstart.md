# Quickstart: MecaBridge Hardware Interface

## 1. Overview
Bring up the MecaBridge ros2-control hardware interface with mecanum drive controller, servo + ESC forward command controllers, and watchdog safety.

## 2. Prerequisites
- ROS 2 Humble environment sourced
- Package `mecabridge_hardware` built
- Pico flashed with matching firmware (protocol version 0x01)
- USB connected (enumerates as /dev/ttyACM*)

## 3. YAML Configuration (example)
Save as `config/mecabridge.yaml`:
```yaml
mecabridge_hardware:
  serial_port: /dev/ttyACM0
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
      min_rad: 0.0
      max_rad: 3.14159
    continuous:
      joint_name: rot_servo_joint
      max_velocity_rad_s: 10.0
  escs:
    left:
      joint_name: esc_left_joint
      esc_min_pwm: 1000
      esc_max_pwm: 2000
    right:
      joint_name: esc_right_joint
      esc_min_pwm: 1000
      esc_max_pwm: 2000
  features:
    enable_servos: true
    enable_escs: true
    enable_diagnostics: false
```

## 4. URDF (ros2_control snippet)
Include in robot xacro:
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
  <joint name="front_right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="rear_left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="rear_right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="pan_servo_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
  </joint>
  <joint name="rot_servo_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="esc_left_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="esc_right_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

## 5. Controller Configuration
`config/controllers.yaml`:
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
      joints:
        - pan_servo_joint
    misc_forward_controller:
      type: forward_command_controller/ForwardCommandController
      joints:
        - rot_servo_joint
        - esc_left_joint
        - esc_right_joint

mecanum_controller:
  ros__parameters:
    front_left_wheel_command_joint_name: front_left_wheel_joint
    front_right_wheel_command_joint_name: front_right_wheel_joint
    rear_left_wheel_command_joint_name: rear_left_wheel_joint
    rear_right_wheel_command_joint_name: rear_right_wheel_joint
    kinematics:
      wheels_radius: 0.048
      sum_of_robot_center_projection_on_X_Y_axis: 0.38   # example lx+ly
    base_frame_id: base_link
    odom_frame_id: odom
```

## 6. Launch Outline
Create `launch/mecabridge_bringup.launch.py` (pseudo):
```python
# 1. Load robot_description (xacro)
# 2. Start ros2_control_node with controller params
# 3. Spawn controllers sequentially
```

## 7. Running
1. Build: `colcon build --packages-select mecabridge_hardware`
2. Source: `. install/setup.bash`
3. Launch bringup: `ros2 launch mecabridge_hardware mecabridge_bringup.launch.py`
4. Send cmd_vel: `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.1}}"`

## 8. Verification Checklist
- `ros2 control list_hardware_interfaces` shows expected command/state interfaces.
- `ros2 topic echo /joint_states` updates at ~50 Hz.
- Disconnect USB: Within 150 ms wheel velocities go to 0 (verify in joint_states).
- Latency test: custom script logs p95 < 20 ms.

## 9. Safety Notes
- Ensure ESC calibration done externally; incorrect min/max can cause motion at neutral (adjust YAML).
- Always test on stand before ground contact.

## 10. Next Steps
Implement tasks in tasks.md (to be generated) then begin TDD cycle.
