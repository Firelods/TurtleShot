# Distance to PWM Converter

This package provides a hardware interface node that converts high-level movement commands to PWM signals for motor control.

## Overview

The `distance_to_pwm` package translates distance and angle commands into timed PWM signals for differential drive motors, enabling precise open-loop control on hardware without encoders.

## Features

- Convert distance commands (meters) to PWM duration
- Convert angle commands (degrees) to PWM rotation signals
- Configurable calibration parameters
- 50Hz control loop for smooth motor control

## Dependencies

- `rclpy` - ROS 2 Python client library
- `std_msgs` - Standard message types

## Building

```bash
colcon build --packages-select distance_to_pwm
source install/setup.bash
```

## Usage

### Run the Converter Node

```bash
ros2 run distance_to_pwm converter
```

### Command Format

The node subscribes to string commands with the following format:

- **Straight movement**: `S<distance>` where distance is in meters
  - `S0.5` - Move forward 0.5 meters
  - `S-0.3` - Move backward 0.3 meters

- **Turn in place**: `T<angle>` where angle is in degrees
  - `T90` - Turn left 90 degrees
  - `T-45` - Turn right 45 degrees

### Example Commands

```bash
# Move forward 1 meter
ros2 topic pub --once /motor_commands std_msgs/msg/String "data: 'S1.0'"

# Turn left 90 degrees
ros2 topic pub --once /motor_commands std_msgs/msg/String "data: 'T90'"

# Move backward 0.5 meters
ros2 topic pub --once /motor_commands std_msgs/msg/String "data: 'S-0.5'"
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `input_topic` | `/motor_commands` | Topic for receiving commands |
| `left_motor_topic` | `/left_motor_pwm` | Left motor PWM output topic |
| `right_motor_topic` | `/right_motor_pwm` | Right motor PWM output topic |
| `pwm_speed` | `150` | Standard PWM value for movement |
| `linear_speed` | `0.5` | Linear speed (m/s) at `pwm_speed` |
| `angular_speed` | `1.0` | Angular speed (rad/s) at `pwm_speed` |

## Topics

### Subscribed
- `/motor_commands` (`std_msgs/String`) - Movement commands

### Published
- `/left_motor_pwm` (`std_msgs/Int32`) - Left motor PWM value
- `/right_motor_pwm` (`std_msgs/Int32`) - Right motor PWM value

## Calibration

For accurate movement, calibrate the `linear_speed` and `angular_speed` parameters:

1. Set a known `pwm_speed` value
2. Command `S1.0` and measure actual distance traveled
3. Adjust `linear_speed` = actual_distance / 1.0
4. Command `T360` and measure actual rotation
5. Adjust `angular_speed` accordingly

## License

MIT License
