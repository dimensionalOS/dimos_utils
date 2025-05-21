# Joystick Controller Mappings for Teleop Twist Joy

This document provides mappings between controller input indices and physical controller elements, along with guidance on configuring the `teleop_twist_joy.py` script.

## Controller Button Mappings

The following table provides the mapping between button indices in the Joy message and physical buttons on the controller:

| Index | Button Name on Controller |
| ----- | ------------------------- |
| 0     | A                         |
| 1     | B                         |
| 2     | X                         |
| 3     | Y                         |
| 4     | LB                        |
| 5     | RB                        |
| 6     | Back                      |
| 7     | Start                     |
| 8     | Power                     |
| 9     | Button Stick Left         |
| 10    | Button Stick Right        |

## Controller Axis Mappings

The following table provides the mapping between axis indices in the Joy message and physical axes on the controller:

| Index | Axis Name on Controller     |
| ----- | --------------------------- |
| 0     | Left/Right Axis Stick Left  |
| 1     | Up/Down Axis Stick Left     |
| 2     | Left/Right Axis Stick Right |
| 3     | Up/Down Axis Stick Right    |
| 4     | RT                          |
| 5     | LT                          |
| 6     | Cross Key Left/Right        |
| 7     | Cross Key Up/Down           |

## Required Setup

**IMPORTANT**: Before using `teleop_twist_joy.py`, you must first run the `joy.py` script to publish joystick data to LCM:

```bash
# Start the joy node in one terminal:
python3 utils/joy.py

# Then run teleop_twist_joy in another terminal:
python3 utils/teleop_twist_joy.py
```

The `joy.py` script reads data from your physical controller and publishes standardized LCM Joy messages, which are then processed by `teleop_twist_joy.py`. See [README_joy.md](../README_joy.md) for more details on joystick configuration and calibration.

## Using Teleop Twist Joy

The `teleop_twist_joy.py` script converts joystick input into twist messages for robot control. Here are some common configurations:

### Default Configuration

By default, the following axes are used:

- Linear X: Axis 0 (Left/Right Stick Left)
- Linear Y: Axis 1 (Up/Down Stick Left)
- Linear Z: Axis 7 (Cross Key Up/Down)
- Angular Yaw: Axis 6 (Cross Key Left/Right)
- Angular Pitch: Axis 3 (Up/Down Stick Right)
- Angular Roll: Axis 2 (Left/Right Stick Right)

### Button Mode

Instead of using a joystick axis, you can use a pair of buttons to simulate an axis using the `b4,5` format, where:

- The first number (4) is the button index for negative movement
- The second number (5) is the button index for positive movement

Example: `--axis-linear-x b4,5` would use LB (4) for negative X movement and RB (5) for positive X movement.

### Command-Line Examples

Basic usage:

```bash
python3 utils/teleop_twist_joy.py
```

Custom axis mappings:

```bash
python3 utils/teleop_twist_joy.py --axis-linear-x 1 --axis-angular-yaw 0
```

Using buttons for an axis:

```bash
python3 utils/teleop_twist_joy.py --axis-linear-x b4,5
```

Verbose mode (for debugging):

```bash
python3 utils/teleop_twist_joy.py --verbose
```

Custom scaling:

```bash
python3 utils/teleop_twist_joy.py --scale-linear-x 0.8 --scale-angular-yaw 0.3
```

### Enabling and Turbo Modes

By default:

- Button 0 (A) must be held to enable movement
- Turbo mode is enabled and button 1 (B) is used to toggle turbo mode

To change these settings:

```bash
python3 utils/teleop_twist_joy.py --enable-button 1 --enable-turbo-button 5
```

To disable the requirement for the enable button:

```bash
python3 utils/teleop_twist_joy.py --require-enable-button false
```

## Troubleshooting

If you're experiencing issues with teleop control:

1. **No response from controller**: Make sure the `joy.py` script is running in a separate terminal
2. **Incorrect axis mappings**: Use the `--verbose` flag to see the actual axis values being received
3. **Controller not detected**: Try running `python3 utils/joy.py --list` to see available controllers
4. **Calibration issues**: Run `python3 utils/joy.py --calibrate` to set up your controller properly

For best results, always run the `joy.py` script with the `--calibrate` option the first time you use a new controller.
