# Joy Controller Interface for LCM

This document describes the `joy.py` utility, which is a replacement for the ROS Joy node using LCM messaging instead. It reads joystick/gamepad inputs and publishes them as LCM Joy messages.

## Overview

The `joy.py` script converts physical joystick inputs into standardized LCM Joy messages, which can then be used by other components like `teleop_twist_joy.py` for robot control. It provides:

- Automatic detection of connected joysticks/gamepads
- Joystick calibration for different controller types
- Configuration saving/loading
- Regular publishing of joystick states via LCM

## Installation Requirements

- Python 3.x
- pygame (for joystick access)
- LCM (Lightweight Communications and Marshalling)
- python_lcm_msgs (for LCM message types)

## Basic Usage

Run the script from the terminal:

```bash
python3 utils/joy.py
```

The script will:
1. Detect available joysticks
2. Auto-select the joystick if only one is available or prompt you to choose one
3. Load an existing configuration or create a default one
4. Begin publishing joystick data to the `joy#sensor_msgs.Joy` LCM channel

## Command-line Options

```
--calibrate    Run the joystick calibration wizard
--config PATH  Specify a custom configuration file path
--rate RATE    Set the publishing rate in Hz (default: 50)
--list         List available joysticks and exit
```

## Calibration

For the best experience, you should calibrate your joystick the first time you use it:

```bash
python3 utils/joy.py --calibrate
```

This interactive calibration:
- Maps physical joystick axes and buttons to the standardized format
- Configures inversion settings for axes
- Sets deadzone values
- Saves the configuration for future use

## Joystick Mapping

The script standardizes different controller types to provide a consistent interface with:

- 8 axes: Left stick (X/Y), Right stick (X/Y), Triggers (L/R), D-pad (X/Y)
- 11 buttons: A, B, X, Y, LB, RB, Back, Start, Power, Left stick button, Right stick button

## Integration with Other Components

This script is designed to work with other LCM components like:

- `teleop_twist_joy.py`: Converts joystick inputs to velocity commands
- `joy_listener.py`: Monitors and displays joystick inputs

## Configuration File

The configuration is stored in `~/.joy_lcm_config.json` by default and contains:
- Axis mappings
- Button mappings 
- Axis inversion settings
- Deadzone values

## Troubleshooting

If you encounter issues:

1. **No joysticks detected**: Ensure your controller is properly connected
2. **Incorrect behavior**: Run the calibration again using `--calibrate`
3. **Message format issues**: Make sure you're using compatible `python_lcm_msgs` versions

For detailed diagnostics, you can use the `joy_listener.py` script to view the raw joystick messages.
