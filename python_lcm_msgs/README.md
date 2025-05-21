# LCM ROS Messages

This package contains LCM generated Python bindings for ROS message types.

## Installation

```bash
pip install -e .
```

## Usage

To use these message types in your Python code, import them with the `lcm_msgs` prefix:

```python
# Instead of
# from sensor_msgs import JointState

# Use
from lcm_msgs.sensor_msgs import JointState
from lcm_msgs.geometry_msgs import Pose, Twist
from lcm_msgs.std_msgs import Header

# Create and use the message instances
joint_state = JointState()
joint_state.name = ["joint1", "joint2"]
joint_state.position = [0.0, 1.0]
joint_state.name_length = len(joint_state.name)
joint_state.position_length = len(joint_state.position)

# Create a Twist message
twist = Twist()
twist.linear.x = 1.0
twist.linear.y = 2.0
twist.angular.z = 0.5

# Encode for LCM publishing
encoded_msg = joint_state.encode()
```

## Development

### Adding new message types

1. Generate Python bindings for your LCM message types
2. Place them in the appropriate package directory
3. Update the imports to use the `lcm_msgs` prefix when importing from other message packages
4. Run the provided fix script to correct internal imports:

```bash
./fix_imports.py
```

### Troubleshooting

If you encounter import errors like:

```
AttributeError: module 'geometry_msgs' has no attribute 'Vector3'
```

It's likely due to the internal import structure of the generated LCM files. Run the fix_imports.py script:

```bash
./fix_imports.py
```

### Updating Existing Code

To update imports in your existing code to use the lcm_msgs namespace, run:

```bash
./update_imports.py /path/to/your/code
```

### Intellisense Support

The package structure supports intellisense in most IDEs. You'll get autocompletion and type hints for all message types and their attributes.
