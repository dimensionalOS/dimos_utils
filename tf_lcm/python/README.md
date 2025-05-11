# tf_lcm Python Bindings

This directory contains Python bindings for the tf_lcm library, allowing you to use the LCM-based transform library in Python applications.

## Overview

The `tf_lcm` package provides a LCM-based alternative to ROS TF2, focusing on efficient transform handling with LCM as the transport layer. These Python bindings expose the core functionality of tf_lcm to Python, including:

- Transform broadcasting
- Transform listening
- Buffer management
- Transform lookups (single and advanced)
- Frame management

## Installation

The Python bindings for tf_lcm can be installed using the provided `build.py` script, which handles the CMake build process and proper Python packaging.

### Dependencies

- Python 3.6+
- lcm (Python bindings)
- lcm_msgs (Python generated message bindings)
- CMake 3.10+
- C++ compiler (g++, clang++)

### Installation Steps

1. Navigate to the `tf_lcm/python` directory:

```bash
cd tf_lcm/python
```

2. Run the build script:

```bash
python build.py
```

This will:
- Build the C++ extension module using CMake
- Find and copy the module to the correct location
- Install the package with pip

### Additional Installation Options

The build script supports several options:

```bash
# Install in development mode (editable installation)
python build.py develop

# Build a wheel without installing
python build.py wheel

# Keep the build directory (for debugging)
python build.py --keep-build
```

### Verifying Installation

After installation, you can verify that the package was installed correctly by importing it from any directory:

```bash
python -c "import tf_lcm_py; print(f'Successfully imported tf_lcm_py version: {tf_lcm_py.__version__}')"
```

## Usage

### Basic Setup

To use tf_lcm from Python, you need to:

1. Import the module
2. Set up LCM communication
3. Create a transform buffer and listener
4. Use the transform API (lookups, broadcasting, etc.)

#### Option 1: Using Native Python LCM (Recommended)

Using the native Python LCM library for message handling gives you more control and stability:

```python
import tf_lcm_py
import lcm
import datetime

# Import LCM message types
import lcm_msgs
from lcm_msgs.tf2_msgs.TFMessage import TFMessage

# Create a buffer for transforms
buffer = tf_lcm_py.Buffer(30.0)  # 30 seconds cache time

# Create LCM instance using native Python LCM
lc = lcm.LCM()

# Define transform message handlers
def tf_callback(channel, data):
    msg = TFMessage.decode(data)
    for i in range(msg.transforms_length):
        transform = msg.transforms[i]
        buffer.set_transform(transform, "lcm_publisher", False)

def tf_static_callback(channel, data):
    msg = TFMessage.decode(data)
    for i in range(msg.transforms_length):
        transform = msg.transforms[i]
        buffer.set_transform(transform, "static_publisher", True)

# Subscribe to transform channels (IMPORTANT: use the correct channel names)
lc.subscribe("tf#tf2_msgs.TFMessage", tf_callback)
lc.subscribe("tf_static#tf2_msgs.TFMessage", tf_static_callback)

# Main loop
while True:
    # Handle LCM messages
    lc.handle_timeout(100)  # 100ms timeout
    
    # Look up transforms as needed
    if buffer.can_transform("world", "robot", datetime.datetime.now()):
        transform = buffer.lookup_transform("world", "robot", datetime.datetime.now())
        print(f"Position: {transform.transform.translation.x}, {transform.transform.translation.y}, {transform.transform.translation.z}")
```

#### Option 2: Using the Built-in LCM Wrapper

You can also use the built-in LCM wrapper and TransformListener class:

```python
import tf_lcm_py
import threading
import datetime
import lcm_msgs

# Create an LCM instance
lcm_instance = tf_lcm_py.LCM()

# Set up a background thread for message handling
def lcm_handler_thread(lcm):
    while True:
        lcm.handle_timeout(100)  # 100ms timeout

handler_thread = threading.Thread(target=lcm_handler_thread, args=(lcm_instance,))
handler_thread.daemon = True
handler_thread.start()

# Create a buffer
buffer = tf_lcm_py.Buffer(10.0)  # 10 seconds cache time

# Create a listener attached to the buffer
listener = tf_lcm_py.TransformListener(lcm_instance, buffer)

# Now you can look up transforms
try:
    transform = buffer.lookup_transform("world", "robot", datetime.datetime.now())
    print(f"Position: {transform.transform.translation.x}, {transform.transform.translation.y}, {transform.transform.translation.z}")
except tf_lcm_py.TransformException as e:
    print(f"Transform lookup failed: {e}")
```

### Broadcasting Transforms

To broadcast transforms:

```python
import time
import tf_lcm_py
from lcm_msgs.geometry_msgs import TransformStamped, Transform, Vector3, Quaternion
from lcm_msgs.std_msgs import Header
from lcm_msgs.tf2_msgs.TFMessage import TFMessage

# Create a broadcaster
broadcaster = tf_lcm_py.TransformBroadcaster()

# Create a transform message
transform = TransformStamped()
transform.header = Header()
transform.header.frame_id = "world"
transform.header.stamp.sec = int(time.time())
transform.header.stamp.nsec = int((time.time() % 1) * 1e9)
transform.child_frame_id = "robot"

transform.transform = Transform()
transform.transform.translation = Vector3()
transform.transform.translation.x = 1.0
transform.transform.translation.y = 2.0
transform.transform.translation.z = 3.0

transform.transform.rotation = Quaternion()
transform.transform.rotation.w = 1.0  # Identity rotation

# Send the transform
broadcaster.send_transform(transform)
```

### Static Transform Broadcasting

To broadcast static transforms (which never change and are cached permanently):

```python
# Create a static broadcaster
static_broadcaster = tf_lcm_py.StaticTransformBroadcaster()

# Create a transform message (same as above)
transform = TransformStamped()
# ... set up the transform fields ...

# Send the static transform (will be broadcast once with higher reliability)
static_broadcaster.send_transform(transform)
```

## Channel Names (IMPORTANT)

A critical detail for interoperability with C++ code is the channel names used for transform messages. The Python code **must** use the same channel names as the C++ implementation:

- Regular transforms: `tf#tf2_msgs.TFMessage`
- Static transforms: `tf_static#tf2_msgs.TFMessage`

Using just `tf` or `tf_static` without the message type suffix will not work.

```python
# CORRECT channel subscription
lc.subscribe("tf#tf2_msgs.TFMessage", tf_callback)
lc.subscribe("tf_static#tf2_msgs.TFMessage", tf_static_callback)

# INCORRECT channel subscription - will miss messages
lc.subscribe("tf", tf_callback)
lc.subscribe("tf_static", tf_static_callback)
```

## Examples

See the `examples` directory for complete working examples:

- `tf_broadcaster.py`: Demonstrates publishing transforms
- `tf_listener.py`: Demonstrates listening for transforms
- `final_robot_link_test.py`: Demonstrates a complete robot link transform lookup test
- `lcm_monitor.py`: A utility for monitoring LCM transform messages

## Type Conversion

The bindings automatically handle conversion between Python LCM message types and C++ types. To ensure smooth conversion:

1. For functions returning transform data back to Python, make sure to handle the types correctly.
2. Use proper datetime objects with a `timestamp()` method for time handling.

## Troubleshooting

If you're experiencing issues with the Python bindings:

1. **No transforms received**: Check that you're using the correct channel names (`tf#tf2_msgs.TFMessage` and `tf_static#tf2_msgs.TFMessage`).

2. **Type errors**: Make sure your Python message types are correctly imported from the lcm_msgs package.

3. **Socket errors**: If you see `setsockopt` errors, it might indicate issues with LCM multicast configuration.

4. **Mutex errors**: These often arise from threading issues. Try simplifying your code to use the native Python LCM library as shown in Option 1 above.

## Thread Safety

The tf_lcm library is thread-safe, but you must ensure that the LCM message handling thread is running for the system to work. The native Python LCM approach (Option 1) can sometimes be more stable than using multiple threads with the wrapped LCM instance.
