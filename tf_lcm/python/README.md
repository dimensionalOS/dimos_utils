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

You can also use the built-in LCM wrapper and TransformListener class. This approach simplifies the code but requires careful resource management to avoid segmentation faults and socket errors. Here's the recommended pattern:

```python
import tf_lcm_py
import threading
import datetime
import lcm_msgs
import signal
import sys
from contextlib import contextmanager

# Context manager for safely managing LCM instance lifecycle
@contextmanager
def safe_lcm_instance():
    """Context manager for safely managing LCM instance lifecycle"""
    # Try multiple provider URLs to find one that works
    provider_urls = [
        None,  # Default
        "udpm://localhost:7667",
        "file:///tmp/lcm-log.data",
        "memq://"
    ]
    
    lcm_instance = None
    for provider in provider_urls:
        try:
            if provider is None:
                lcm_instance = tf_lcm_py.LCM()
            else:
                lcm_instance = tf_lcm_py.LCM(provider)
            
            if lcm_instance.good():
                print(f"Successfully initialized LCM with provider: {provider if provider else 'default'}")
                break
            else:
                print(f"LCM initialized but not in good state with provider: {provider if provider else 'default'}")
                lcm_instance = None
        except Exception as e:
            print(f"Failed to initialize LCM with provider {provider}: {e}")
    
    if lcm_instance is None:
        print("WARNING: Could not initialize any LCM provider, using fallback")
        lcm_instance = tf_lcm_py.LCM("null://")
    
    try:
        yield lcm_instance
    finally:
        # Keeping a reference until the end of the context ensures proper cleanup
        pass

# Track resources for proper cleanup
resources_to_cleanup = []

# Cleanup function to explicitly release resources
def cleanup_resources():
    # Clean up resources in reverse order (last created first)
    for resource in reversed(resources_to_cleanup):
        try:
            # For objects like TransformListener that might have close/shutdown methods
            if hasattr(resource, 'close'):
                resource.close()
            elif hasattr(resource, 'shutdown'):
                resource.shutdown()
            
            # Explicitly delete the resource
            del resource
        except Exception as e:
            print(f"Error during cleanup: {e}")
    
    # Clear the resources list
    resources_to_cleanup.clear()

# Signal handler for graceful termination
def signal_handler(sig, frame):
    print("\nInterrupt received, cleaning up...")
    cleanup_resources()
    sys.exit(1)

# Register signal handlers
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# Create resources in a specific order
buffer = tf_lcm_py.Buffer(10.0)  # 10 seconds cache time
resources_to_cleanup.append(buffer)

# Use context manager for LCM instance
with safe_lcm_instance() as lcm_instance:
    resources_to_cleanup.append(lcm_instance)
    
    # Create a listener attached to the buffer
    listener = tf_lcm_py.TransformListener(lcm_instance, buffer)
    resources_to_cleanup.append(listener)
    
    # For continuous operation, set up a background thread
    def lcm_handler_thread(lcm):
        try:
            while True:
                try:
                    # Handle timeouts and check if LCM is still healthy
                    if not lcm.handle_timeout(100):  # 100ms timeout
                        if not lcm.good():
                            print("WARNING: LCM instance is no longer in a good state")
                except Exception as e:
                    print(f"Error in LCM handler thread: {e}")
        except KeyboardInterrupt:
            # Thread will exit when the main thread is terminated
            pass

    # Start handler thread
    handler_thread = threading.Thread(target=lcm_handler_thread, args=(lcm_instance,))
    handler_thread.daemon = True  # Thread will exit when main thread exits
    handler_thread.start()
    
    # Now you can look up transforms safely
    try:
        # Main application loop
        while True:
            try:
                now = datetime.datetime.now()
                
                # Check if transform is available
                if buffer.can_transform("world", "robot", now):
                    # Look up the transform
                    transform = buffer.lookup_transform("world", "robot", now, lcm_module=lcm_msgs)
                    
                    # Process the transform data
                    print(f"Position: ({transform.transform.translation.x:.6f}, "
                          f"{transform.transform.translation.y:.6f}, "
                          f"{transform.transform.translation.z:.6f})")
                
                # Add application-specific logic here
                
            except tf_lcm_py.TransformException as e:
                print(f"Transform lookup failed: {e}")
            
            # Sleep to avoid consuming too much CPU
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nUser interrupted, shutting down...")
    finally:
        # Always clean up resources when exiting
        print("Cleaning up resources...")
        cleanup_resources()
```

### Important Resource Management Considerations

When using the built-in LCM wrapper, it's critical to follow these best practices to avoid segmentation faults and socket errors:

1. **Proper Resource Initialization**: Create resources in the right order (Buffer -> LCM -> TransformListener) and track them for cleanup.

2. **Context Managers**: Use context managers for LCM instances to handle initialization failures gracefully.

3. **Explicit Resource Cleanup**: Always clean up resources in reverse order of creation when your application exits.

4. **Signal Handling**: Register signal handlers to ensure proper cleanup on unexpected termination.

5. **Error Handling**: Wrap LCM operations in try/except blocks to handle network errors gracefully.

6. **Resource Tracking**: Keep references to all resources to prevent premature garbage collection.

Following these patterns will help you avoid common issues like segmentation faults at program exit and socket initialization errors.

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
