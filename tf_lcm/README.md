# TF_LCM - Transform Library for LCM

This library implements transform (TF) functionality similar to ROS2's TF2 package, but using LCM (Lightweight Communications and Marshalling) for communication instead of ROS middleware. It provides both C++ and Python interfaces through pybind11 bindings.

## Latest Updates

- **Enhanced Graph Traversal**: Improved multi-hop transform lookups using breadth-first search for more robust transform chains
- **Time Travel Functionality**: Added capability to retrieve transforms at specific timestamps and between different time points
- **Quaternion Handling**: Enhanced quaternion normalization and inversion for more accurate rotation representations
- **Performance Benchmarks**: Added benchmark tools to evaluate transform lookup and time travel performance

## Features

- **Transform Storage and Lookup**: Maintain a transform tree and look up transforms between arbitrary frames
- **Transform Broadcasting**: Broadcast transforms to other nodes
- **Transform Listening**: Listen for transforms from other nodes
- **Static Transforms**: Support for static (fixed) transforms
- **Timeout Support**: Wait for transforms to become available
- **Time Travel**: Support for temporal transform lookups

## Prerequisites

- C++14 compatible compiler
- CMake 3.10 or higher
- LCM (Lightweight Communications and Marshalling)
- Python 3 (for Python bindings)

## Building

```bash
# Create build directory
mkdir -p build && cd build

# Configure with CMake
cmake ..

# Build
make

# Install (optional)
make install
```

## Test and Benchmark Tools

The library includes several test and benchmark tools to validate functionality and measure performance:

### Core Tests

- **test_lookup**: A simplified test for transform lookups that verifies the core functionality without dependencies on LCM
  ```
  ./build/test/test_lookup
  ```

- **chained_test**: Verifies multi-hop transform functionality with a chain of transforms
  ```
  ./build/test/chained_test
  ```

- **time_travel_demo**: Demonstrates the time travel functionality with transforms at different timestamps
  ```
  ./build/test/time_travel_demo
  ```

- **robot_link_lookup_test**: Tests lookup between specific frames on a robot ("world" to "link6")
  ```
  ./build/test/robot_link_lookup_test
  ```

### Performance Benchmarks

- **transform_benchmark**: Benchmarks the performance of transforming a configurable number of points between frames
  ```
  # Transform 1,000,000 points (default)
  ./build/test/transform_benchmark
  
  # Transform a custom number of points
  ./build/test/transform_benchmark 500000
  ```

- **time_travel_benchmark**: Benchmarks the performance of transform lookups at different timestamps
  ```
  # Run with 10,000 lookups per test case (default)
  ./build/test/time_travel_benchmark
  
  # Run with a custom number of lookups and timeout
  ./build/test/time_travel_benchmark 5000 30.0
  ```

## Usage Examples

### C++ Examples

#### 1. Broadcasting a Transform

```cpp
#include <lcm/lcm-cpp.hpp>
#include "tf_lcm/broadcaster.hpp"

int main() {
    // Create a broadcaster
    tf_lcm::TransformBroadcaster broadcaster;
    
    // Create a transform
    geometry_msgs::TransformStamped transform;
    
    // Fill in header
    transform.header.frame_id = "world";
    transform.header.stamp.sec = time_seconds;  // Current time in seconds
    transform.header.stamp.nsec = time_nanoseconds;  // Current time nanoseconds part
    
    // Set child frame
    transform.child_frame_id = "robot_base";
    
    // Set translation
    transform.transform.translation.x = 1.0;
    transform.transform.translation.y = 2.0;
    transform.transform.translation.z = 0.5;
    
    // Set rotation (quaternion)
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    
    // Broadcast the transform
    broadcaster.sendTransform(transform);
    
    return 0;
}
```

#### 2. Broadcasting a Static Transform

```cpp
#include <lcm/lcm-cpp.hpp>
#include "tf_lcm/broadcaster.hpp"

int main() {
    // Create a static broadcaster
    tf_lcm::StaticTransformBroadcaster static_broadcaster;
    
    // Create a transform (similar to regular transform)
    geometry_msgs::TransformStamped transform;
    
    // Fill in transform data...
    
    // Broadcast the static transform
    static_broadcaster.sendTransform(transform);
    
    return 0;
}
```

#### 3. Listening for Transforms

```cpp
#include <lcm/lcm-cpp.hpp>
#include "tf_lcm/listener.hpp"

int main() {
    // Create a buffer with 10 seconds cache time
    tf_lcm::Buffer buffer(10.0);
    
    // Create a listener with the buffer
    tf_lcm::TransformListener listener(buffer);
    
    // Get current time
    auto now = std::chrono::system_clock::now();
    
    try {
        // Look up a transform
        geometry_msgs::TransformStamped transform = 
            buffer.lookupTransform("world", "robot_base", now);
        
        // Use the transform...
        std::cout << "Position: (" 
                  << transform.transform.translation.x << ", "
                  << transform.transform.translation.y << ", "
                  << transform.transform.translation.z << ")" << std::endl;
    }
    catch (const tf_lcm::TransformException& ex) {
        std::cerr << "Exception: " << ex.what() << std::endl;
    }
    
    return 0;
}
```

#### 4. Waiting for a Transform with Timeout

```cpp
#include <lcm/lcm-cpp.hpp>
#include "tf_lcm/listener.hpp"

int main() {
    tf_lcm::Buffer buffer(10.0);
    tf_lcm::TransformListener listener(buffer);
    
    auto now = std::chrono::system_clock::now();
    
    try {
        // Wait up to 5 seconds for the transform to become available
        geometry_msgs::TransformStamped transform = 
            buffer.lookupTransform("map", "robot_base", now, 
                                  std::chrono::duration<double>(5.0));
        
        // Use the transform...
    }
    catch (const tf_lcm::TimeoutException& ex) {
        std::cerr << "Timeout waiting for transform: " << ex.what() << std::endl;
    }
    catch (const tf_lcm::TransformException& ex) {
        std::cerr << "Exception: " << ex.what() << std::endl;
    }
    
    return 0;
}
```

#### 5. Using Time Travel

```cpp
#include <lcm/lcm-cpp.hpp>
#include "tf_lcm/listener.hpp"

int main() {
    // Create a buffer with 10-second cache time
    tf_lcm::Buffer buffer(10.0);
    tf_lcm::TransformListener listener(buffer);
    
    // Define different time points
    auto now = std::chrono::system_clock::now();
    auto past = now - std::chrono::seconds(2);  // 2 seconds in the past
    auto future = now + std::chrono::seconds(2);  // 2 seconds in the future
    
    try {
        // 1. Basic time travel - lookup transform at a specific time
        geometry_msgs::TransformStamped past_transform = 
            buffer.lookupTransform("world", "robot_base", past);
        
        std::cout << "Robot position 2 seconds ago: (" 
                  << past_transform.transform.translation.x << ", "
                  << past_transform.transform.translation.y << ", "
                  << past_transform.transform.translation.z << ")" << std::endl;
        
        // 2. Advanced time travel - transform between different times
        // This transforms a point from one frame at one time to another frame at a different time
        geometry_msgs::TransformStamped transform_across_time = 
            buffer.lookupTransform(
                "map",          // Target frame
                future,         // Target time
                "robot_base",   // Source frame
                past,           // Source time
                "world"         // Fixed frame (common reference)
            );
        
        // This gives the transform from robot_base at past time to map at future time,
        // using world as the common reference frame
        
        // 3. Transforming points using time-based transforms
        // Example point in robot_base frame at past time
        struct Point3D {
            double x, y, z;
        };
        
        Point3D point_in_robot_base = {1.0, 0.5, 0.0};
        
        // Extract transform components
        double tx = transform_across_time.transform.translation.x;
        double ty = transform_across_time.transform.translation.y;
        double tz = transform_across_time.transform.translation.z;
        
        // Extract quaternion
        double qw = transform_across_time.transform.rotation.w;
        double qx = transform_across_time.transform.rotation.x;
        double qy = transform_across_time.transform.rotation.y;
        double qz = transform_across_time.transform.rotation.z;
        
        // Apply quaternion rotation to the point
        // (simplified calculation for demonstration)
        Point3D rotated_point;
        double px = point_in_robot_base.x;
        double py = point_in_robot_base.y;
        double pz = point_in_robot_base.z;
        
        // Quaternion rotation calculation (q * p * q^-1)
        // Simplified implementation for demonstration
        rotated_point.x = (1 - 2*qy*qy - 2*qz*qz) * px + (2*qx*qy - 2*qz*qw) * py + (2*qx*qz + 2*qy*qw) * pz;
        rotated_point.y = (2*qx*qy + 2*qz*qw) * px + (1 - 2*qx*qx - 2*qz*qz) * py + (2*qy*qz - 2*qx*qw) * pz;
        rotated_point.z = (2*qx*qz - 2*qy*qw) * px + (2*qy*qz + 2*qx*qw) * py + (1 - 2*qx*qx - 2*qy*qy) * pz;
        
        // Apply translation
        Point3D transformed_point = {
            rotated_point.x + tx,
            rotated_point.y + ty,
            rotated_point.z + tz
        };
        
        std::cout << "Point transformed across time: (" 
                  << transformed_point.x << ", "
                  << transformed_point.y << ", "
                  << transformed_point.z << ")" << std::endl;
    }
    catch (const tf_lcm::TransformException& ex) {
        std::cerr << "Exception: " << ex.what() << std::endl;
    }
    
    return 0;
}
```

#### 6. Transforming Multiple Points Efficiently

```cpp
#include <lcm/lcm-cpp.hpp>
#include "tf_lcm/listener.hpp"
#include <vector>
#include <chrono>

// Structure to represent a 3D point
struct Point3D {
    double x, y, z;
};

// Transform a point using a transform
Point3D transformPoint(const geometry_msgs::TransformStamped& transform, const Point3D& point) {
    // Extract rotation components
    double qw = transform.transform.rotation.w;
    double qx = transform.transform.rotation.x;
    double qy = transform.transform.rotation.y;
    double qz = transform.transform.rotation.z;
    
    // Extract translation components
    double tx = transform.transform.translation.x;
    double ty = transform.transform.translation.y;
    double tz = transform.transform.translation.z;
    
    // Apply rotation using quaternion math
    double px = point.x;
    double py = point.y;
    double pz = point.z;
    
    // Apply the rotation (simplified for readability)
    Point3D rotated;
    rotated.x = (1 - 2*qy*qy - 2*qz*qz) * px + (2*qx*qy - 2*qz*qw) * py + (2*qx*qz + 2*qy*qw) * pz;
    rotated.y = (2*qx*qy + 2*qz*qw) * px + (1 - 2*qx*qx - 2*qz*qz) * py + (2*qy*qz - 2*qx*qw) * pz;
    rotated.z = (2*qx*qz - 2*qy*qw) * px + (2*qy*qz + 2*qx*qw) * py + (1 - 2*qx*qx - 2*qy*qy) * pz;
    
    // Apply translation
    return {
        rotated.x + tx,
        rotated.y + ty,
        rotated.z + tz
    };
}

int main() {
    // Create buffer and listener
    tf_lcm::Buffer buffer(10.0);
    tf_lcm::TransformListener listener(buffer);
    
    // Generate some test points
    std::vector<Point3D> points;
    for (int i = 0; i < 1000; ++i) {
        points.push_back({static_cast<double>(i) * 0.01, 
                        static_cast<double>(i) * 0.02, 
                        static_cast<double>(i) * 0.03});
    }
    
    auto now = std::chrono::system_clock::now();
    
    try {
        // Get transform once and apply to all points
        auto transform = buffer.lookupTransform("world", "link6", now);
        
        // Start timing
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Transform all points
        std::vector<Point3D> transformed_points;
        transformed_points.reserve(points.size());
        
        for (const auto& point : points) {
            transformed_points.push_back(transformPoint(transform, point));
        }
        
        // End timing
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration<double>(end_time - start_time).count();
        
        std::cout << "Transformed " << points.size() << " points in " 
                  << duration << " seconds" << std::endl;
        std::cout << "Rate: " << (points.size() / duration) << " points/second" << std::endl;
    }
    catch (const tf_lcm::TransformException& ex) {
        std::cerr << "Exception: " << ex.what() << std::endl;
    }
    
    return 0;
}
```

### Python Examples

#### 1. Using the Library with Python LCM Messages

The TF library integrates with the `python_lcm_msgs` package, which provides Python bindings for LCM-based message types.

```python
# First, ensure you have the python_lcm_msgs package in your PYTHONPATH
import python_lcm_msgs
from python_lcm_msgs import geometry_msgs
from tf_lcm import Buffer, TransformListener
import lcm
import time

# Create an LCM instance
lc = lcm.LCM()

# Create a buffer with 10-second cache time
buffer = Buffer(10.0)

# Create a listener with our existing buffer and LCM instance
listener = TransformListener(lc, buffer)

# Start a background thread to handle LCM messages
import threading
def lcm_thread():
    while True:
        lc.handle()

thread = threading.Thread(target=lcm_thread)
thread.daemon = True
thread.start()

# Wait for transforms to become available
time.sleep(1.0)

# Look up transform
try:
    # Basic transform lookup
    transform = buffer.lookup_transform("world", "link6", time.time())
    
    print(f"Transform from world to link6:")
    print(f"  Position: ({transform.transform.translation.x}, "
          f"{transform.transform.translation.y}, "
          f"{transform.transform.translation.z})")
    print(f"  Rotation: ({transform.transform.rotation.w}, "
          f"{transform.transform.rotation.x}, "
          f"{transform.transform.rotation.y}, "
          f"{transform.transform.rotation.z})")
    
    # Transforming a point
    point = [1.0, 0.0, 0.0]  # Point in link6 frame
    
    # Extract transform components
    trans = [transform.transform.translation.x, 
             transform.transform.translation.y, 
             transform.transform.translation.z]
    
    quat = [transform.transform.rotation.w,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z]
    
    # Apply the transform (simplified for example)
    # In practice, use a dedicated library like transforms3d for this
    import numpy as np
    from scipy.spatial.transform import Rotation
    
    # Convert quaternion to rotation matrix
    r = Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]])  # scipy uses x,y,z,w order
    rotmat = r.as_matrix()
    
    # Apply rotation then translation
    rotated = np.dot(rotmat, point)
    transformed = [rotated[0] + trans[0], rotated[1] + trans[1], rotated[2] + trans[2]]
    
    print(f"Transformed point: {transformed}")
    
except Exception as e:
    print(f"Error: {e}")
```

#### 2. Broadcasting a Transform

```python
import time
import lcm
import tf_lcm_py
from lcm_msgs.geometry_msgs import TransformStamped, Transform, Vector3, Quaternion
from lcm_msgs.std_msgs import Header

def main():
    # Create a broadcaster
    broadcaster = tf_lcm_py.TransformBroadcaster()
    
    # Create a transform
    transform = TransformStamped()
    
    # Fill in header
    transform.header = Header()
    transform.header.frame_id = "world"
    now = time.time()
    transform.header.stamp.sec = int(now)
    transform.header.stamp.nsec = int((now - int(now)) * 1e9)
    
    # Set child frame
    transform.child_frame_id = "robot_base"
    
    # Set translation and rotation
    transform.transform = Transform()
    transform.transform.translation = Vector3()
    transform.transform.rotation = Quaternion()
    
    transform.transform.translation.x = 1.0
    transform.transform.translation.y = 2.0
    transform.transform.translation.z = 0.5
    
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0
    transform.transform.rotation.w = 1.0
    
    # Broadcast the transform
    broadcaster.send_transform(transform)
    
    print(f"Broadcasted transform from {transform.header.frame_id} to {transform.child_frame_id}")

if __name__ == "__main__":
    main()
```

#### 2. Listening for Transforms

```python
import time
import datetime
import lcm
import tf_lcm_py

def main():
    # Create a listener with a new buffer
    listener = tf_lcm_py.TransformListener()
    
    # Get the buffer from the listener
    buffer = listener.get_buffer()
    
    # Current time as a system_clock timepoint
    now = datetime.datetime.now()
    
    try:
        # Look up a transform
        transform = buffer.lookup_transform("world", "robot_base", now)
        
        print(f"Found transform from {transform.header.frame_id} to {transform.child_frame_id}")
        print(f"  Translation: ({transform.transform.translation.x}, "
              f"{transform.transform.translation.y}, {transform.transform.translation.z})")
    except tf_lcm_py.TransformException as e:
        print(f"Exception: {str(e)}")

if __name__ == "__main__":
    main()
```

#### 3. Using Timeout

```python
import time
import datetime
import lcm
import tf_lcm_py

def main():
    # Create a listener with a new buffer
    listener = tf_lcm_py.TransformListener()
    buffer = listener.get_buffer()
    
    now = datetime.datetime.now()
    
    try:
        # Wait up to 5 seconds for the transform
        transform = buffer.lookup_transform("map", "robot_base", now, 5.0)
        
        print(f"Found transform: ({transform.transform.translation.x}, "
              f"{transform.transform.translation.y}, {transform.transform.translation.z})")
    except tf_lcm_py.TimeoutException as e:
        print(f"Timeout waiting for transform: {str(e)}")
    except tf_lcm_py.TransformException as e:
        print(f"Exception: {str(e)}")

if __name__ == "__main__":
    main()
```

## Implementation Details

### LCM Communication

The TF library communicates using the following LCM channels:

- `tf#tf2_msgs.TFMessage` - Regular transforms
- `tf_static#tf2_msgs.TFMessage` - Static transforms

### Message Types

The library uses the following LCM message types:

- `tf2_msgs.TFMessage` - Container for transforms
- `geometry_msgs.TransformStamped` - Single transform with header
- `geometry_msgs.Transform` - Transform data (translation and rotation)
- `geometry_msgs.Vector3` - 3D vector for translation
- `geometry_msgs.Quaternion` - Quaternion for rotation

## API Reference

### C++ API

#### Buffer

- `Buffer(double cache_time = 10.0)` - Constructor
- `bool setTransform(const geometry_msgs::TransformStamped& transform, const std::string& authority, bool is_static = false)` - Store a transform
- `bool setTransforms(const std::vector<geometry_msgs::TransformStamped>& transforms, const std::string& authority, bool is_static = false)` - Store multiple transforms
- `geometry_msgs::TransformStamped lookupTransform(const std::string& target_frame, const std::string& source_frame, const std::chrono::system_clock::time_point& time, const std::chrono::duration<double>& timeout = std::chrono::duration<double>(0.0))` - Look up a transform
- `geometry_msgs::TransformStamped lookupTransform(const std::string& target_frame, const std::chrono::system_clock::time_point& target_time, const std::string& source_frame, const std::chrono::system_clock::time_point& source_time, const std::string& fixed_frame, const std::chrono::duration<double>& timeout = std::chrono::duration<double>(0.0))` - Look up a transform with time travel
- `bool canTransform(const std::string& target_frame, const std::string& source_frame, const std::chrono::system_clock::time_point& time, std::string* error_msg = nullptr)` - Check if transform is possible
- `std::vector<std::string> getAllFrameNames() const` - Get all frame IDs
- `void clear()` - Clear all transforms

#### TransformBroadcaster

- `TransformBroadcaster()` - Constructor
- `TransformBroadcaster(std::shared_ptr<lcm::LCM> lcm)` - Constructor with provided LCM
- `void sendTransform(const geometry_msgs::TransformStamped& transform)` - Send a transform
- `void sendTransform(const std::vector<geometry_msgs::TransformStamped>& transforms)` - Send multiple transforms

#### StaticTransformBroadcaster

- `StaticTransformBroadcaster()` - Constructor
- `StaticTransformBroadcaster(std::shared_ptr<lcm::LCM> lcm)` - Constructor with provided LCM
- `void sendTransform(const geometry_msgs::TransformStamped& transform)` - Send a static transform
- `void sendTransform(const std::vector<geometry_msgs::TransformStamped>& transforms)` - Send multiple static transforms

#### TransformListener

- `TransformListener(double buffer_size = 10.0)` - Constructor with new buffer
- `TransformListener(Buffer& buffer)` - Constructor with existing buffer
- `TransformListener(std::shared_ptr<lcm::LCM> lcm, Buffer& buffer)` - Constructor with provided LCM and buffer
- `Buffer& getBuffer()` - Get the buffer

### Python API

The Python API mirrors the C++ API but with Python naming conventions (snake_case instead of camelCase):

- `Buffer.set_transform(transform, authority, is_static=False)`
- `Buffer.lookup_transform(target_frame, source_frame, time, timeout=0.0)`
- etc.

## License

MIT License
