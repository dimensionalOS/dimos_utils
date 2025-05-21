# C++ LCM Message Types

This directory contains C++ bindings for LCM (Lightweight Communications and Marshalling) message types generated from ROS message definitions.

## Prerequisites

Before using these message types, you need to install the LCM library:

```bash
# On macOS with Homebrew
brew install lcm

# On Ubuntu/Debian
sudo apt-get install liblcm-dev

# Building from source
git clone https://github.com/lcm-proj/lcm.git
cd lcm
mkdir build && cd build
cmake ..
make
sudo make install
```

## Using the Generated LCM Types

There are several ways to include and use these generated LCM message types in your C++ projects:

### Option A: Using CMake (Recommended)

1. Create a `CMakeLists.txt` file in your project:

```cmake
cmake_minimum_required(VERSION 3.10)
project(my_lcm_project)

# Find LCM package
find_package(lcm REQUIRED)

# Specify include directories 
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    /path/to/ros_to_lcm  # Path to the root where cpp_lcm_msgs is located
)

# Add executable
add_executable(my_program src/main.cpp)

# Link against LCM
target_link_libraries(my_program lcm)
```

2. Example C++ file (`src/main.cpp`):

```cpp
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "cpp_lcm_msgs/actionlib_msgs/GoalID.hpp"

int main() {
    // Initialize LCM
    lcm::LCM lcm;
    if (!lcm.good()) {
        std::cerr << "Failed to initialize LCM" << std::endl;
        return 1;
    }
    
    // Create a message
    actionlib_msgs::GoalID goal_id;
    goal_id.id = "test_goal_1";
    
    // Publish the message
    lcm.publish("GOAL_CHANNEL", &goal_id);
    
    std::cout << "Published a GoalID with id: " << goal_id.id << std::endl;
    return 0;
}
```

3. Build the project:

```bash
mkdir build && cd build
cmake ..
make
```

### Option B: Using a Makefile

1. Create a `Makefile`:

```makefile
CXX = g++
CXXFLAGS = -std=c++11 -I/path/to/ros_to_lcm
LDFLAGS = $(shell pkg-config --libs lcm)

my_program: main.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f my_program
```

2. Example C++ file (`main.cpp`):

```cpp
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "cpp_lcm_msgs/sensor_msgs/MagneticField.hpp"

int main() {
    // Initialize LCM
    lcm::LCM lcm;
    if (!lcm.good()) {
        std::cerr << "Failed to initialize LCM" << std::endl;
        return 1;
    }
    
    // Create a message
    sensor_msgs::MagneticField magnetic_field;
    magnetic_field.header.frame_id = "base_link";
    magnetic_field.header.stamp.sec = 1234;
    magnetic_field.header.stamp.nsec = 5678;
    
    // Set magnetic field vector
    magnetic_field.magnetic_field.x = 1.0;
    magnetic_field.magnetic_field.y = 2.0;
    magnetic_field.magnetic_field.z = 3.0;
    
    // Publish the message
    lcm.publish("MAGNETIC_FIELD", &magnetic_field);
    
    std::cout << "Published a MagneticField message" << std::endl;
    return 0;
}
```

3. Build the project:

```bash
make
```

### Option C: Direct Compilation

1. Compile your program directly:

```bash
g++ -std=c++11 -I/path/to/ros_to_lcm -o my_program main.cpp $(pkg-config --libs lcm)
```

2. Example C++ file (`main.cpp`):

```cpp
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "cpp_lcm_msgs/geometry_msgs/Twist.hpp"

int main() {
    // Initialize LCM
    lcm::LCM lcm;
    if (!lcm.good()) {
        std::cerr << "Failed to initialize LCM" << std::endl;
        return 1;
    }
    
    // Create a message
    geometry_msgs::Twist twist;
    twist.linear.x = 0.5;  // forward at 0.5 m/s
    twist.angular.z = 0.2; // turning at 0.2 rad/s
    
    // Publish the message
    lcm.publish("CMD_VEL", &twist);
    
    std::cout << "Published a Twist command" << std::endl;
    return 0;
}
```

## Creating a Subscriber

Here's how to create a subscriber for LCM messages:

```cpp
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "cpp_lcm_msgs/geometry_msgs/Twist.hpp"

class TwistHandler {
public:
    void handleMessage(const lcm::ReceiveBuffer* rbuf,
                      const std::string& channel,
                      const geometry_msgs::Twist* twist) {
        std::cout << "Received message on channel: " << channel << std::endl;
        std::cout << "  Linear velocity: [" 
                  << twist->linear.x << ", " 
                  << twist->linear.y << ", " 
                  << twist->linear.z << "]" << std::endl;
        std::cout << "  Angular velocity: [" 
                  << twist->angular.x << ", " 
                  << twist->angular.y << ", " 
                  << twist->angular.z << "]" << std::endl;
    }
};

int main() {
    lcm::LCM lcm;
    if (!lcm.good()) {
        std::cerr << "Failed to initialize LCM" << std::endl;
        return 1;
    }
    
    TwistHandler handlerObject;
    lcm.subscribe("CMD_VEL", &TwistHandler::handleMessage, &handlerObject);
    
    // Wait for messages
    while (0 == lcm.handle()) {
        // Message handling happens in the callback
    }
    
    return 0;
}
```

## Tips

1. **Path Management**: Always make sure the include path to the message types is correctly set in your build system.

2. **Relative Include Paths**: When including message types, use the relative path from your include directory:
   ```cpp
   #include "cpp_lcm_msgs/geometry_msgs/Twist.hpp"
   ```

3. **LCM Channel Naming**: Choose consistent channel names for your messages.

4. **Error Handling**: Always check if the LCM instance is initialized properly using `lcm.good()`.

## Troubleshooting

1. **Missing Header Files**: Ensure your include paths are correctly set to point to the directory containing the `cpp_lcm_msgs` folder.

2. **Linking Errors**: Make sure you've linked against the LCM library with `-llcm`.

3. **Runtime Errors**: Verify that your LCM message types match on both publisher and subscriber sides.
