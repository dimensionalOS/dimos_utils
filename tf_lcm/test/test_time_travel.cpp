#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <lcm/lcm-cpp.hpp>
#include "tf_lcm/buffer.hpp"
#include "tf_lcm/broadcaster.hpp"
#include "tf_lcm/listener.hpp"

int main(int argc, char** argv) {
    std::cout << "Testing time travel functionality..." << std::endl;
    
    // Create an LCM instance to share
    auto lcm = std::make_shared<lcm::LCM>();
    if (!lcm->good()) {
        std::cerr << "Failed to initialize LCM" << std::endl;
        return 1;
    }
    
    // Create a broadcaster
    tf_lcm::TransformBroadcaster broadcaster(lcm);
    
    // Create a buffer and listener
    tf_lcm::Buffer buffer(10.0);  // 10 seconds buffer
    tf_lcm::TransformListener listener(lcm, buffer);
    
    // Start a thread to handle LCM messages in the background
    std::atomic<bool> keep_handling(true);
    std::thread lcm_handler_thread([&lcm, &keep_handling]() {
        while (keep_handling) {
            lcm->handleTimeout(100); // Process messages with 100ms timeout
        }
    });
    
    // Create transforms at different times
    
    // Get current time
    auto now = std::chrono::system_clock::now();
    
    // Create timestamps for various points in time
    auto time_0 = now;
    auto time_1 = now + std::chrono::seconds(1);
    auto time_2 = now + std::chrono::seconds(2);
    auto time_3 = now + std::chrono::seconds(3);
    
    // Transform at time_0: robot at position (0, 0, 0)
    {
        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = "map";
        transform.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(time_0.time_since_epoch()).count();
        transform.header.stamp.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
            time_0.time_since_epoch() % std::chrono::seconds(1)).count();
        transform.child_frame_id = "robot";
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        
        buffer.setTransform(transform, "test_source");
        broadcaster.sendTransform(transform);
        std::cout << "Added transform at time_0: robot at (0, 0, 0)" << std::endl;
    }
    
    // Transform at time_1: robot at position (1, 0, 0)
    {
        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = "map";
        transform.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(time_1.time_since_epoch()).count();
        transform.header.stamp.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
            time_1.time_since_epoch() % std::chrono::seconds(1)).count();
        transform.child_frame_id = "robot";
        transform.transform.translation.x = 1.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        
        buffer.setTransform(transform, "test_source");
        broadcaster.sendTransform(transform);
        std::cout << "Added transform at time_1: robot at (1, 0, 0)" << std::endl;
    }
    
    // Transform at time_2: robot at position (1, 1, 0)
    {
        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = "map";
        transform.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(time_2.time_since_epoch()).count();
        transform.header.stamp.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
            time_2.time_since_epoch() % std::chrono::seconds(1)).count();
        transform.child_frame_id = "robot";
        transform.transform.translation.x = 1.0;
        transform.transform.translation.y = 1.0;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        
        buffer.setTransform(transform, "test_source");
        broadcaster.sendTransform(transform);
        std::cout << "Added transform at time_2: robot at (1, 1, 0)" << std::endl;
    }
    
    // Transform at time_3: robot at position (2, 1, 0)
    {
        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = "map";
        transform.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(time_3.time_since_epoch()).count();
        transform.header.stamp.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
            time_3.time_since_epoch() % std::chrono::seconds(1)).count();
        transform.child_frame_id = "robot";
        transform.transform.translation.x = 2.0;
        transform.transform.translation.y = 1.0;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        
        buffer.setTransform(transform, "test_source");
        broadcaster.sendTransform(transform);
        std::cout << "Added transform at time_3: robot at (2, 1, 0)" << std::endl;
    }
    
    // Add a static transform for a sensor on the robot
    {
        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = "robot";
        transform.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(time_0.time_since_epoch()).count();
        transform.header.stamp.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
            time_0.time_since_epoch() % std::chrono::seconds(1)).count();
        transform.child_frame_id = "sensor";
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.5;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        
        buffer.setTransform(transform, "test_source", true);  // true for static transform
        std::cout << "Added static transform: sensor at (0, 0.5, 0) relative to robot" << std::endl;
    }
    
    // Sleep a short time to ensure transforms are processed
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Look up transforms at different times
    try {
        // Lookup at time_0
        auto transform_0 = buffer.lookupTransform("map", "robot", time_0);
        std::cout << "Transform at time_0:" << std::endl;
        std::cout << "  Translation: (" << transform_0.transform.translation.x
                  << ", " << transform_0.transform.translation.y
                  << ", " << transform_0.transform.translation.z << ")" << std::endl;
        
        // Lookup at time_1
        auto transform_1 = buffer.lookupTransform("map", "robot", time_1);
        std::cout << "Transform at time_1:" << std::endl;
        std::cout << "  Translation: (" << transform_1.transform.translation.x
                  << ", " << transform_1.transform.translation.y
                  << ", " << transform_1.transform.translation.z << ")" << std::endl;
        
        // Lookup at time_2
        auto transform_2 = buffer.lookupTransform("map", "robot", time_2);
        std::cout << "Transform at time_2:" << std::endl;
        std::cout << "  Translation: (" << transform_2.transform.translation.x
                  << ", " << transform_2.transform.translation.y
                  << ", " << transform_2.transform.translation.z << ")" << std::endl;
        
        // Lookup at time_3
        auto transform_3 = buffer.lookupTransform("map", "robot", time_3);
        std::cout << "Transform at time_3:" << std::endl;
        std::cout << "  Translation: (" << transform_3.transform.translation.x
                  << ", " << transform_3.transform.translation.y
                  << ", " << transform_3.transform.translation.z << ")" << std::endl;
        
        // Now demonstrate the time travel feature by looking up sensor position in map frame at different times
        
        // Sensor in map frame at time_0
        auto sensor_0 = buffer.lookupTransform("map", "sensor", time_0);
        std::cout << "Sensor in map frame at time_0:" << std::endl;
        std::cout << "  Translation: (" << sensor_0.transform.translation.x
                  << ", " << sensor_0.transform.translation.y
                  << ", " << sensor_0.transform.translation.z << ")" << std::endl;
        
        // Sensor in map frame at time_3
        auto sensor_3 = buffer.lookupTransform("map", "sensor", time_3);
        std::cout << "Sensor in map frame at time_3:" << std::endl;
        std::cout << "  Translation: (" << sensor_3.transform.translation.x
                  << ", " << sensor_3.transform.translation.y
                  << ", " << sensor_3.transform.translation.z << ")" << std::endl;
        
        // Advanced: Use the fixed frame feature to transform between different times
        // This transforms a point from sensor at time_1 to robot at time_3, using map as the fixed frame
        auto advanced = buffer.lookupTransform("robot", time_3, "sensor", time_1, "map");
        std::cout << "Advanced transform (sensor at time_1 to robot at time_3):" << std::endl;
        std::cout << "  Translation: (" << advanced.transform.translation.x
                  << ", " << advanced.transform.translation.y
                  << ", " << advanced.transform.translation.z << ")" << std::endl;
        
    } catch (const tf_lcm::TransformException& ex) {
        std::cerr << "Exception looking up transform: " << ex.what() << std::endl;
    }
    
    std::cout << "Time travel test completed." << std::endl;
    
    // Stop the LCM handler thread
    keep_handling = false;
    if (lcm_handler_thread.joinable()) {
        lcm_handler_thread.join();
    }
    
    return 0;
}
