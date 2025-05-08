#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include "tf_lcm/buffer.hpp"

/**
 * This test demonstrates the time travel functionality in the TF library.
 * It shows how transforms can be looked up at specific times, allowing
 * for interpolation between transforms and tracking an object's position
 * over time.
 */
int main(int argc, char** argv) {
    std::cout << "=== Time Travel Functionality Demo ===" << std::endl;
    
    // Create a buffer with 10-second cache time
    tf_lcm::Buffer buffer(10.0);
    
    // Helper function to print a transform
    auto print_transform = [](const std::string& label, const geometry_msgs::TransformStamped& transform) {
        std::cout << label << ":" << std::endl;
        std::cout << "  Header frame: " << transform.header.frame_id << std::endl;
        std::cout << "  Child frame: " << transform.child_frame_id << std::endl;
        std::cout << "  Translation: (" 
                  << transform.transform.translation.x << ", "
                  << transform.transform.translation.y << ", "
                  << transform.transform.translation.z << ")" << std::endl;
        std::cout << "  Timestamp: " 
                  << transform.header.stamp.sec << "." 
                  << std::setfill('0') << std::setw(9) << transform.header.stamp.nsec 
                  << std::endl;
    };
    
    // Helper function to create a timestamp
    // Returns seconds and nanoseconds components for time stamps
    auto create_timestamp = [](const std::chrono::system_clock::time_point& time_point,
                               int64_t& sec, int32_t& nsec) {
        sec = std::chrono::duration_cast<std::chrono::seconds>(time_point.time_since_epoch()).count();
        nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
            time_point.time_since_epoch() % std::chrono::seconds(1)).count();
    };
    
    // Get current time as base
    auto base_time = std::chrono::system_clock::now();
    
    // Create timestamps for transforms at various points in time
    std::cout << "\n1. Creating transforms at different times:" << std::endl;
    
    // Time points at 0, 1, 2, and 3 seconds from base time
    auto time_0s = base_time;
    auto time_1s = base_time + std::chrono::seconds(1);
    auto time_2s = base_time + std::chrono::seconds(2);
    auto time_3s = base_time + std::chrono::seconds(3);
    
    // Create transforms for "robot" frame at different positions over time
    
    // At time_0s: robot at (0,0,0)
    {
        int64_t sec;
        int32_t nsec;
        create_timestamp(time_0s, sec, nsec);
        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = "world";  // Parent frame
        transform.header.stamp.sec = sec;
        transform.header.stamp.nsec = nsec;
        transform.child_frame_id = "robot";   // Child frame
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.w = 1.0;  // Identity rotation
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        
        buffer.setTransform(transform, "time_travel_demo");
        std::cout << "  Added: robot at (0,0,0) at t=0s" << std::endl;
    }
    
    // At time_1s: robot at (1,0,0)
    {
        int64_t sec;
        int32_t nsec;
        create_timestamp(time_1s, sec, nsec);
        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = "world";
        transform.header.stamp.sec = sec;
        transform.header.stamp.nsec = nsec;
        transform.child_frame_id = "robot";
        transform.transform.translation.x = 1.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        
        buffer.setTransform(transform, "time_travel_demo");
        std::cout << "  Added: robot at (1,0,0) at t=1s" << std::endl;
    }
    
    // At time_2s: robot at (1,1,0)
    {
        int64_t sec;
        int32_t nsec;
        create_timestamp(time_2s, sec, nsec);
        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = "world";
        transform.header.stamp.sec = sec;
        transform.header.stamp.nsec = nsec;
        transform.child_frame_id = "robot";
        transform.transform.translation.x = 1.0;
        transform.transform.translation.y = 1.0;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        
        buffer.setTransform(transform, "time_travel_demo");
        std::cout << "  Added: robot at (1,1,0) at t=2s" << std::endl;
    }
    
    // At time_3s: robot at (2,1,0)
    {
        int64_t sec;
        int32_t nsec;
        create_timestamp(time_3s, sec, nsec);
        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = "world";
        transform.header.stamp.sec = sec;
        transform.header.stamp.nsec = nsec;
        transform.child_frame_id = "robot";
        transform.transform.translation.x = 2.0;
        transform.transform.translation.y = 1.0;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        
        buffer.setTransform(transform, "time_travel_demo");
        std::cout << "  Added: robot at (2,1,0) at t=3s" << std::endl;
    }
    
    // Add a static transform for a sensor on the robot
    {
        int64_t sec;
        int32_t nsec;
        create_timestamp(time_0s, sec, nsec);
        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = "robot";
        transform.header.stamp.sec = sec;
        transform.header.stamp.nsec = nsec;
        transform.child_frame_id = "sensor";
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.5;
        transform.transform.translation.z = 0.1;
        transform.transform.rotation.w = 1.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        
        buffer.setTransform(transform, "time_travel_demo", true);  // true for static transform
        std::cout << "  Added: static sensor at (0,0.5,0.1) relative to robot" << std::endl;
    }
    
    // Part 2: Looking up transforms at specific times
    std::cout << "\n2. Demonstrating time travel - looking up transforms at specific times:" << std::endl;
    
    try {
        // Look up robot position at time_0s (t=0s)
        auto robot_at_0s = buffer.lookupTransform("world", "robot", time_0s);
        print_transform("Robot position at t=0s", robot_at_0s);
        
        // Look up robot position at time_2s (t=2s)
        auto robot_at_2s = buffer.lookupTransform("world", "robot", time_2s);
        print_transform("Robot position at t=2s", robot_at_2s);
        
        // Look up sensor position at time_0s
        auto sensor_at_0s = buffer.lookupTransform("world", "sensor", time_0s);
        print_transform("Sensor position at t=0s", sensor_at_0s);
        
        // Look up sensor position at time_3s
        auto sensor_at_3s = buffer.lookupTransform("world", "sensor", time_3s);
        print_transform("Sensor position at t=3s", sensor_at_3s);
        
        // Part 3: Advanced time travel - transform between different times
        std::cout << "\n3. Advanced time travel - transforming between different times:" << std::endl;
        
        // Transform sensor at time_1s to robot at time_3s, with world as the fixed frame
        // This is useful for calculating where a point that was observed at time_1s
        // would be relative to the robot's current position at time_3s
        auto advanced_transform = buffer.lookupTransform(
            "robot", time_3s,     // target frame and time
            "sensor", time_1s,    // source frame and time
            "world"               // fixed frame (common reference)
        );
        
        print_transform("Sensor at t=1s relative to robot at t=3s", advanced_transform);
        
        std::cout << "\nTime travel demonstration completed successfully!" << std::endl;
        
    } catch (const tf_lcm::TransformException& ex) {
        std::cerr << "Exception during time travel: " << ex.what() << std::endl;
        return 1;
    }
    
    return 0;
}
