#include <iostream>
#include <chrono>
#include <thread>
#include <lcm/lcm-cpp.hpp>
#include "tf_lcm/broadcaster.hpp"
#include "geometry_msgs/TransformStamped.hpp"
#include "std_msgs/Header.hpp"

int main(int argc, char** argv) {
    std::cout << "Testing TransformBroadcaster..." << std::endl;
    
    // Create LCM instance
    auto lcm = std::make_shared<lcm::LCM>();
    if (!lcm->good()) {
        std::cerr << "Failed to initialize LCM" << std::endl;
        return 1;
    }
    
    // Create a broadcaster with the specific LCM instance
    tf_lcm::TransformBroadcaster broadcaster(lcm);
    
    // Create a transform
    geometry_msgs::TransformStamped transform;
    
    // Fill in header
    transform.header.frame_id = "world";
    transform.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    transform.header.stamp.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch() % std::chrono::seconds(1)).count();
    
    // Set child frame
    transform.child_frame_id = "robot_base";
    
    // Set translation
    transform.transform.translation.x = 1.0;
    transform.transform.translation.y = 2.0;
    transform.transform.translation.z = 0.5;
    
    // Set rotation (identity quaternion)
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    
    // Broadcast the transform
    broadcaster.sendTransform(transform);
    
    std::cout << "Broadcasted transform from " << transform.header.frame_id
              << " to " << transform.child_frame_id << std::endl;
    
    // Continue broadcasting for a short time so that a listener can pick it up (reduced iterations for faster test completion)
    for (int i = 0; i < 15; i++) {
        // Update timestamp
        transform.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        transform.header.stamp.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch() % std::chrono::seconds(1)).count();
        
        // Send the transform
        broadcaster.sendTransform(transform);
        
        // Sleep for a short time
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << "Transform broadcasting test completed." << std::endl;
    
    return 0;
}
