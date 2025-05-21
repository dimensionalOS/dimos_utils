#include <iostream>
#include <chrono>
#include <thread>
#include <lcm/lcm-cpp.hpp>
#include "tf_lcm/broadcaster.hpp"
#include "tf_lcm/listener.hpp"

int main(int argc, char** argv) {
    std::cout << "Testing StaticTransformBroadcaster..." << std::endl;
    
    // Create an LCM instance to share
    auto lcm = std::make_shared<lcm::LCM>();
    if (!lcm->good()) {
        std::cerr << "Failed to initialize LCM" << std::endl;
        return 1;
    }
    
    // Create a static broadcaster
    tf_lcm::StaticTransformBroadcaster static_broadcaster(lcm);
    
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
    
    // Create a static transform
    geometry_msgs::TransformStamped static_transform;
    
    // Fill in header
    static_transform.header.frame_id = "map";
    static_transform.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    static_transform.header.stamp.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch() % std::chrono::seconds(1)).count();
    
    // Set child frame
    static_transform.child_frame_id = "odom";
    
    // Set translation
    static_transform.transform.translation.x = 10.0;
    static_transform.transform.translation.y = 5.0;
    static_transform.transform.translation.z = 0.0;
    
    // Set rotation (identity quaternion)
    static_transform.transform.rotation.x = 0.0;
    static_transform.transform.rotation.y = 0.0;
    static_transform.transform.rotation.z = 0.0;
    static_transform.transform.rotation.w = 1.0;
    
    // Broadcast the static transform
    static_broadcaster.sendTransform(static_transform);
    
    std::cout << "Broadcasted static transform from " << static_transform.header.frame_id
              << " to " << static_transform.child_frame_id << std::endl;
    
    // Give time for static transform to be processed
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Add a regular transform
    tf_lcm::TransformBroadcaster regular_broadcaster(lcm);
    
    geometry_msgs::TransformStamped regular_transform;
    regular_transform.header.frame_id = "odom";
    regular_transform.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    regular_transform.header.stamp.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch() % std::chrono::seconds(1)).count();
    regular_transform.child_frame_id = "base_footprint";
    regular_transform.transform.translation.x = 1.0;
    regular_transform.transform.translation.y = 2.0;
    regular_transform.transform.translation.z = 0.0;
    regular_transform.transform.rotation.w = 1.0;
    
    regular_broadcaster.sendTransform(regular_transform);
    
    std::cout << "Broadcasted regular transform from " << regular_transform.header.frame_id
              << " to " << regular_transform.child_frame_id << std::endl;
    
    // Sleep a bit to let the listener receive transforms
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Give time for static transform to be processed
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Try to look up transform from map to base_footprint
    try {
        auto now = std::chrono::system_clock::now();
        auto transform = buffer.lookupTransform("map", "base_footprint", now);
        
        std::cout << "Found transform from map to base_footprint:" << std::endl;
        std::cout << "  Translation: (" << transform.transform.translation.x
                  << ", " << transform.transform.translation.y
                  << ", " << transform.transform.translation.z << ")" << std::endl;
    } catch (const tf_lcm::TransformException& ex) {
        std::cerr << "Exception looking up transform: " << ex.what() << std::endl;
    }
    
    std::cout << "Static transform test completed." << std::endl;
    
    // Stop the LCM handler thread
    keep_handling = false;
    if (lcm_handler_thread.joinable()) {
        lcm_handler_thread.join();
    }
    
    return 0;
}
