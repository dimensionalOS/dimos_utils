#include <iostream>
#include <chrono>
#include <thread>
#include <lcm/lcm-cpp.hpp>
#include "tf_lcm/buffer.hpp"
#include "tf_lcm/broadcaster.hpp"
#include "tf_lcm/listener.hpp"

// Helper function for delayed transform broadcast
void publishTransformWithDelay(tf_lcm::TransformBroadcaster& broadcaster,
                             const std::string& parent_frame,
                             const std::string& child_frame,
                             int delay_ms) {
    std::cout << "Will publish transform from " << parent_frame << " to " << child_frame
              << " after " << delay_ms << "ms delay" << std::endl;
    
    // Sleep for the specified delay
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    
    // Create a transform
    geometry_msgs::TransformStamped transform;
    
    // Fill in header
    transform.header.frame_id = parent_frame;
    transform.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    transform.header.stamp.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch() % std::chrono::seconds(1)).count();
    
    // Set child frame
    transform.child_frame_id = child_frame;
    
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
    
    std::cout << "Published transform from " << parent_frame << " to " << child_frame << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "Testing transform timeout functionality..." << std::endl;
    
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
    
    // Start a thread to publish a transform after a shorter delay
    std::thread publish_thread(publishTransformWithDelay, std::ref(broadcaster), "world", "delayed_frame", 500);
    
    // Start a thread to handle LCM messages in the background
    std::atomic<bool> keep_handling(true);
    std::thread lcm_handler_thread([&lcm, &keep_handling]() {
        while (keep_handling) {
            lcm->handleTimeout(100); // Process messages with 100ms timeout
        }
    });
    
    // Try to look up the transform with different timeouts
    try {
        std::cout << "Attempting lookup with 0.2s timeout..." << std::endl;
        auto now = std::chrono::system_clock::now();
        // This should timeout since the transform arrives after 500ms
        auto transform = buffer.lookupTransform("world", "delayed_frame", now,
                                             std::chrono::duration<double>(0.2));
        
        std::cout << "Found transform (this shouldn't happen):" << std::endl;
        std::cout << "  Translation: (" << transform.transform.translation.x
                  << ", " << transform.transform.translation.y
                  << ", " << transform.transform.translation.z << ")" << std::endl;
    } catch (const tf_lcm::TimeoutException& ex) {
        std::cout << "Expected timeout exception: " << ex.what() << std::endl;
    } catch (const tf_lcm::TransformException& ex) {
        std::cerr << "Unexpected exception: " << ex.what() << std::endl;
    }
    
    // Try again with a longer timeout that should succeed
    try {
        std::cout << "Attempting lookup with 1.0s timeout (should succeed)..." << std::endl;
        auto now = std::chrono::system_clock::now();
        // This should succeed since the timeout is longer than the delay (500ms)
        auto transform = buffer.lookupTransform("world", "delayed_frame", now,
                                             std::chrono::duration<double>(1.0));
        
        std::cout << "Found transform:" << std::endl;
        std::cout << "  Translation: (" << transform.transform.translation.x
                  << ", " << transform.transform.translation.y
                  << ", " << transform.transform.translation.z << ")" << std::endl;
    } catch (const tf_lcm::TransformException& ex) {
        std::cerr << "Unexpected exception: " << ex.what() << std::endl;
    }
    
    // Wait for the publishing thread to finish
    if (publish_thread.joinable()) {
        publish_thread.join();
    }
    
    // Stop the LCM handler thread
    keep_handling = false;
    if (lcm_handler_thread.joinable()) {
        lcm_handler_thread.join();
    }
    
    std::cout << "Transform timeout test completed." << std::endl;
    
    return 0;
}
