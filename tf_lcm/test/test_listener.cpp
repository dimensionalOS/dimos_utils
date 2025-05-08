#include <iostream>
#include <chrono>
#include <thread>
#include <lcm/lcm-cpp.hpp>
#include "tf_lcm/listener.hpp"

int main(int argc, char** argv) {
    std::cout << "Testing TransformListener..." << std::endl;
    
    // Create LCM instance
    auto lcm = std::make_shared<lcm::LCM>();
    if (!lcm->good()) {
        std::cerr << "Failed to initialize LCM" << std::endl;
        return 1;
    }
    
    // Create a listener with a buffer
    tf_lcm::Buffer buffer(10.0);  // 10 seconds buffer
    tf_lcm::TransformListener listener(lcm, buffer);
    
    // Start a thread to handle LCM messages in the background
    std::atomic<bool> keep_handling(true);
    std::thread lcm_handler_thread([&lcm, &keep_handling]() {
        while (keep_handling) {
            lcm->handleTimeout(100); // Process messages with 100ms timeout
        }
    });
    
    std::cout << "Listening for transforms. Run the broadcaster test in another terminal." << std::endl;
    
    // Wait for transforms and try to look them up (reduced iterations for faster test completion)
    for (int i = 0; i < 15; i++) {
        try {
            // Get the current time
            auto now = std::chrono::system_clock::now();
            
            // Try to look up the transform
            geometry_msgs::TransformStamped transform = buffer.lookupTransform("world", "robot_base", now);
            
            std::cout << "Found transform from " << transform.header.frame_id
                      << " to " << transform.child_frame_id << std::endl;
            std::cout << "  Translation: (" << transform.transform.translation.x
                      << ", " << transform.transform.translation.y
                      << ", " << transform.transform.translation.z << ")" << std::endl;
            std::cout << "  Rotation: (" << transform.transform.rotation.x
                      << ", " << transform.transform.rotation.y
                      << ", " << transform.transform.rotation.z
                      << ", " << transform.transform.rotation.w << ")" << std::endl;
        } catch (const tf_lcm::TransformException& ex) {
            std::cerr << "Exception: " << ex.what() << std::endl;
        }
        
        // Sleep for a short time
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    
    std::cout << "Transform listening test completed." << std::endl;
    
    // Stop the LCM handler thread
    keep_handling = false;
    if (lcm_handler_thread.joinable()) {
        lcm_handler_thread.join();
    }
    
    return 0;
}
