#include <iostream>
#include "tf_lcm/buffer.hpp"

// Minimal test for core TF functionality
int main() {
    std::cout << "Running minimal TF test..." << std::endl;
    
    // Create a buffer
    tf_lcm::Buffer buffer(10.0);
    
    // Current time for transforms
    auto now = std::chrono::system_clock::now();
    auto time_sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
    auto time_nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch() % std::chrono::seconds(1)).count();
    
    // Create a single transform: world -> robot
    geometry_msgs::TransformStamped t1;
    t1.header.frame_id = "world";
    t1.header.stamp.sec = time_sec;
    t1.header.stamp.nsec = time_nsec;
    t1.child_frame_id = "robot";
    t1.transform.translation.x = 1.0;
    t1.transform.translation.y = 2.0;
    t1.transform.translation.z = 0.0;
    t1.transform.rotation.w = 1.0;  // Identity rotation
    
    std::cout << "Adding transform: world -> robot" << std::endl;
    buffer.setTransform(t1, "test_authority");
    
    try {
        // Direct lookup
        auto result = buffer.lookupTransform("world", "robot", now);
        std::cout << "Successfully looked up transform world -> robot:" << std::endl;
        std::cout << "  Translation: (" 
                  << result.transform.translation.x << ", "
                  << result.transform.translation.y << ", "
                  << result.transform.translation.z << ")" << std::endl;
        
        // Reverse lookup
        auto reverse = buffer.lookupTransform("robot", "world", now);
        std::cout << "Successfully looked up reverse transform robot -> world:" << std::endl;
        std::cout << "  Translation: (" 
                  << reverse.transform.translation.x << ", "
                  << reverse.transform.translation.y << ", "
                  << reverse.transform.translation.z << ")" << std::endl;
        
        std::cout << "PASS: Basic transforms work correctly!" << std::endl;
        return 0;
    } catch (const tf_lcm::TransformException& ex) {
        std::cerr << "FAIL: Transform exception: " << ex.what() << std::endl;
        return 1;
    }
}
