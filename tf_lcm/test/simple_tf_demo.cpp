#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include "tf_lcm/buffer.hpp"
#include "tf_lcm/broadcaster.hpp"

// Simple demonstration of TF functionality
int main(int argc, char** argv) {
    std::cout << "TF LCM Simple Demonstration" << std::endl;
    std::cout << "==========================" << std::endl;
    
    // Create a buffer
    tf_lcm::Buffer buffer(10.0);  // 10 seconds buffer
    
    // Current time
    auto now = std::chrono::system_clock::now();
    auto time_sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
    auto time_nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch() % std::chrono::seconds(1)).count();
    
    // Create a simple transform chain: world -> robot -> camera
    std::vector<geometry_msgs::TransformStamped> transforms;
    
    // world -> robot transform
    geometry_msgs::TransformStamped t1;
    t1.header.frame_id = "world";
    t1.header.stamp.sec = time_sec;
    t1.header.stamp.nsec = time_nsec;
    t1.child_frame_id = "robot";
    t1.transform.translation.x = 1.0;
    t1.transform.translation.y = 2.0;
    t1.transform.translation.z = 0.0;
    t1.transform.rotation.w = 1.0;  // Identity rotation
    transforms.push_back(t1);
    
    // robot -> camera transform
    geometry_msgs::TransformStamped t2;
    t2.header.frame_id = "robot";
    t2.header.stamp.sec = time_sec;
    t2.header.stamp.nsec = time_nsec;
    t2.child_frame_id = "camera";
    t2.transform.translation.x = 0.2;
    t2.transform.translation.y = 0.0;
    t2.transform.translation.z = 0.3;
    t2.transform.rotation.w = 1.0;  // Identity rotation
    transforms.push_back(t2);
    
    // Add transforms to buffer
    buffer.setTransforms(transforms, "demo_source");
    
    std::cout << "Added transforms to buffer:" << std::endl;
    std::cout << "- world -> robot" << std::endl;
    std::cout << "- robot -> camera" << std::endl;
    
    // Demonstrate lookups
    try {
        // Direct lookup
        auto direct = buffer.lookupTransform("world", "robot", now);
        std::cout << "\nDirect lookup (world -> robot):" << std::endl;
        std::cout << "  Translation: (" 
                  << direct.transform.translation.x << ", " 
                  << direct.transform.translation.y << ", "
                  << direct.transform.translation.z << ")" << std::endl;
        
        // Multi-hop lookup
        auto multihop = buffer.lookupTransform("world", "camera", now);
        std::cout << "\nMulti-hop lookup (world -> camera):" << std::endl;
        std::cout << "  Translation: (" 
                  << multihop.transform.translation.x << ", " 
                  << multihop.transform.translation.y << ", "
                  << multihop.transform.translation.z << ")" << std::endl;
        
        // Reverse lookup
        auto reverse = buffer.lookupTransform("camera", "world", now);
        std::cout << "\nReverse lookup (camera -> world):" << std::endl;
        std::cout << "  Translation: (" 
                  << reverse.transform.translation.x << ", " 
                  << reverse.transform.translation.y << ", "
                  << reverse.transform.translation.z << ")" << std::endl;
        
        // Verify canTransform
        bool can_transform = buffer.canTransform("world", "camera", now);
        std::cout << "\nCan transform from world to camera: " 
                  << (can_transform ? "yes" : "no") << std::endl;
        
        bool cannot_transform = buffer.canTransform("world", "nonexistent", now);
        std::cout << "Can transform from world to nonexistent: " 
                  << (cannot_transform ? "yes" : "no") << std::endl;
        
    } catch (const tf_lcm::TransformException& ex) {
        std::cerr << "Transform exception: " << ex.what() << std::endl;
        return 1;
    }
    
    std::cout << "\nDemo completed successfully!" << std::endl;
    return 0;
}
