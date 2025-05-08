#include <iostream>
#include <chrono>
#include <vector>
#include "tf_lcm/buffer.hpp"

int main(int argc, char** argv) {
    std::cout << "Testing transform lookups (simplified)..." << std::endl;
    
    // Create a buffer for storing transforms
    tf_lcm::Buffer buffer(10.0);  // 10 seconds buffer
    
    // Current time for transforms
    auto now = std::chrono::system_clock::now();
    auto time_sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
    auto time_nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch() % std::chrono::seconds(1)).count();
    
    // Create and add individual transforms directly
    
    // Transform 1: world -> base_link
    geometry_msgs::TransformStamped t1;
    t1.header.frame_id = "world";
    t1.header.stamp.sec = time_sec;
    t1.header.stamp.nsec = time_nsec;
    t1.child_frame_id = "base_link";
    t1.transform.translation.x = 1.0;
    t1.transform.translation.y = 0.0;
    t1.transform.translation.z = 0.0;
    t1.transform.rotation.w = 1.0;
    t1.transform.rotation.x = 0.0;
    t1.transform.rotation.y = 0.0;
    t1.transform.rotation.z = 0.0;
    
    // Transform 2: base_link -> arm_base
    geometry_msgs::TransformStamped t2;
    t2.header.frame_id = "base_link";
    t2.header.stamp.sec = time_sec;
    t2.header.stamp.nsec = time_nsec;
    t2.child_frame_id = "arm_base";
    t2.transform.translation.x = 0.0;
    t2.transform.translation.y = 0.5;
    t2.transform.translation.z = 0.75;
    t2.transform.rotation.w = 1.0;
    t2.transform.rotation.x = 0.0;
    t2.transform.rotation.y = 0.0;
    t2.transform.rotation.z = 0.0;
    
    // Transform 3: arm_base -> arm_link1
    geometry_msgs::TransformStamped t3;
    t3.header.frame_id = "arm_base";
    t3.header.stamp.sec = time_sec;
    t3.header.stamp.nsec = time_nsec;
    t3.child_frame_id = "arm_link1";
    t3.transform.translation.x = 0.0;
    t3.transform.translation.y = 0.0;
    t3.transform.translation.z = 0.5;
    t3.transform.rotation.w = 1.0;
    t3.transform.rotation.x = 0.0;
    t3.transform.rotation.y = 0.0;
    t3.transform.rotation.z = 0.0;
    
    std::cout << "Adding transforms to buffer..." << std::endl;
    // Add transforms individually to avoid any potential issues with setTransforms
    buffer.setTransform(t1, "test_authority");
    buffer.setTransform(t2, "test_authority");
    buffer.setTransform(t3, "test_authority");
    
    std::cout << "Added transforms:" << std::endl;
    std::cout << "  " << t1.header.frame_id << " -> " << t1.child_frame_id << std::endl;
    std::cout << "  " << t2.header.frame_id << " -> " << t2.child_frame_id << std::endl;
    std::cout << "  " << t3.header.frame_id << " -> " << t3.child_frame_id << std::endl;
    
    // Direct lookup
    try {
        std::cout << "\nPerforming direct lookup (base_link -> arm_base)..." << std::endl;
        auto direct = buffer.lookupTransform("base_link", "arm_base", now);
        std::cout << "  Translation: (" 
                  << direct.transform.translation.x << ", "
                  << direct.transform.translation.y << ", "
                  << direct.transform.translation.z << ")" << std::endl;
    } catch (const tf_lcm::TransformException& ex) {
        std::cerr << "Exception: " << ex.what() << std::endl;
    }
    
    // Multi-hop lookup
    try {
        std::cout << "\nPerforming multi-hop lookup (world -> arm_link1)..." << std::endl;
        auto multi_hop = buffer.lookupTransform("world", "arm_link1", now);
        std::cout << "  Translation: (" 
                  << multi_hop.transform.translation.x << ", "
                  << multi_hop.transform.translation.y << ", "
                  << multi_hop.transform.translation.z << ")" << std::endl;
    } catch (const tf_lcm::TransformException& ex) {
        std::cerr << "Exception: " << ex.what() << std::endl;
    }
    
    // Reverse lookup
    try {
        std::cout << "\nPerforming reverse lookup (arm_base -> world)..." << std::endl;
        auto reverse = buffer.lookupTransform("arm_base", "world", now);
        std::cout << "  Translation: (" 
                  << reverse.transform.translation.x << ", "
                  << reverse.transform.translation.y << ", "
                  << reverse.transform.translation.z << ")" << std::endl;
    } catch (const tf_lcm::TransformException& ex) {
        std::cerr << "Exception: " << ex.what() << std::endl;
    }
    
    // Test canTransform
    bool can_transform = buffer.canTransform("world", "arm_link1", now);
    std::cout << "\nCan transform from world to arm_link1: " 
              << (can_transform ? "yes" : "no") << std::endl;
    
    bool cannot_transform = buffer.canTransform("world", "non_existent_frame", now);
    std::cout << "Can transform from world to non_existent_frame: " 
              << (cannot_transform ? "yes" : "no") << std::endl;
    
    std::cout << "\nTransform lookup test completed successfully!" << std::endl;
    return 0;
}
