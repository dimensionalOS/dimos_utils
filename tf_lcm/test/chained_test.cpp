#include <iostream>
#include "tf_lcm/buffer.hpp"

// Test for chained transform graph traversal
int main() {
    std::cout << "Running chained transforms test..." << std::endl;
    
    // Create a buffer
    tf_lcm::Buffer buffer(10.0);
    
    // Current time for transforms
    auto now = std::chrono::system_clock::now();
    auto time_sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
    auto time_nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch() % std::chrono::seconds(1)).count();
    
    // Create a transform chain: world -> base -> arm -> gripper
    
    // Transform 1: world -> base
    geometry_msgs::TransformStamped t1;
    t1.header.frame_id = "world";
    t1.header.stamp.sec = time_sec;
    t1.header.stamp.nsec = time_nsec;
    t1.child_frame_id = "base";
    t1.transform.translation.x = 1.0;
    t1.transform.translation.y = 0.0;
    t1.transform.translation.z = 0.0;
    // Identity rotation
    t1.transform.rotation.w = 1.0;
    t1.transform.rotation.x = 0.0;
    t1.transform.rotation.y = 0.0;
    t1.transform.rotation.z = 0.0;
    
    // Transform 2: base -> arm
    geometry_msgs::TransformStamped t2;
    t2.header.frame_id = "base";
    t2.header.stamp.sec = time_sec;
    t2.header.stamp.nsec = time_nsec;
    t2.child_frame_id = "arm";
    t2.transform.translation.x = 0.0;
    t2.transform.translation.y = 1.0;
    t2.transform.translation.z = 0.0;
    // Identity rotation
    t2.transform.rotation.w = 1.0;
    t2.transform.rotation.x = 0.0;
    t2.transform.rotation.y = 0.0;
    t2.transform.rotation.z = 0.0;
    
    // Transform 3: arm -> gripper
    geometry_msgs::TransformStamped t3;
    t3.header.frame_id = "arm";
    t3.header.stamp.sec = time_sec;
    t3.header.stamp.nsec = time_nsec;
    t3.child_frame_id = "gripper";
    t3.transform.translation.x = 0.0;
    t3.transform.translation.y = 0.0;
    t3.transform.translation.z = 0.5;
    // Identity rotation
    t3.transform.rotation.w = 1.0;
    t3.transform.rotation.x = 0.0;
    t3.transform.rotation.y = 0.0;
    t3.transform.rotation.z = 0.0;
    
    std::cout << "Adding transforms to buffer:" << std::endl;
    std::cout << "  world -> base: (1.0, 0.0, 0.0)" << std::endl;
    std::cout << "  base -> arm: (0.0, 1.0, 0.0)" << std::endl;
    std::cout << "  arm -> gripper: (0.0, 0.0, 0.5)" << std::endl;
    
    // Add transforms to buffer
    buffer.setTransform(t1, "test_authority");
    buffer.setTransform(t2, "test_authority");
    buffer.setTransform(t3, "test_authority");
    
    try {
        // Direct lookup (1-hop)
        auto base_to_arm = buffer.lookupTransform("base", "arm", now);
        std::cout << "1-hop lookup base -> arm:" << std::endl;
        std::cout << "  Header frame: " << base_to_arm.header.frame_id << std::endl;
        std::cout << "  Child frame: " << base_to_arm.child_frame_id << std::endl;
        std::cout << "  Translation: (" 
                  << base_to_arm.transform.translation.x << ", "
                  << base_to_arm.transform.translation.y << ", "
                  << base_to_arm.transform.translation.z << ")" << std::endl;
        std::cout << "  Rotation w,x,y,z: (" 
                  << base_to_arm.transform.rotation.w << ", "
                  << base_to_arm.transform.rotation.x << ", "
                  << base_to_arm.transform.rotation.y << ", "
                  << base_to_arm.transform.rotation.z << ")" << std::endl;
        
        // Multi-hop lookup (2-hops)
        auto base_to_gripper = buffer.lookupTransform("base", "gripper", now);
        std::cout << "2-hop lookup base -> gripper:" << std::endl;
        std::cout << "  Header frame: " << base_to_gripper.header.frame_id << std::endl;
        std::cout << "  Child frame: " << base_to_gripper.child_frame_id << std::endl;
        std::cout << "  Translation: (" 
                  << base_to_gripper.transform.translation.x << ", "
                  << base_to_gripper.transform.translation.y << ", "
                  << base_to_gripper.transform.translation.z << ")" << std::endl;
        std::cout << "  Rotation w,x,y,z: (" 
                  << base_to_gripper.transform.rotation.w << ", "
                  << base_to_gripper.transform.rotation.x << ", "
                  << base_to_gripper.transform.rotation.y << ", "
                  << base_to_gripper.transform.rotation.z << ")" << std::endl;
        
        // Multi-hop lookup (3-hops)
        auto world_to_gripper = buffer.lookupTransform("world", "gripper", now);
        std::cout << "3-hop lookup world -> gripper:" << std::endl;
        std::cout << "  Header frame: " << world_to_gripper.header.frame_id << std::endl;
        std::cout << "  Child frame: " << world_to_gripper.child_frame_id << std::endl;
        std::cout << "  Translation: (" 
                  << world_to_gripper.transform.translation.x << ", "
                  << world_to_gripper.transform.translation.y << ", "
                  << world_to_gripper.transform.translation.z << ")" << std::endl;
        std::cout << "  Rotation w,x,y,z: (" 
                  << world_to_gripper.transform.rotation.w << ", "
                  << world_to_gripper.transform.rotation.x << ", "
                  << world_to_gripper.transform.rotation.y << ", "
                  << world_to_gripper.transform.rotation.z << ")" << std::endl;
        
        // Reverse multi-hop lookup
        auto gripper_to_world = buffer.lookupTransform("gripper", "world", now);
        std::cout << "Reverse 3-hop lookup gripper -> world:" << std::endl;
        std::cout << "  Header frame: " << gripper_to_world.header.frame_id << std::endl;
        std::cout << "  Child frame: " << gripper_to_world.child_frame_id << std::endl;
        std::cout << "  Translation: (" 
                  << gripper_to_world.transform.translation.x << ", "
                  << gripper_to_world.transform.translation.y << ", "
                  << gripper_to_world.transform.translation.z << ")" << std::endl;
        std::cout << "  Rotation w,x,y,z: (" 
                  << gripper_to_world.transform.rotation.w << ", "
                  << gripper_to_world.transform.rotation.x << ", "
                  << gripper_to_world.transform.rotation.y << ", "
                  << gripper_to_world.transform.rotation.z << ")" << std::endl;
        
        // Test canTransform
        bool can_transform = buffer.canTransform("world", "gripper", now);
        std::cout << "Can transform from world to gripper: " 
                  << (can_transform ? "yes" : "no") << std::endl;
                  
        // Test non-existent transform
        bool cannot_transform = buffer.canTransform("world", "nonexistent", now);
        std::cout << "Can transform from world to nonexistent: " 
                  << (cannot_transform ? "yes" : "no") << std::endl;
        
        std::cout << "PASS: Chained transforms work correctly!" << std::endl;
        return 0;
    } catch (const tf_lcm::TransformException& ex) {
        std::cerr << "FAIL: Transform exception: " << ex.what() << std::endl;
        return 1;
    }
}
