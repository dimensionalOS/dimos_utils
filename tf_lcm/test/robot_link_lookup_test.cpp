#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <atomic>
#include <signal.h>
#include "tf_lcm/buffer.hpp"
#include "tf_lcm/listener.hpp"

// Signal handling for clean shutdown
std::atomic<bool> keep_running(true);
void signal_handler(int signal) {
    std::cout << "Received signal " << signal << ", shutting down..." << std::endl;
    keep_running = false;
}

int main(int argc, char** argv) {
    // Setup signal handling for clean shutdown
    signal(SIGINT, signal_handler);
    
    std::cout << "=== Robot Link Transform Lookup Test ===" << std::endl;
    std::cout << "Listening for transforms between 'world' and 'link6'..." << std::endl;
    std::cout << "Press Ctrl+C to exit" << std::endl;
    
    // Define the source and target frames
    const std::string target_frame = "world";
    const std::string source_frame = "link6";
    
    // Create LCM and buffer with 30 seconds cache time (longer for debugging)
    auto lcm = std::make_shared<lcm::LCM>();
    if (!lcm->good()) {
        std::cerr << "ERROR: Failed to initialize LCM!" << std::endl;
        return 1;
    }
    
    tf_lcm::Buffer buffer(30.0);  // 30 seconds buffer
    
    // Create transform listener
    tf_lcm::TransformListener listener(lcm, buffer);
    
    // Helper function to print transform information in a readable format
    auto print_transform = [](const geometry_msgs::TransformStamped& transform) {
        std::cout << "Transform found!" << std::endl;
        std::cout << "  Header frame: " << transform.header.frame_id << std::endl;
        std::cout << "  Child frame: " << transform.child_frame_id << std::endl;
        std::cout << "  Timestamp: " 
                  << transform.header.stamp.sec << "."
                  << std::setfill('0') << std::setw(9) << transform.header.stamp.nsec
                  << std::endl;
        std::cout << "  Translation: ("
                  << transform.transform.translation.x << ", "
                  << transform.transform.translation.y << ", "
                  << transform.transform.translation.z << ")" << std::endl;
        std::cout << "  Rotation (quaternion): ("
                  << transform.transform.rotation.w << ", "
                  << transform.transform.rotation.x << ", "
                  << transform.transform.rotation.y << ", "
                  << transform.transform.rotation.z << ")" << std::endl;
    };
    
    // Main lookup loop
    int lookup_attempts = 0;
    bool found_transform = false;
    
    while (keep_running) {
        try {
            // Wait a short time to process incoming messages
            lcm->handleTimeout(100);  // 100ms timeout
            
            // Try to look up the transform
            // First check if we can transform
            auto now = std::chrono::system_clock::now();
            bool can_transform = buffer.canTransform(target_frame, source_frame, now);
            
            if (can_transform) {
                // Look up the transform
                auto transform = buffer.lookupTransform(target_frame, source_frame, now);
                print_transform(transform);
                found_transform = true;
                
                // Get all frame names to help with debugging
                auto frames = buffer.getAllFrameNames();
                std::cout << "All frames in buffer (" << frames.size() << " total):" << std::endl;
                for (const auto& frame : frames) {
                    std::cout << "  " << frame << std::endl;
                }
                
                // Test passed, we can exit
                std::cout << "\n✅ SUCCESS: Found transform between '" << target_frame 
                          << "' and '" << source_frame << "'!" << std::endl;
                break;
            } else {
                // Increment counter and report status every few attempts
                lookup_attempts++;
                if (lookup_attempts % 10 == 0) {
                    std::cout << "Waiting for transform between '" << target_frame 
                              << "' and '" << source_frame << "' (" 
                              << lookup_attempts << " attempts)..." << std::endl;
                    
                    // Print out all frames we have received
                    auto frames = buffer.getAllFrameNames();
                    std::cout << "Frames received so far (" << frames.size() << " total):" << std::endl;
                    for (const auto& frame : frames) {
                        std::cout << "  " << frame << std::endl;
                    }
                }
                
                // Sleep briefly to avoid spinning too fast
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            
            // Timeout after many attempts (60 seconds)
            if (lookup_attempts > 120) {
                std::cerr << "\n❌ ERROR: Failed to find transform between '" << target_frame 
                          << "' and '" << source_frame << "' after " 
                          << lookup_attempts << " attempts!" << std::endl;
                break;
            }
        } catch (const tf_lcm::TransformException& ex) {
            lookup_attempts++;
            if (lookup_attempts % 10 == 0) {
                std::cerr << "Exception: " << ex.what() << std::endl;
                
                // Print out all frames we have received to help with debugging
                auto frames = buffer.getAllFrameNames();
                std::cout << "Frames received so far (" << frames.size() << " total):" << std::endl;
                for (const auto& frame : frames) {
                    std::cout << "  " << frame << std::endl;
                }
            }
            
            // Sleep to avoid spinning too fast
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // Timeout after many attempts (60 seconds)
            if (lookup_attempts > 120) {
                std::cerr << "\n❌ ERROR: Failed to find transform between '" << target_frame 
                          << "' and '" << source_frame << "' after " 
                          << lookup_attempts << " attempts!" << std::endl;
                break;
            }
        }
    }
    
    // Final result
    if (!found_transform) {
        std::cerr << "\n❌ Test FAILED: Could not find transform between '" 
                  << target_frame << "' and '" << source_frame << "'." << std::endl;
        return 1;
    }
    
    return 0;
}
