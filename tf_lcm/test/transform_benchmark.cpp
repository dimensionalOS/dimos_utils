#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <vector>
#include <random>
#include <string>
#include <signal.h>
#include <atomic>
#include "tf_lcm/buffer.hpp"
#include "tf_lcm/listener.hpp"

// Signal handling for clean shutdown
std::atomic<bool> keep_running(true);
void signal_handler(int signal) {
    std::cout << "Received signal " << signal << ", shutting down..." << std::endl;
    keep_running = false;
}

// Structure to represent a 3D point
struct Point3D {
    double x, y, z;
};

// Generate n random points with coordinates in range [min, max]
std::vector<Point3D> generateRandomPoints(int n, double min = -10.0, double max = 10.0) {
    std::vector<Point3D> points;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(min, max);
    
    points.reserve(n);
    for (int i = 0; i < n; i++) {
        points.push_back({dist(gen), dist(gen), dist(gen)});
    }
    
    return points;
}

// Transform a point using a transform
Point3D transformPoint(const geometry_msgs::TransformStamped& transform, const Point3D& point) {
    // Extract rotation components
    double qw = transform.transform.rotation.w;
    double qx = transform.transform.rotation.x;
    double qy = transform.transform.rotation.y;
    double qz = transform.transform.rotation.z;
    
    // Extract translation components
    double tx = transform.transform.translation.x;
    double ty = transform.transform.translation.y;
    double tz = transform.transform.translation.z;
    
    // Apply rotation (quaternion rotation of a point)
    double px = point.x;
    double py = point.y;
    double pz = point.z;
    
    // Apply the rotation using quaternion-vector multiplication
    // Formula: p' = q * p * q^-1 where p = (0, px, py, pz)
    // For efficiency, we can simplify this calculation
    
    // First compute q * p
    double tmp_w = -qx * px - qy * py - qz * pz;
    double tmp_x = qw * px + qy * pz - qz * py;
    double tmp_y = qw * py + qz * px - qx * pz;
    double tmp_z = qw * pz + qx * py - qy * px;
    
    // Then compute (q * p) * q^-1 (note: q^-1 = (qw, -qx, -qy, -qz) for unit quaternions)
    double rx = tmp_x * qw + tmp_w * (-qx) + tmp_y * (-qz) - tmp_z * (-qy);
    double ry = tmp_y * qw + tmp_w * (-qy) + tmp_z * (-qx) - tmp_x * (-qz);
    double rz = tmp_z * qw + tmp_w * (-qz) + tmp_x * (-qy) - tmp_y * (-qx);
    
    // Apply translation
    rx += tx;
    ry += ty;
    rz += tz;
    
    return {rx, ry, rz};
}

int main(int argc, char** argv) {
    // Setup signal handling for clean shutdown
    signal(SIGINT, signal_handler);
    
    // Parse command line arguments
    int num_points = 1000000;  // Default 1 million points
    
    if (argc > 1) {
        try {
            num_points = std::stoi(argv[1]);
            if (num_points <= 0) {
                std::cerr << "Number of points must be positive" << std::endl;
                return 1;
            }
        } catch (std::exception& e) {
            std::cerr << "Invalid argument for number of points: " << argv[1] << std::endl;
            std::cerr << "Usage: " << argv[0] << " [num_points]" << std::endl;
            return 1;
        }
    }
    
    std::cout << "=== Transform Benchmark Test ===" << std::endl;
    std::cout << "Will transform " << num_points << " random points from link6 to world frame" << std::endl;
    
    // Define the source and target frames
    const std::string source_frame = "link6";
    const std::string target_frame = "world";
    
    // Create LCM and buffer with 30 seconds cache time
    auto lcm = std::make_shared<lcm::LCM>();
    if (!lcm->good()) {
        std::cerr << "ERROR: Failed to initialize LCM!" << std::endl;
        return 1;
    }
    
    tf_lcm::Buffer buffer(30.0);  // 30 seconds buffer
    
    // Create transform listener
    tf_lcm::TransformListener listener(lcm, buffer);
    
    std::cout << "Waiting for transform between '" << source_frame << "' and '" 
              << target_frame << "'..." << std::endl;
    
    // Wait for the transform to become available
    auto now = std::chrono::system_clock::now();
    bool found = false;
    int attempts = 0;
    geometry_msgs::TransformStamped transform;
    
    while (keep_running && !found && attempts < 60) {
        try {
            // Process LCM messages
            lcm->handleTimeout(100);
            
            // Try to get the transform
            now = std::chrono::system_clock::now();
            if (buffer.canTransform(target_frame, source_frame, now)) {
                transform = buffer.lookupTransform(target_frame, source_frame, now);
                found = true;
                std::cout << "Transform found!" << std::endl;
                std::cout << "  Translation: (" 
                          << transform.transform.translation.x << ", "
                          << transform.transform.translation.y << ", "
                          << transform.transform.translation.z << ")" << std::endl;
            } else {
                attempts++;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        } catch (const tf_lcm::TransformException& ex) {
            attempts++;
            if (attempts % 5 == 0) {
                std::cerr << "Exception: " << ex.what() << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    
    if (!found) {
        std::cerr << "Could not find transform between '" << source_frame 
                  << "' and '" << target_frame << "' after " 
                  << attempts << " attempts. Exiting." << std::endl;
        return 1;
    }
    
    // Generate random points
    std::cout << "Generating " << num_points << " random points..." << std::endl;
    auto start_gen = std::chrono::high_resolution_clock::now();
    auto points = generateRandomPoints(num_points);
    auto end_gen = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> gen_time = end_gen - start_gen;
    std::cout << "Generated " << points.size() << " points in " 
              << gen_time.count() << " seconds" << std::endl;
    
    // Transform all points
    std::cout << "Transforming points..." << std::endl;
    std::vector<Point3D> transformed_points;
    transformed_points.reserve(points.size());
    
    // Start timing
    auto start_transform = std::chrono::high_resolution_clock::now();
    
    // Transform all points
    for (const auto& point : points) {
        transformed_points.push_back(transformPoint(transform, point));
    }
    
    // End timing
    auto end_transform = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> transform_time = end_transform - start_transform;
    
    // Calculate performance metrics
    double points_per_second = num_points / transform_time.count();
    
    // Print results
    std::cout << "\n=== Benchmark Results ===" << std::endl;
    std::cout << "Total points transformed: " << num_points << std::endl;
    std::cout << "Total transform time: " << transform_time.count() << " seconds" << std::endl;
    std::cout << "Performance: " << points_per_second << " points/second" << std::endl;
    
    // Print first and last transformed point as sanity check
    if (!transformed_points.empty()) {
        std::cout << "\nFirst transformed point: (" 
                  << transformed_points.front().x << ", "
                  << transformed_points.front().y << ", "
                  << transformed_points.front().z << ")" << std::endl;
        
        std::cout << "Last transformed point: (" 
                  << transformed_points.back().x << ", "
                  << transformed_points.back().y << ", "
                  << transformed_points.back().z << ")" << std::endl;
    }
    
    std::cout << "\nBenchmark completed successfully!" << std::endl;
    
    return 0;
}
