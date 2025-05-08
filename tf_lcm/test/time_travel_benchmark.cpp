#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <vector>
#include <random>
#include <algorithm>
#include <numeric>
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

// Class to handle benchmark results
class BenchmarkResults {
public:
    void addResult(const std::string& name, const std::chrono::duration<double>& duration, int lookups) {
        double time_sec = duration.count();
        double lookups_per_sec = lookups / time_sec;
        
        results_.push_back({name, time_sec, lookups_per_sec});
    }
    
    void printResults() const {
        // Find the longest name for formatting
        size_t max_name_length = 0;
        for (const auto& result : results_) {
            max_name_length = std::max(max_name_length, result.name.length());
        }
        
        // Print header
        std::cout << "\n=== Benchmark Results ===" << std::endl;
        std::cout << std::left << std::setw(max_name_length + 2) << "Test Case" 
                  << std::right << std::setw(15) << "Time (s)" 
                  << std::setw(20) << "Lookups/Second" << std::endl;
        
        std::cout << std::string(max_name_length + 2 + 15 + 20, '-') << std::endl;
        
        // Print each result
        for (const auto& result : results_) {
            std::cout << std::left << std::setw(max_name_length + 2) << result.name 
                      << std::right << std::setw(15) << std::fixed << std::setprecision(6) << result.time_sec 
                      << std::setw(20) << std::fixed << std::setprecision(2) << result.lookups_per_sec << std::endl;
        }
    }
    
private:
    struct Result {
        std::string name;
        double time_sec;
        double lookups_per_sec;
    };
    
    std::vector<Result> results_;
};

// Helper function to format a timestamp
std::string formatTimestamp(int64_t sec, int32_t nsec) {
    std::stringstream ss;
    ss << sec << "." << std::setfill('0') << std::setw(9) << nsec;
    return ss.str();
}

// Benchmark function for exact timestamp lookups
void benchmarkExactTimestamp(tf_lcm::Buffer& buffer, 
                           const std::string& target_frame,
                           const std::string& source_frame,
                           const std::vector<std::chrono::system_clock::time_point>& timestamps,
                           BenchmarkResults& results) {
    const int num_lookups = timestamps.size();
    
    // Warm-up run (not timed)
    for (int i = 0; i < std::min(10, num_lookups); ++i) {
        try {
            buffer.lookupTransform(target_frame, source_frame, timestamps[i]);
        } catch (const tf_lcm::TransformException&) {
            // Ignore exceptions during warm-up
        }
    }
    
    // Timed run
    auto start_time = std::chrono::high_resolution_clock::now();
    
    int successful_lookups = 0;
    for (const auto& timestamp : timestamps) {
        try {
            auto transform = buffer.lookupTransform(target_frame, source_frame, timestamp);
            successful_lookups++;
        } catch (const tf_lcm::TransformException& ex) {
            // Failed lookup
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = end_time - start_time;
    
    // Add results
    std::stringstream ss;
    ss << "Exact timestamp lookups (" << successful_lookups << "/" << num_lookups << " successful)";
    results.addResult(ss.str(), duration, num_lookups);
}

// Benchmark function for interpolated timestamp lookups
void benchmarkInterpolatedTimestamps(tf_lcm::Buffer& buffer,
                                  const std::string& target_frame,
                                  const std::string& source_frame,
                                  const std::vector<std::chrono::system_clock::time_point>& known_timestamps,
                                  int num_lookups,
                                  BenchmarkResults& results) {
    if (known_timestamps.size() < 2) {
        std::cerr << "Need at least 2 known timestamps for interpolation benchmark" << std::endl;
        return;
    }
    
    // Generate random timestamps between known ones
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Use the same time point type as known_timestamps
    std::vector<std::chrono::system_clock::time_point> interpolated_timestamps;
    interpolated_timestamps.reserve(num_lookups);
    
    for (int i = 0; i < num_lookups; ++i) {
        // Pick two adjacent timestamps
        size_t idx = gen() % (known_timestamps.size() - 1);
        auto start = known_timestamps[idx];
        auto end = known_timestamps[idx + 1];
        
        // Calculate a random point between them
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        double alpha = dist(gen);
        
        // Convert to microseconds for consistency with system_clock precision
        auto start_us = std::chrono::time_point_cast<std::chrono::microseconds>(start).time_since_epoch().count();
        auto end_us = std::chrono::time_point_cast<std::chrono::microseconds>(end).time_since_epoch().count();
        auto interp_us = start_us + static_cast<long long>((end_us - start_us) * alpha);
        
        // Create interpolated time point using microsecond precision
        auto interpolated_time = std::chrono::system_clock::time_point() + std::chrono::microseconds(interp_us);
        interpolated_timestamps.push_back(interpolated_time);
    }
    
    // Warm-up run (not timed)
    for (int i = 0; i < std::min(10, num_lookups); ++i) {
        try {
            buffer.lookupTransform(target_frame, source_frame, interpolated_timestamps[i]);
        } catch (const tf_lcm::TransformException&) {
            // Ignore exceptions during warm-up
        }
    }
    
    // Timed run
    auto start_time = std::chrono::high_resolution_clock::now();
    
    int successful_lookups = 0;
    for (const auto& timestamp : interpolated_timestamps) {
        try {
            auto transform = buffer.lookupTransform(target_frame, source_frame, timestamp);
            successful_lookups++;
        } catch (const tf_lcm::TransformException& ex) {
            // Failed lookup
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = end_time - start_time;
    
    // Add results
    std::stringstream ss;
    ss << "Interpolated timestamp lookups (" << successful_lookups << "/" << num_lookups << " successful)";
    results.addResult(ss.str(), duration, num_lookups);
}

// Benchmark function for extrapolated timestamp lookups
void benchmarkExtrapolatedTimestamps(tf_lcm::Buffer& buffer,
                                  const std::string& target_frame,
                                  const std::string& source_frame,
                                  const std::vector<std::chrono::system_clock::time_point>& known_timestamps,
                                  int num_lookups,
                                  double max_extrapolation_sec,
                                  BenchmarkResults& results) {
    if (known_timestamps.empty()) {
        std::cerr << "Need at least 1 known timestamp for extrapolation benchmark" << std::endl;
        return;
    }
    
    // Find min and max timestamps
    auto min_timestamp = *std::min_element(known_timestamps.begin(), known_timestamps.end());
    auto max_timestamp = *std::max_element(known_timestamps.begin(), known_timestamps.end());
    
    // Generate extrapolated timestamps (half before min, half after max)
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, max_extrapolation_sec);
    
    std::vector<std::chrono::system_clock::time_point> extrapolated_timestamps;
    extrapolated_timestamps.reserve(num_lookups);
    
    for (int i = 0; i < num_lookups / 2; ++i) {
        // Before min
        double seconds_before = dist(gen);
        // Use microseconds for consistent precision with system_clock
        auto extrapolated_time = min_timestamp - std::chrono::microseconds(static_cast<int64_t>(seconds_before * 1000000));
        extrapolated_timestamps.push_back(extrapolated_time);
        
        // After max
        double seconds_after = dist(gen);
        extrapolated_time = max_timestamp + std::chrono::microseconds(static_cast<int64_t>(seconds_after * 1000000));
        extrapolated_timestamps.push_back(extrapolated_time);
    }
    
    // If odd number, add one more
    if (num_lookups % 2 != 0) {
        double seconds_after = dist(gen);
        auto extrapolated_time = max_timestamp + std::chrono::microseconds(static_cast<int64_t>(seconds_after * 1000000));
        extrapolated_timestamps.push_back(extrapolated_time);
    }
    
    // Warm-up run (not timed)
    for (int i = 0; i < std::min(10, num_lookups); ++i) {
        try {
            buffer.lookupTransform(target_frame, source_frame, extrapolated_timestamps[i]);
        } catch (const tf_lcm::TransformException&) {
            // Ignore exceptions during warm-up
        }
    }
    
    // Timed run
    auto start_time = std::chrono::high_resolution_clock::now();
    
    int successful_lookups = 0;
    for (const auto& timestamp : extrapolated_timestamps) {
        try {
            auto transform = buffer.lookupTransform(target_frame, source_frame, timestamp);
            successful_lookups++;
        } catch (const tf_lcm::TransformException& ex) {
            // Failed lookup
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = end_time - start_time;
    
    // Add results
    std::stringstream ss;
    ss << "Extrapolated timestamp lookups (" << successful_lookups << "/" << num_lookups << " successful)";
    results.addResult(ss.str(), duration, num_lookups);
}

// Benchmark cross-time transform lookups (advanced time travel)
void benchmarkCrossTimeTransforms(tf_lcm::Buffer& buffer,
                               const std::string& target_frame,
                               const std::string& source_frame,
                               const std::string& fixed_frame,
                               const std::vector<std::chrono::system_clock::time_point>& known_timestamps,
                               int num_lookups,
                               BenchmarkResults& results) {
    if (known_timestamps.size() < 2) {
        std::cerr << "Need at least 2 known timestamps for cross-time transform benchmark" << std::endl;
        return;
    }
    
    // Generate pairs of timestamps from known timestamps
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> idx_dist(0, known_timestamps.size() - 1);
    
    std::vector<std::pair<std::chrono::system_clock::time_point, std::chrono::system_clock::time_point>> timestamp_pairs;
    timestamp_pairs.reserve(num_lookups);
    
    for (int i = 0; i < num_lookups; ++i) {
        size_t source_idx = idx_dist(gen);
        size_t target_idx = idx_dist(gen);
        
        // Ensure we pick different timestamps
        while (source_idx == target_idx && known_timestamps.size() > 1) {
            target_idx = idx_dist(gen);
        }
        
        timestamp_pairs.push_back({known_timestamps[source_idx], known_timestamps[target_idx]});
    }
    
    // Warm-up run (not timed)
    for (int i = 0; i < std::min(10, num_lookups); ++i) {
        try {
            const auto& pair = timestamp_pairs[i];
            const auto& target_time = pair.first;
            const auto& source_time = pair.second;
            buffer.lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame);
        } catch (const tf_lcm::TransformException&) {
            // Ignore exceptions during warm-up
        }
    }
    
    // Timed run
    auto start_time = std::chrono::high_resolution_clock::now();
    
    int successful_lookups = 0;
    for (const auto& pair : timestamp_pairs) {
        try {
            const auto& target_time = pair.first;
            const auto& source_time = pair.second;
            auto transform = buffer.lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame);
            successful_lookups++;
        } catch (const tf_lcm::TransformException& ex) {
            // Failed lookup
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = end_time - start_time;
    
    // Add results
    std::stringstream ss;
    ss << "Cross-time transform lookups (" << successful_lookups << "/" << num_lookups << " successful)";
    results.addResult(ss.str(), duration, num_lookups);
}

int main(int argc, char** argv) {
    // Setup signal handling for clean shutdown
    signal(SIGINT, signal_handler);
    
    // Parse command line arguments
    int num_lookups = 10000;  // Default number of lookups
    double max_wait_time = 60.0;  // Default max wait time in seconds
    
    if (argc > 1) {
        try {
            num_lookups = std::stoi(argv[1]);
            if (num_lookups <= 0) {
                std::cerr << "Number of lookups must be positive" << std::endl;
                return 1;
            }
        } catch (std::exception& e) {
            std::cerr << "Invalid argument for number of lookups: " << argv[1] << std::endl;
            std::cerr << "Usage: " << argv[0] << " [num_lookups] [max_wait_time]" << std::endl;
            return 1;
        }
    }
    
    if (argc > 2) {
        try {
            max_wait_time = std::stod(argv[2]);
            if (max_wait_time <= 0) {
                std::cerr << "Max wait time must be positive" << std::endl;
                return 1;
            }
        } catch (std::exception& e) {
            std::cerr << "Invalid argument for max wait time: " << argv[2] << std::endl;
            std::cerr << "Usage: " << argv[0] << " [num_lookups] [max_wait_time]" << std::endl;
            return 1;
        }
    }
    
    std::cout << "=== Time Travel Benchmark Test ===" << std::endl;
    std::cout << "Will perform " << num_lookups << " lookups per test case" << std::endl;
    
    // Define the frames
    const std::string source_frame = "link6";
    const std::string target_frame = "world";
    const std::string fixed_frame = "world";  // Common reference frame for cross-time transforms
    
    // Create LCM and buffer with 30 seconds cache time
    auto lcm = std::make_shared<lcm::LCM>();
    if (!lcm->good()) {
        std::cerr << "ERROR: Failed to initialize LCM!" << std::endl;
        return 1;
    }
    
    tf_lcm::Buffer buffer(30.0);  // 30 seconds buffer
    
    // Create transform listener
    tf_lcm::TransformListener listener(lcm, buffer);
    
    std::cout << "Waiting for transforms between '" << source_frame << "' and '" 
              << target_frame << "'..." << std::endl;
    
    // Wait for transforms to become available
    bool found_transform = false;
    std::vector<std::chrono::system_clock::time_point> known_timestamps;
    auto start_wait = std::chrono::system_clock::now();
    
    while (keep_running && !found_transform && 
           std::chrono::duration<double>(std::chrono::system_clock::now() - start_wait).count() < max_wait_time) {
        try {
            // Process LCM messages
            lcm->handleTimeout(100);
            
            // Try to get the transform
            auto now = std::chrono::system_clock::now();
            if (buffer.canTransform(target_frame, source_frame, now)) {
                auto transform = buffer.lookupTransform(target_frame, source_frame, now);
                known_timestamps.push_back(now);
                found_transform = true;
                
                std::cout << "Transform found at " 
                          << formatTimestamp(transform.header.stamp.sec, transform.header.stamp.nsec) << std::endl;
            }
        } catch (const tf_lcm::TransformException& ex) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    
    if (!found_transform) {
        std::cerr << "Could not find transform between '" << source_frame 
                  << "' and '" << target_frame << "' after " 
                  << max_wait_time << " seconds. Exiting." << std::endl;
        return 1;
    }
    
    // Collect more timestamps over a period
    std::cout << "Collecting transform timestamps over a period..." << std::endl;
    
    int num_timestamps_to_collect = 50;
    int collected = 1;  // We already have one
    auto collection_start = std::chrono::system_clock::now();
    
    while (keep_running && collected < num_timestamps_to_collect && 
           std::chrono::duration<double>(std::chrono::system_clock::now() - collection_start).count() < max_wait_time) {
        try {
            // Process LCM messages
            lcm->handleTimeout(100);
            
            // Try to get the transform at current time
            auto now = std::chrono::system_clock::now();
            if (buffer.canTransform(target_frame, source_frame, now)) {
                auto transform = buffer.lookupTransform(target_frame, source_frame, now);
                
                // Add this timestamp to our collection
                known_timestamps.push_back(now);
                collected++;
                
                if (collected % 10 == 0) {
                    std::cout << "Collected " << collected << " timestamps..." << std::endl;
                }
                
                // Don't collect too quickly
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        } catch (const tf_lcm::TransformException& ex) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    std::cout << "Collected " << known_timestamps.size() << " timestamps for benchmark" << std::endl;
    
    if (known_timestamps.size() < 2) {
        std::cerr << "Not enough timestamps collected for benchmark. Need at least 2." << std::endl;
        return 1;
    }
    
    // Sort timestamps
    std::sort(known_timestamps.begin(), known_timestamps.end());
    
    // Output timestamp range for debugging
    auto first_ts = known_timestamps.front();
    auto last_ts = known_timestamps.back();
    auto first_sec = std::chrono::time_point_cast<std::chrono::seconds>(first_ts).time_since_epoch().count();
    auto first_nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(first_ts.time_since_epoch() % std::chrono::seconds(1)).count();
    auto last_sec = std::chrono::time_point_cast<std::chrono::seconds>(last_ts).time_since_epoch().count();
    auto last_nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(last_ts.time_since_epoch() % std::chrono::seconds(1)).count();
    
    std::cout << "Timestamp range: " 
              << formatTimestamp(first_sec, first_nsec) << " to " 
              << formatTimestamp(last_sec, last_nsec) << std::endl;
    
    // Create results tracker
    BenchmarkResults results;
    
    // Run benchmarks
    std::cout << "\nRunning benchmark tests..." << std::endl;
    
    std::cout << "1. Exact timestamp lookups..." << std::endl;
    benchmarkExactTimestamp(buffer, target_frame, source_frame, known_timestamps, results);
    
    std::cout << "2. Interpolated timestamp lookups..." << std::endl;
    benchmarkInterpolatedTimestamps(buffer, target_frame, source_frame, known_timestamps, num_lookups, results);
    
    std::cout << "3. Extrapolated timestamp lookups (up to 2 seconds)..." << std::endl;
    benchmarkExtrapolatedTimestamps(buffer, target_frame, source_frame, known_timestamps, num_lookups, 2.0, results);
    
    std::cout << "4. Cross-time transform lookups..." << std::endl;
    benchmarkCrossTimeTransforms(buffer, target_frame, source_frame, fixed_frame, known_timestamps, num_lookups, results);
    
    // Print results
    results.printResults();
    
    std::cout << "\nTime travel benchmark completed successfully!" << std::endl;
    
    return 0;
}
