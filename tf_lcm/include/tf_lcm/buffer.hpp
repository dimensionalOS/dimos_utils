#pragma once

#include <map>
#include <string>
#include <mutex>
#include <vector>
#include <unordered_map>
#include <memory>
#include <chrono>
#include <functional>
#include <lcm/lcm-cpp.hpp>
#include "geometry_msgs/TransformStamped.hpp"
#include "tf2_msgs/TFMessage.hpp"
#include "tf_lcm/transform_storage.hpp"
#include "tf_lcm/exceptions.hpp"

namespace tf_lcm {

/**
 * @brief Main TF Buffer class that stores and provides access to TF data
 * 
 * This class is the main interface for the TF library. It stores transforms
 * and provides methods to look up transforms between frames.
 */
class Buffer {
public:
    /**
     * @brief Construct a TF buffer
     * 
     * @param cache_time How long to keep transform data in the buffer (in seconds)
     */
    Buffer(double cache_time = 10.0);
    
    /**
     * @brief Destructor
     */
    ~Buffer();
    
    /**
     * @brief Set transform
     * 
     * @param transform The transform to store
     * @param authority The authority that provided this transform
     * @param is_static Whether this transform is static (doesn't change over time)
     * @return true if the transform was successfully added
     */
    bool setTransform(const geometry_msgs::TransformStamped& transform, const std::string& authority, bool is_static = false);
    
    /**
     * @brief Set transforms
     * 
     * @param transforms The transforms to store
     * @param authority The authority that provided these transforms
     * @param is_static Whether these transforms are static
     * @return true if the transforms were successfully added
     */
    bool setTransforms(const std::vector<geometry_msgs::TransformStamped>& transforms, const std::string& authority, bool is_static = false);
    
    /**
     * @brief Look up a transform
     * 
     * @param target_frame The target frame
     * @param source_frame The source frame
     * @param time The time at which to look up the transform
     * @param timeout How long to wait for the transform to be available
     * @return The transform from source_frame to target_frame
     * @throw tf_lcm::LookupException if the transform is not available
     */
    geometry_msgs::TransformStamped lookupTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        const std::chrono::system_clock::time_point& time,
        const std::chrono::duration<double>& timeout = std::chrono::duration<double>(0.0));
    
    /**
     * @brief Look up a transform with a fixed frame
     * 
     * @param target_frame The target frame
     * @param target_time The time to transform to
     * @param source_frame The source frame
     * @param source_time The time to transform from
     * @param fixed_frame The fixed frame to use
     * @param timeout How long to wait for the transform to be available
     * @return The transform from source_frame at source_time to target_frame at target_time
     * @throw tf_lcm::LookupException if the transform is not available
     */
    geometry_msgs::TransformStamped lookupTransform(
        const std::string& target_frame,
        const std::chrono::system_clock::time_point& target_time,
        const std::string& source_frame,
        const std::chrono::system_clock::time_point& source_time,
        const std::string& fixed_frame,
        const std::chrono::duration<double>& timeout = std::chrono::duration<double>(0.0));
    
    /**
     * @brief Check if a transform is possible
     * 
     * @param target_frame The target frame
     * @param source_frame The source frame
     * @param time The time at which to check
     * @param error_msg Error message if transform is not possible
     * @return true if the transform is possible
     */
    bool canTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        const std::chrono::system_clock::time_point& time,
        std::string* error_msg = nullptr);
    
    /**
     * @brief Check if a transform with a fixed frame is possible
     * 
     * @param target_frame The target frame
     * @param target_time The time to transform to
     * @param source_frame The source frame
     * @param source_time The time to transform from
     * @param fixed_frame The fixed frame to use
     * @param error_msg Error message if transform is not possible
     * @return true if the transform is possible
     */
    bool canTransform(
        const std::string& target_frame,
        const std::chrono::system_clock::time_point& target_time,
        const std::string& source_frame,
        const std::chrono::system_clock::time_point& source_time,
        const std::string& fixed_frame,
        std::string* error_msg = nullptr);
    
    /**
     * @brief Get all frame IDs known to the buffer
     * 
     * @return Vector of frame IDs
     */
    std::vector<std::string> getAllFrameNames() const;
    
    /**
     * @brief Clear all transforms from the buffer
     */
    void clear();

private:
    std::unordered_map<std::string, std::unordered_map<std::string, std::vector<TransformStorage>>> buffer_;
    std::unordered_map<std::string, std::unordered_map<std::string, std::vector<TransformStorage>>> static_buffer_;
    double cache_time_;
    mutable std::mutex mutex_;
    
    // Helper methods
    bool _canTransformNoLock(
        const std::string& target_frame,
        const std::string& source_frame,
        const std::chrono::system_clock::time_point& time,
        std::string* error_msg = nullptr);
    
    geometry_msgs::TransformStamped _lookupTransformNoLock(
        const std::string& target_frame,
        const std::string& source_frame,
        const std::chrono::system_clock::time_point& time);
    
    void _waitForTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        const std::chrono::system_clock::time_point& time,
        const std::chrono::duration<double>& timeout);
    
    /**
     * @brief Look up a direct transform between two frames
     * 
     * This helper method is used to find a direct transform without graph traversal
     * 
     * @param source_frame Source frame
     * @param target_frame Target frame
     * @param time Time point for the transform
     * @return The direct transform
     * @throws LookupException if no direct transform exists
     */
    geometry_msgs::TransformStamped _lookupDirectTransform(
        const std::string& source_frame,
        const std::string& target_frame,
        const std::chrono::system_clock::time_point& time);
    
    /**
     * @brief Invert a transform
     * 
     * @param transform Transform to invert
     * @return The inverted transform
     */
    geometry_msgs::TransformStamped _inverseTransform(
        const geometry_msgs::TransformStamped& transform);
    
    /**
     * @brief Compose two transforms
     * 
     * @param t1 First transform
     * @param t2 Second transform
     * @return The composed transform
     */
    geometry_msgs::TransformStamped _composeTransforms(
        const geometry_msgs::TransformStamped& t1,
        const geometry_msgs::TransformStamped& t2);
};

} // namespace tf_lcm
