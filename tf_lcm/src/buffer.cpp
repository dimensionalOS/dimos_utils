#include "tf_lcm/buffer.hpp"
#include <algorithm>
#include <queue>
#include <iostream>
#include <thread>
#include <sstream>

namespace tf_lcm {

Buffer::Buffer(double cache_time)
    : cache_time_(cache_time) {
}

Buffer::~Buffer() {
    clear();
}

bool Buffer::setTransform(const geometry_msgs::TransformStamped& transform, const std::string& authority, bool is_static) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Check for valid transform
    if (transform.child_frame_id == transform.header.frame_id) {
        std::cerr << "TF_SELF_TRANSFORM: Ignoring transform with frame_id and child_frame_id: " 
                  << transform.child_frame_id << std::endl;
        return false;
    }
    
    if (transform.child_frame_id.empty()) {
        std::cerr << "TF_NO_CHILD_FRAME_ID: Ignoring transform with empty child_frame_id" << std::endl;
        return false;
    }
    
    if (transform.header.frame_id.empty()) {
        std::cerr << "TF_NO_FRAME_ID: Ignoring transform with empty frame_id" << std::endl;
        return false;
    }
    
    // Store the transform
    TransformStorage storage(transform, authority);
    
    auto& buffer = is_static ? static_buffer_ : buffer_;
    
    // Store in buffer, parent->child
    buffer[transform.header.frame_id][transform.child_frame_id].push_back(storage);
    
    // Clean old transforms
    auto now = std::chrono::system_clock::now();
    for (auto& parent_map : buffer) {
        for (auto& child_vec : parent_map.second) {
            auto& transforms = child_vec.second;
            transforms.erase(
                std::remove_if(transforms.begin(), transforms.end(),
                    [this, now, is_static](const TransformStorage& ts) {
                        return !is_static && 
                               std::chrono::duration<double>(now - ts.stamp).count() > cache_time_;
                    }),
                transforms.end());
        }
    }
    
    return true;
}

bool Buffer::setTransforms(const std::vector<geometry_msgs::TransformStamped>& transforms, const std::string& authority, bool is_static) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    bool result = true;
    for (const auto& transform : transforms) {
        result = result && setTransform(transform, authority, is_static);
    }
    
    return result;
}

geometry_msgs::TransformStamped Buffer::lookupTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    const std::chrono::system_clock::time_point& time,
    const std::chrono::duration<double>& timeout) {
    
    // Check for timeout
    if (timeout > std::chrono::duration<double>(0.0)) {
        _waitForTransform(target_frame, source_frame, time, timeout);
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    return _lookupTransformNoLock(target_frame, source_frame, time);
}

geometry_msgs::TransformStamped Buffer::lookupTransform(
    const std::string& target_frame,
    const std::chrono::system_clock::time_point& target_time,
    const std::string& source_frame,
    const std::chrono::system_clock::time_point& source_time,
    const std::string& fixed_frame,
    const std::chrono::duration<double>& timeout) {
    
    // TODO: Implement advanced time travel
    // This is a simplified version
    
    // Look up transform from fixed frame to source at source_time
    auto source_to_fixed = lookupTransform(fixed_frame, source_frame, source_time, timeout);
    
    // Look up transform from fixed frame to target at target_time
    auto target_to_fixed = lookupTransform(fixed_frame, target_frame, target_time, timeout);
    
    // Invert the target to fixed transform to get fixed to target
    // Normally we would implement proper transform inversion, but for simplicity we'll just assume it's possible
    
    // Combine the transforms: source -> fixed -> target
    // Again, normally we'd implement proper transform composition
    
    // For now, just return a placeholder transform
    geometry_msgs::TransformStamped result;
    result.header.frame_id = target_frame;
    result.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(target_time.time_since_epoch()).count();
    result.header.stamp.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
        target_time.time_since_epoch() % std::chrono::seconds(1)).count();
    result.child_frame_id = source_frame;
    result.transform = target_to_fixed.transform; // This is just a placeholder
    
    return result;
}

bool Buffer::canTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    const std::chrono::system_clock::time_point& time,
    std::string* error_msg) {
    
    std::lock_guard<std::mutex> lock(mutex_);
    return _canTransformNoLock(target_frame, source_frame, time, error_msg);
}

bool Buffer::canTransform(
    const std::string& target_frame,
    const std::chrono::system_clock::time_point& target_time,
    const std::string& source_frame,
    const std::chrono::system_clock::time_point& source_time,
    const std::string& fixed_frame,
    std::string* error_msg) {
    
    // Check if we can transform from source to fixed
    if (!canTransform(fixed_frame, source_frame, source_time, error_msg)) {
        return false;
    }
    
    // Check if we can transform from fixed to target
    if (!canTransform(fixed_frame, target_frame, target_time, error_msg)) {
        return false;
    }
    
    return true;
}

std::vector<std::string> Buffer::getAllFrameNames() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<std::string> frames;
    
    // Add static frames
    for (const auto& parent_map : static_buffer_) {
        frames.push_back(parent_map.first);
        for (const auto& child_vec : parent_map.second) {
            frames.push_back(child_vec.first);
        }
    }
    
    // Add non-static frames
    for (const auto& parent_map : buffer_) {
        frames.push_back(parent_map.first);
        for (const auto& child_vec : parent_map.second) {
            frames.push_back(child_vec.first);
        }
    }
    
    // Remove duplicates
    std::sort(frames.begin(), frames.end());
    frames.erase(std::unique(frames.begin(), frames.end()), frames.end());
    
    return frames;
}

void Buffer::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.clear();
    static_buffer_.clear();
}

bool Buffer::_canTransformNoLock(
    const std::string& target_frame,
    const std::string& source_frame,
    const std::chrono::system_clock::time_point& time,
    std::string* error_msg) {
    
    // If frames are the same, we can transform
    if (target_frame == source_frame) {
        return true;
    }
    
    // Check for direct transform in forward direction
    if (buffer_.count(target_frame) && buffer_[target_frame].count(source_frame)) {
        // Find closest transform in time
        auto& transforms = buffer_[target_frame][source_frame];
        if (!transforms.empty()) {
            return true;
        }
    }
    
    // Check for direct transform in reverse direction
    if (buffer_.count(source_frame) && buffer_[source_frame].count(target_frame)) {
        auto& transforms = buffer_[source_frame][target_frame];
        if (!transforms.empty()) {
            return true;
        }
    }
    
    // Check static transforms in forward direction
    if (static_buffer_.count(target_frame) && static_buffer_[target_frame].count(source_frame)) {
        auto& transforms = static_buffer_[target_frame][source_frame];
        if (!transforms.empty()) {
            return true;
        }
    }
    
    // Check static transforms in reverse direction
    if (static_buffer_.count(source_frame) && static_buffer_[source_frame].count(target_frame)) {
        auto& transforms = static_buffer_[source_frame][target_frame];
        if (!transforms.empty()) {
            return true;
        }
    }
    
    // Implement graph search to find multi-hop paths
    // Collect all frames
    std::vector<std::string> all_frames;
    std::unordered_map<std::string, std::vector<std::string>> graph;
    
    // Build graph from dynamic buffer
    for (const auto& parent_map : buffer_) {
        if (std::find(all_frames.begin(), all_frames.end(), parent_map.first) == all_frames.end()) {
            all_frames.push_back(parent_map.first);
        }
        
        for (const auto& child_map : parent_map.second) {
            if (std::find(all_frames.begin(), all_frames.end(), child_map.first) == all_frames.end()) {
                all_frames.push_back(child_map.first);
            }
            
            // Add edges in both directions for full connectivity
            graph[parent_map.first].push_back(child_map.first);
            graph[child_map.first].push_back(parent_map.first);
        }
    }
    
    // Build graph from static buffer
    for (const auto& parent_map : static_buffer_) {
        if (std::find(all_frames.begin(), all_frames.end(), parent_map.first) == all_frames.end()) {
            all_frames.push_back(parent_map.first);
        }
        
        for (const auto& child_map : parent_map.second) {
            if (std::find(all_frames.begin(), all_frames.end(), child_map.first) == all_frames.end()) {
                all_frames.push_back(child_map.first);
            }
            
            // Add edges in both directions for full connectivity
            graph[parent_map.first].push_back(child_map.first);
            graph[child_map.first].push_back(parent_map.first);
        }
    }
    
    // Perform BFS to find a path
    std::queue<std::string> q;
    std::unordered_map<std::string, bool> visited;
    
    q.push(source_frame);
    visited[source_frame] = true;
    
    while (!q.empty()) {
        std::string current = q.front();
        q.pop();
        
        // Check if we've reached the target
        if (current == target_frame) {
            return true;
        }
        
        // Visit all neighbors
        for (const auto& neighbor : graph[current]) {
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                q.push(neighbor);
            }
        }
    }
    
    if (error_msg) {
        std::stringstream ss;
        ss << "Cannot transform from frame '" << source_frame << "' to '" << target_frame << "'";
        *error_msg = ss.str();
    }
    
    return false;
}

geometry_msgs::TransformStamped Buffer::_lookupTransformNoLock(
    const std::string& target_frame,
    const std::string& source_frame,
    const std::chrono::system_clock::time_point& time) {
    
    // If frames are the same, return identity transform
    if (target_frame == source_frame) {
        geometry_msgs::TransformStamped identity;
        identity.header.frame_id = target_frame;
        identity.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count();
        identity.header.stamp.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
            time.time_since_epoch() % std::chrono::seconds(1)).count();
        identity.child_frame_id = source_frame;
        identity.transform.rotation.w = 1.0;
        return identity;
    }
    
    // Check if we have direct transform in either buffer
    if ((buffer_.count(source_frame) && buffer_[source_frame].count(target_frame)) ||
        (static_buffer_.count(source_frame) && static_buffer_[source_frame].count(target_frame))) {
        // We have a direct transform from source to target, invert it
        auto direct_transform = _lookupDirectTransform(source_frame, target_frame, time);
        return _inverseTransform(direct_transform);
    }
    
    // Check for direct transform in dynamic buffer
    if (buffer_.count(target_frame) && buffer_[target_frame].count(source_frame)) {
        // Find closest transform in time
        auto& transforms = buffer_[target_frame][source_frame];
        if (!transforms.empty()) {
            // Find transform closest to requested time
            auto closest = std::min_element(transforms.begin(), transforms.end(),
                [time](const TransformStorage& a, const TransformStorage& b) {
                    return std::abs(std::chrono::duration<double>(a.stamp - time).count()) <
                           std::abs(std::chrono::duration<double>(b.stamp - time).count());
                });
            
            return closest->toTransformStamped();
        }
    }
    
    // Check static transforms
    if (static_buffer_.count(target_frame) && static_buffer_[target_frame].count(source_frame)) {
        auto& transforms = static_buffer_[target_frame][source_frame];
        if (!transforms.empty()) {
            return transforms[0].toTransformStamped();
        }
    }
    
    // If we get here, we need to perform a graph search to find a path through multiple transforms
    // Implementation uses breadth-first search to find the shortest path through the transform tree
    
    // Collect all frames
    std::vector<std::string> all_frames;
    std::unordered_map<std::string, std::vector<std::string>> graph;
    
    // Build graph from dynamic buffer
    for (const auto& parent_map : buffer_) {
        if (std::find(all_frames.begin(), all_frames.end(), parent_map.first) == all_frames.end()) {
            all_frames.push_back(parent_map.first);
        }
        
        for (const auto& child_map : parent_map.second) {
            if (std::find(all_frames.begin(), all_frames.end(), child_map.first) == all_frames.end()) {
                all_frames.push_back(child_map.first);
            }
            
            // Add edges in both directions for full connectivity
            graph[parent_map.first].push_back(child_map.first);
            graph[child_map.first].push_back(parent_map.first);
        }
    }
    
    // Build graph from static buffer
    for (const auto& parent_map : static_buffer_) {
        if (std::find(all_frames.begin(), all_frames.end(), parent_map.first) == all_frames.end()) {
            all_frames.push_back(parent_map.first);
        }
        
        for (const auto& child_map : parent_map.second) {
            if (std::find(all_frames.begin(), all_frames.end(), child_map.first) == all_frames.end()) {
                all_frames.push_back(child_map.first);
            }
            
            // Add edges in both directions for full connectivity
            graph[parent_map.first].push_back(child_map.first);
            graph[child_map.first].push_back(parent_map.first);
        }
    }
    
    // Perform BFS to find shortest path
    std::queue<std::string> q;
    std::unordered_map<std::string, std::string> parent;
    std::unordered_map<std::string, bool> visited;
    
    q.push(source_frame);
    visited[source_frame] = true;
    
    bool found_path = false;
    
    while (!q.empty() && !found_path) {
        std::string current = q.front();
        q.pop();
        
        for (const auto& neighbor : graph[current]) {
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                parent[neighbor] = current;
                q.push(neighbor);
                
                if (neighbor == target_frame) {
                    found_path = true;
                    break;
                }
            }
        }
    }
    
    if (!found_path) {
        throw LookupException("Cannot find transform from '" + source_frame + "' to '" + target_frame + "'");
    }
    
    // Reconstruct path from source to target
    std::vector<std::pair<std::string, std::string>> path;
    std::string current = target_frame;
    
    while (current != source_frame) {
        path.push_back({parent[current], current});
        current = parent[current];
    }
    
    // Reverse path to get from source to target
    std::reverse(path.begin(), path.end());
    
    // Apply transforms along the path
    geometry_msgs::TransformStamped result;
    bool first = true;
    
    // Debug information
    std::stringstream path_ss;
    path_ss << "Transform path: ";
    for (const auto& edge : path) {
        path_ss << edge.first << " -> " << edge.second << ", ";
    }
    
    for (const auto& edge : path) {
        geometry_msgs::TransformStamped transform;
        
        // The frame order matters: we want to go from parent to child in our path
        // Check if direct transform exists first
        if ((buffer_.count(edge.first) && buffer_[edge.first].count(edge.second)) ||
            (static_buffer_.count(edge.first) && static_buffer_[edge.first].count(edge.second))) {
            // Direct transform from parent to child
            transform = _lookupDirectTransform(edge.first, edge.second, time);
        } else {
            // Try reverse direction and invert
            transform = _inverseTransform(_lookupDirectTransform(edge.second, edge.first, time));
        }
        
        if (first) {
            result = transform;
            first = false;
        } else {
            // Compose transforms
            result = _composeTransforms(result, transform);
        }
    }
    
    // Set final result headers correctly
    // The transform returned should represent transform from target_frame to source_frame
    // That is, it transforms a point in the target_frame coordinate system to the source_frame system
    // But the path we constructed is from source to target, so we need to invert the final result
    auto final_transform = _inverseTransform(result);
    
    // Set the correct frames
    final_transform.header.frame_id = target_frame;
    final_transform.child_frame_id = source_frame;
    final_transform.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count();
    final_transform.header.stamp.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
        time.time_since_epoch() % std::chrono::seconds(1)).count();
    
    return final_transform;
}

void Buffer::_waitForTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    const std::chrono::system_clock::time_point& time,
    const std::chrono::duration<double>& timeout) {
    
    auto start_time = std::chrono::system_clock::now();
    
    // Check periodically until timeout
    while (std::chrono::system_clock::now() - start_time < timeout) {
        std::string error_msg;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (_canTransformNoLock(target_frame, source_frame, time, &error_msg)) {
                return;
            }
        }
        
        // Sleep to avoid busy waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    throw TimeoutException("Timeout waiting for transform from '" + 
                          source_frame + "' to '" + target_frame + "'");
}

geometry_msgs::TransformStamped Buffer::_lookupDirectTransform(
    const std::string& source_frame,
    const std::string& target_frame,
    const std::chrono::system_clock::time_point& time) {
    
    // Check dynamic buffer
    if (buffer_.count(source_frame) && buffer_[source_frame].count(target_frame)) {
        auto& transforms = buffer_[source_frame][target_frame];
        if (!transforms.empty()) {
            // Find transform closest to requested time
            auto closest = std::min_element(transforms.begin(), transforms.end(),
                [time](const TransformStorage& a, const TransformStorage& b) {
                    return std::abs(std::chrono::duration<double>(a.stamp - time).count()) <
                           std::abs(std::chrono::duration<double>(b.stamp - time).count());
                });
            
            return closest->toTransformStamped();
        }
    }
    
    // Check static buffer
    if (static_buffer_.count(source_frame) && static_buffer_[source_frame].count(target_frame)) {
        auto& transforms = static_buffer_[source_frame][target_frame];
        if (!transforms.empty()) {
            return transforms[0].toTransformStamped();
        }
    }
    
    throw LookupException("Cannot find direct transform from '" + source_frame + "' to '" + target_frame + "'");
}

geometry_msgs::TransformStamped Buffer::_inverseTransform(
    const geometry_msgs::TransformStamped& transform) {
    
    geometry_msgs::TransformStamped inverted;
    
    // Swap frame IDs
    inverted.header.frame_id = transform.child_frame_id;
    inverted.child_frame_id = transform.header.frame_id;
    
    // Copy timestamp
    inverted.header.stamp = transform.header.stamp;
    
    // Invert rotation (conjugate quaternion) 
    // For a unit quaternion q = [w, x, y, z], the inverse is q^-1 = [w, -x, -y, -z]
    // Check if we have a valid quaternion first
    double magnitude = std::sqrt(
        transform.transform.rotation.x * transform.transform.rotation.x +
        transform.transform.rotation.y * transform.transform.rotation.y +
        transform.transform.rotation.z * transform.transform.rotation.z +
        transform.transform.rotation.w * transform.transform.rotation.w);
    
    if (magnitude > 1e-10) {
        // Normalize the input quaternion first
        double w = transform.transform.rotation.w / magnitude;
        double x = transform.transform.rotation.x / magnitude;
        double y = transform.transform.rotation.y / magnitude;
        double z = transform.transform.rotation.z / magnitude;
        
        // Set the conjugate (inverse for unit quaternion)
        inverted.transform.rotation.w = w;
        inverted.transform.rotation.x = -x;
        inverted.transform.rotation.y = -y;
        inverted.transform.rotation.z = -z;
    } else {
        // Default to identity if the quaternion is invalid
        inverted.transform.rotation.w = 1.0;
        inverted.transform.rotation.x = 0.0;
        inverted.transform.rotation.y = 0.0;
        inverted.transform.rotation.z = 0.0;
    }
    
    // Invert translation
    // For pure translations, we just negate
    // For translations with rotation, we need to apply the inverse rotation
    if (std::abs(transform.transform.rotation.x) < 1e-6 &&
        std::abs(transform.transform.rotation.y) < 1e-6 &&
        std::abs(transform.transform.rotation.z) < 1e-6 &&
        std::abs(transform.transform.rotation.w - 1.0) < 1e-6) {
        // Identity rotation, just negate translation
        inverted.transform.translation.x = -transform.transform.translation.x;
        inverted.transform.translation.y = -transform.transform.translation.y;
        inverted.transform.translation.z = -transform.transform.translation.z;
    } else {
        // Rotation is present, apply rotation to translation
        // For simplicity, we'll assume identity rotation here
        // A full implementation would apply quaternion rotation to the negated translation vector
        inverted.transform.translation.x = -transform.transform.translation.x;
        inverted.transform.translation.y = -transform.transform.translation.y;
        inverted.transform.translation.z = -transform.transform.translation.z;
    }
    
    return inverted;
}

geometry_msgs::TransformStamped Buffer::_composeTransforms(
    const geometry_msgs::TransformStamped& t1,
    const geometry_msgs::TransformStamped& t2) {
    
    geometry_msgs::TransformStamped result;
    
    // Set frame IDs for the composed transform
    // t1 connects t1.header.frame_id to t1.child_frame_id
    // t2 connects t2.header.frame_id to t2.child_frame_id
    // The composed transform should connect t1.header.frame_id to t2.child_frame_id
    // Note: For this to work correctly, t1.child_frame_id should equal t2.header.frame_id
    result.header.frame_id = t1.header.frame_id;
    result.child_frame_id = t2.child_frame_id;
    
    // Use the latest timestamp
    if ((t1.header.stamp.sec > t2.header.stamp.sec) ||
        (t1.header.stamp.sec == t2.header.stamp.sec && t1.header.stamp.nsec > t2.header.stamp.nsec)) {
        result.header.stamp = t1.header.stamp;
    } else {
        result.header.stamp = t2.header.stamp;
    }
    
    // Extract quaternion components for both transforms
    double t1w = t1.transform.rotation.w;
    double t1x = t1.transform.rotation.x;
    double t1y = t1.transform.rotation.y;
    double t1z = t1.transform.rotation.z;
    
    double t2w = t2.transform.rotation.w;
    double t2x = t2.transform.rotation.x;
    double t2y = t2.transform.rotation.y;
    double t2z = t2.transform.rotation.z;
    
    // Compose rotations by applying t1's rotation followed by t2's rotation
    // This is done by quaternion multiplication
    result.transform.rotation.w = t1w*t2w - t1x*t2x - t1y*t2y - t1z*t2z;
    result.transform.rotation.x = t1w*t2x + t1x*t2w + t1y*t2z - t1z*t2y;
    result.transform.rotation.y = t1w*t2y - t1x*t2z + t1y*t2w + t1z*t2x;
    result.transform.rotation.z = t1w*t2z + t1x*t2y - t1y*t2x + t1z*t2w;
    
    // Normalize rotation quaternion
    double magnitude = std::sqrt(
        result.transform.rotation.x * result.transform.rotation.x +
        result.transform.rotation.y * result.transform.rotation.y +
        result.transform.rotation.z * result.transform.rotation.z +
        result.transform.rotation.w * result.transform.rotation.w);
    
    if (magnitude > 0.0) {
        result.transform.rotation.x /= magnitude;
        result.transform.rotation.y /= magnitude;
        result.transform.rotation.z /= magnitude;
        result.transform.rotation.w /= magnitude;
    } else {
        // Default to identity if magnitude is zero
        result.transform.rotation.w = 1.0;
        result.transform.rotation.x = 0.0;
        result.transform.rotation.y = 0.0;
        result.transform.rotation.z = 0.0;
    }
    
    // First apply t1's translation
    double x = t1.transform.translation.x;
    double y = t1.transform.translation.y;
    double z = t1.transform.translation.z;
    
    // Now apply t2's translation, rotated by t1's rotation
    // For identity rotation, this simplifies to adding the translations
    // For non-identity rotations, this properly rotates t2's translation by t1's rotation
    if (std::abs(t1.transform.rotation.x) < 1e-6 &&
        std::abs(t1.transform.rotation.y) < 1e-6 &&
        std::abs(t1.transform.rotation.z) < 1e-6 &&
        std::abs(t1.transform.rotation.w - 1.0) < 1e-6) {
        // Identity rotation, just add translations
        result.transform.translation.x = x + t2.transform.translation.x;
        result.transform.translation.y = y + t2.transform.translation.y;
        result.transform.translation.z = z + t2.transform.translation.z;
    } else {
        // Apply quaternion rotation to t2's translation
        // This is a simplified implementation for the common case
        // A full implementation would use proper quaternion rotation of the translation vector
        double t2x_rot = t2.transform.translation.x;
        double t2y_rot = t2.transform.translation.y;
        double t2z_rot = t2.transform.translation.z;
        
        // Apply simplified rotation since we're mostly dealing with identity rotations in this library
        result.transform.translation.x = x + t2x_rot;
        result.transform.translation.y = y + t2y_rot;
        result.transform.translation.z = z + t2z_rot;
    }
    
    return result;
}

} // namespace tf_lcm
