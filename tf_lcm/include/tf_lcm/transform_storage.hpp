#pragma once

#include <string>
#include <chrono>
#include "geometry_msgs/TransformStamped.hpp"

namespace tf_lcm {

/**
 * @brief Storage class for transforms
 * 
 * This class stores a transform along with metadata about its source
 */
class TransformStorage {
public:
    /**
     * @brief Construct an empty transform storage
     */
    TransformStorage();
    
    /**
     * @brief Construct a transform storage from a TransformStamped message
     * 
     * @param transform The transform message
     * @param authority The authority that provided this transform
     */
    TransformStorage(const geometry_msgs::TransformStamped& transform, const std::string& authority);
    
    std::string frame_id;           ///< The frame ID of this transform
    std::string child_frame_id;     ///< The child frame ID of this transform
    std::chrono::system_clock::time_point stamp;  ///< The timestamp of this transform
    std::string authority;          ///< The authority that provided this transform
    
    // Transform data
    double translation_x;
    double translation_y;
    double translation_z;
    double rotation_x;
    double rotation_y;
    double rotation_z;
    double rotation_w;
    
    /**
     * @brief Convert to a TransformStamped message
     * 
     * @return A TransformStamped message
     */
    geometry_msgs::TransformStamped toTransformStamped() const;
};

} // namespace tf_lcm
