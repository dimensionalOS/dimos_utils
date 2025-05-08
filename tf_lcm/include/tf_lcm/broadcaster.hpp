#pragma once

#include <string>
#include <vector>
#include <memory>
#include <lcm/lcm-cpp.hpp>
#include "geometry_msgs/TransformStamped.hpp"
#include "tf2_msgs/TFMessage.hpp"

namespace tf_lcm {

/**
 * @brief Broadcaster for transforms
 * 
 * This class is used to broadcast transforms to the TF tree.
 */
class TransformBroadcaster {
public:
    /**
     * @brief Construct a broadcaster
     */
    TransformBroadcaster();
    
    /**
     * @brief Construct a broadcaster with a provided LCM instance
     * 
     * @param lcm Shared pointer to an LCM instance
     */
    TransformBroadcaster(std::shared_ptr<lcm::LCM> lcm);
    
    /**
     * @brief Destructor
     */
    ~TransformBroadcaster();
    
    /**
     * @brief Send a transform
     * 
     * @param transform The transform to send
     */
    void sendTransform(const geometry_msgs::TransformStamped& transform);
    
    /**
     * @brief Send multiple transforms
     * 
     * @param transforms The transforms to send
     */
    void sendTransform(const std::vector<geometry_msgs::TransformStamped>& transforms);

private:
    std::shared_ptr<lcm::LCM> lcm_;
    bool owns_lcm_;
};

/**
 * @brief Static transform broadcaster for fixed transforms
 * 
 * This class is used to broadcast static transforms that do not change over time.
 */
class StaticTransformBroadcaster {
public:
    /**
     * @brief Construct a static broadcaster
     */
    StaticTransformBroadcaster();
    
    /**
     * @brief Construct a static broadcaster with a provided LCM instance
     * 
     * @param lcm Shared pointer to an LCM instance
     */
    StaticTransformBroadcaster(std::shared_ptr<lcm::LCM> lcm);
    
    /**
     * @brief Destructor
     */
    ~StaticTransformBroadcaster();
    
    /**
     * @brief Send a static transform
     * 
     * @param transform The transform to send
     */
    void sendTransform(const geometry_msgs::TransformStamped& transform);
    
    /**
     * @brief Send multiple static transforms
     * 
     * @param transforms The transforms to send
     */
    void sendTransform(const std::vector<geometry_msgs::TransformStamped>& transforms);

private:
    std::shared_ptr<lcm::LCM> lcm_;
    bool owns_lcm_;
};

} // namespace tf_lcm
