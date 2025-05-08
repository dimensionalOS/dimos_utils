#include "tf_lcm/broadcaster.hpp"
#include <iostream>

namespace tf_lcm {

// Regular Transform Broadcaster
TransformBroadcaster::TransformBroadcaster()
    : lcm_(std::make_shared<lcm::LCM>()),
      owns_lcm_(true) {
    if (!lcm_->good()) {
        std::cerr << "Failed to initialize LCM in TransformBroadcaster" << std::endl;
    }
}

TransformBroadcaster::TransformBroadcaster(std::shared_ptr<lcm::LCM> lcm)
    : lcm_(lcm),
      owns_lcm_(false) {
    if (!lcm_->good()) {
        std::cerr << "Invalid LCM instance provided to TransformBroadcaster" << std::endl;
    }
}

TransformBroadcaster::~TransformBroadcaster() {
    // Nothing to clean up here as we use smart pointers
}

void TransformBroadcaster::sendTransform(const geometry_msgs::TransformStamped& transform) {
    tf2_msgs::TFMessage message;
    message.transforms_length = 1;
    message.transforms.push_back(transform);
    
    if (lcm_->good()) {
        lcm_->publish("tf#tf2_msgs.TFMessage", &message);
    } else {
        std::cerr << "LCM not initialized in TransformBroadcaster::sendTransform" << std::endl;
    }
}

void TransformBroadcaster::sendTransform(const std::vector<geometry_msgs::TransformStamped>& transforms) {
    if (transforms.empty()) {
        return;
    }
    
    tf2_msgs::TFMessage message;
    message.transforms_length = transforms.size();
    message.transforms = transforms;
    
    if (lcm_->good()) {
        lcm_->publish("tf#tf2_msgs.TFMessage", &message);
    } else {
        std::cerr << "LCM not initialized in TransformBroadcaster::sendTransform" << std::endl;
    }
}

// Static Transform Broadcaster
StaticTransformBroadcaster::StaticTransformBroadcaster()
    : lcm_(std::make_shared<lcm::LCM>()),
      owns_lcm_(true) {
    if (!lcm_->good()) {
        std::cerr << "Failed to initialize LCM in StaticTransformBroadcaster" << std::endl;
    }
}

StaticTransformBroadcaster::StaticTransformBroadcaster(std::shared_ptr<lcm::LCM> lcm)
    : lcm_(lcm),
      owns_lcm_(false) {
    if (!lcm_->good()) {
        std::cerr << "Invalid LCM instance provided to StaticTransformBroadcaster" << std::endl;
    }
}

StaticTransformBroadcaster::~StaticTransformBroadcaster() {
    // Nothing to clean up here as we use smart pointers
}

void StaticTransformBroadcaster::sendTransform(const geometry_msgs::TransformStamped& transform) {
    tf2_msgs::TFMessage message;
    message.transforms_length = 1;
    message.transforms.push_back(transform);
    
    if (lcm_->good()) {
        lcm_->publish("tf_static#tf2_msgs.TFMessage", &message);
    } else {
        std::cerr << "LCM not initialized in StaticTransformBroadcaster::sendTransform" << std::endl;
    }
}

void StaticTransformBroadcaster::sendTransform(const std::vector<geometry_msgs::TransformStamped>& transforms) {
    if (transforms.empty()) {
        return;
    }
    
    tf2_msgs::TFMessage message;
    message.transforms_length = transforms.size();
    message.transforms = transforms;
    
    if (lcm_->good()) {
        lcm_->publish("tf_static#tf2_msgs.TFMessage", &message);
    } else {
        std::cerr << "LCM not initialized in StaticTransformBroadcaster::sendTransform" << std::endl;
    }
}

} // namespace tf_lcm
