#include "tf_lcm/listener.hpp"
#include <iostream>

namespace tf_lcm {

TransformListener::TransformListener(double buffer_size)
    : buffer_(std::make_shared<Buffer>(buffer_size)),
      lcm_(std::make_shared<lcm::LCM>()),
      owns_buffer_(true),
      owns_lcm_(true),
      running_(true) {
    
    if (!lcm_->good()) {
        std::cerr << "Failed to initialize LCM in TransformListener" << std::endl;
        return;
    }
    
    subscribeTransforms();
    thread_ = std::thread(&TransformListener::processThread, this);
}

TransformListener::TransformListener(Buffer& buffer)
    : buffer_(std::shared_ptr<Buffer>(&buffer, [](Buffer*) {})),  // Non-owning pointer
      lcm_(std::make_shared<lcm::LCM>()),
      owns_buffer_(false),
      owns_lcm_(true),
      running_(true) {
    
    if (!lcm_->good()) {
        std::cerr << "Failed to initialize LCM in TransformListener" << std::endl;
        return;
    }
    
    subscribeTransforms();
    thread_ = std::thread(&TransformListener::processThread, this);
}

TransformListener::TransformListener(std::shared_ptr<lcm::LCM> lcm, Buffer& buffer)
    : buffer_(std::shared_ptr<Buffer>(&buffer, [](Buffer*) {})),  // Non-owning pointer
      lcm_(lcm),
      owns_buffer_(false),
      owns_lcm_(false),
      running_(true) {
    
    if (!lcm_->good()) {
        std::cerr << "Invalid LCM instance provided to TransformListener" << std::endl;
        return;
    }
    
    subscribeTransforms();
    thread_ = std::thread(&TransformListener::processThread, this);
}

TransformListener::~TransformListener() {
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
}

void TransformListener::subscribeTransforms() {
    // Subscribe to regular transforms
    lcm_->subscribe("tf#tf2_msgs.TFMessage", 
                    &TransformListener::transformsCallback, 
                    this);
    
    // Subscribe to static transforms
    lcm_->subscribe("tf_static#tf2_msgs.TFMessage", 
                    &TransformListener::staticTransformsCallback, 
                    this);
}

void TransformListener::transformsCallback(
    const lcm::ReceiveBuffer* rbuf, 
    const std::string& channel, 
    const tf2_msgs::TFMessage* msg) {
    
    for (size_t i = 0; i < msg->transforms_length; i++) {
        buffer_->setTransform(msg->transforms[i], "lcm_publisher", false);
    }
}

void TransformListener::staticTransformsCallback(
    const lcm::ReceiveBuffer* rbuf, 
    const std::string& channel, 
    const tf2_msgs::TFMessage* msg) {
    
    for (size_t i = 0; i < msg->transforms_length; i++) {
        buffer_->setTransform(msg->transforms[i], "static_publisher", true);
    }
}

void TransformListener::processThread() {
    while (running_ && lcm_->good()) {
        // Process LCM messages with a timeout
        lcm_->handleTimeout(100);  // 100 ms timeout
    }
}

} // namespace tf_lcm
