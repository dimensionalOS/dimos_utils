#pragma once

#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <lcm/lcm-cpp.hpp>
#include "tf_lcm/buffer.hpp"
#include "tf2_msgs/TFMessage.hpp"

namespace tf_lcm {

/**
 * @brief Listener for transforms
 * 
 * This class listens for transform messages and adds them to a Buffer.
 */
class TransformListener {
public:
    /**
     * @brief Construct a listener with a new buffer
     * 
     * @param buffer_size How long to keep transform data in the buffer (in seconds)
     */
    TransformListener(double buffer_size = 10.0);
    
    /**
     * @brief Construct a listener with an existing buffer
     * 
     * @param buffer The buffer to add transforms to
     */
    TransformListener(Buffer& buffer);
    
    /**
     * @brief Construct a listener with a provided LCM instance
     * 
     * @param lcm Shared pointer to an LCM instance
     * @param buffer The buffer to add transforms to
     */
    TransformListener(std::shared_ptr<lcm::LCM> lcm, Buffer& buffer);
    
    /**
     * @brief Destructor
     */
    ~TransformListener();
    
    /**
     * @brief Get the buffer
     * 
     * @return Reference to the buffer
     */
    Buffer& getBuffer() { return *buffer_; }
    
    /**
     * @brief Get the buffer (const version)
     * 
     * @return Const reference to the buffer
     */
    const Buffer& getBuffer() const { return *buffer_; }

private:
    std::shared_ptr<Buffer> buffer_;
    std::shared_ptr<lcm::LCM> lcm_;
    bool owns_buffer_;
    bool owns_lcm_;
    std::atomic<bool> running_;
    std::thread thread_;
    
    void subscribeTransforms();
    void transformsCallback(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const tf2_msgs::TFMessage* msg);
    void staticTransformsCallback(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const tf2_msgs::TFMessage* msg);
    void processThread();
};

} // namespace tf_lcm
