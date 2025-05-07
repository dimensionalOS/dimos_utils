#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class TransformPublisher : public rclcpp::Node
{
public:
    TransformPublisher(const std::string &parent_frame, const std::string &child_frame, const std::string &topic_name)
        : Node("transform_publisher"), parent_frame_(parent_frame), child_frame_(child_frame), topic_name_(topic_name)
    {
        // Create a buffer and a transform listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create a publisher for the transform
        transform_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(topic_name_, 10);

        // Set a timer to publish the transform periodically
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // 20 ms = 50 Hz
            std::bind(&TransformPublisher::publishTransform, this));
    }

private:
    void publishTransform()
    {
        try
        {
            // Lookup the transform from 'parent_frame_' to 'child_frame_'
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(parent_frame_, child_frame_, tf2::TimePointZero);

            // Publish the transform to the specified topic
            transform_pub_->publish(transform);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not get transform: %s", ex.what());
        }
    }

    std::string parent_frame_;
    std::string child_frame_;
    std::string topic_name_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Check if sufficient arguments are provided
    std::string parent_frame;
    std::string child_frame;
    std::string topic_name;
    if (argc == 4)
    {
        // Get arguments from the command line
        parent_frame = argv[1];
        child_frame = argv[2];
        topic_name = argv[3];
    }
    if (argc < 4)
    {
        // std::cerr << "Usage: transform_publisher <parent_frame> <child_frame> <topic_name>" << std::endl;
        // return 1;
        std::cout << "Using defaults: parent_frame: world, child_frame: link6, topic_name: /real_arm_tf" << std::endl;
        parent_frame = "world";
        child_frame = "link6";
        topic_name = "/real_arm_tf";
    }

    auto node = std::make_shared<TransformPublisher>(parent_frame, child_frame, topic_name);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
