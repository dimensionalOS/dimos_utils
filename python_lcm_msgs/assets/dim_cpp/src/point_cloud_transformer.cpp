#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class PointCloudTransformer : public rclcpp::Node
{
public:
    PointCloudTransformer() : Node("point_cloud_transformer"),
                              tf_buffer_(this->get_clock()),
                              tf_listener_(tf_buffer_)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/point_cloud", 10,
            std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try
        {
            // Get the transform from the 'world' frame to the point cloud's frame
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                "world", msg->header.frame_id, rclcpp::Time(0), rclcpp::Duration(1, 0));

            // Transform the point cloud
            sensor_msgs::msg::PointCloud2 transformed_cloud;
            tf2::doTransform(*msg, transformed_cloud, transform);

            // Here you can process the transformed cloud as needed
            // RCLCPP_INFO(this->get_logger(), "Transformed point cloud available.");
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
