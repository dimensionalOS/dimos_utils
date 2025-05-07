#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <mutex>
#include <iomanip>
#include <ctime>
#include <time.h>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

class DataRecorderNode : public rclcpp::Node
{
public:
    DataRecorderNode()
        : Node("data_recorder_node"), is_recording_(false)
    {
        // Parameters for the topics
        this->declare_parameter("image_topic_1", "/oak/color/image");
        this->declare_parameter("image_topic_2", "/oak/stereo/depth");
        this->declare_parameter("pointcloud_topic", "/oak/stereo/points");
        this->declare_parameter("transform_topic", "/real_arm_tf");
        this->declare_parameter("float_topic", "/gripper_position");
        this->declare_parameter("bool_topic", "/is_recording");

        // File path to save recordings
        this->declare_parameter("output_directory", "./recordings");

        // Retrieve the parameters
        image_topic_1_ = this->get_parameter("image_topic_1").as_string();
        image_topic_2_ = this->get_parameter("image_topic_2").as_string();
        pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
        transform_topic_ = this->get_parameter("transform_topic").as_string();
        float_topic_ = this->get_parameter("float_topic").as_string();
        bool_topic_ = this->get_parameter("bool_topic").as_string();
        output_directory_ = this->get_parameter("output_directory").as_string();

        // QoS object with best effort reliability
        // rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        // rclcpp::QoS qos_profile = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        // Create subscribers
        image_sub_1_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_1_, 10, std::bind(&DataRecorderNode::imageCallback1, this, std::placeholders::_1));
        image_sub_2_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_2_, 10, std::bind(&DataRecorderNode::imageCallback2, this, std::placeholders::_1));
        // pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     pointcloud_topic_, qos_profile, std::bind(&DataRecorderNode::pointcloudCallback, this, std::placeholders::_1));
        transform_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
            transform_topic_, 10, std::bind(&DataRecorderNode::transformCallback, this, std::placeholders::_1));
        float_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            float_topic_, 10, std::bind(&DataRecorderNode::floatCallback, this, std::placeholders::_1));

        bool_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            bool_topic_, 10, std::bind(&DataRecorderNode::boolCallback, this, std::placeholders::_1));

        // Timer to handle recording at 5Hz (200ms)
        timer_ = this->create_wall_timer(200ms, std::bind(&DataRecorderNode::timerCallback, this));
    }

private:
    // Callback to start/stop recording based on bool topic
    void boolCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !is_recording_)
        {
            startRecording();
        }
        else if (!msg->data && is_recording_)
        {
            stopRecording();
        }
    }

    // Callbacks to handle incoming messages, store the most recent data
    void imageCallback1(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_image_1_ = msg;
    }

    void imageCallback2(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_image_2_ = msg;
    }

    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_pointcloud_ = msg;
    }

    void transformCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_transform_ = msg;
    }

    void floatCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_float_ = msg;
    }

    // Timer callback to periodically save the buffered data
    void timerCallback()
    {
        if (is_recording_)
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            frames_saved_++;

            std::string stamp = std::to_string(this->now().seconds());

            if (last_image_1_)
            {
                saveImage(last_image_1_, "image", stamp);
            }
            if (last_image_2_)
            {
                saveImage(last_image_2_, "depth", stamp);
            }
            // if (last_pointcloud_)
            //     savePointCloud(last_pointcloud_);
            if (last_transform_)
            {
                saveTransform(last_transform_, stamp);
            }
            if (last_float_)
            {
                saveFloat(last_float_, stamp);
            }
            else
            {
                saveFloat(std::make_shared<std_msgs::msg::Float32>(), stamp);
            }

            RCLCPP_INFO(this->get_logger(), "Recording data at 5Hz.");
        }
    }

    void startRecording()
    {
        is_recording_ = true;
        frames_saved_ = 0;

        // get current time
        recording_start_time_ = std::chrono::system_clock::now();
        std::time_t start_time_t = std::chrono::system_clock::to_time_t(recording_start_time_);

        // convert time to human readable format
        std::stringstream ss;
        recording_dir_ = output_directory_ + "/recording_" + ss.str();
        // create directory for the recording
        ss << std::put_time(std::localtime(&start_time_t), "%Y-%m-%d_%H-%M-%S");
        recording_dir_ = output_directory_ + "/recording_" + ss.str();

        fs::create_directories(recording_dir_);
        RCLCPP_INFO(this->get_logger(), "Started recording in folder: %s", recording_dir_.c_str());
    }

    void stopRecording()
    {
        is_recording_ = false;

        // Get the end time of the recording
        recording_end_time_ = std::chrono::system_clock::now();
        std::time_t end_time_t = std::chrono::system_clock::to_time_t(recording_end_time_);

        // Create a file to store the summary information
        std::string summary_filename = recording_dir_ + "/recording_summary.txt";
        std::ofstream summary_file(summary_filename);

        if (summary_file.is_open())
        {
            // Write the start and end timestamps to the summary file
            std::time_t start_time_t = std::chrono::system_clock::to_time_t(recording_start_time_);
            summary_file << "Recording Start Time: " << std::put_time(std::localtime(&start_time_t), "%Y-%m-%d %H:%M:%S") << "\n";
            summary_file << "Recording End Time: " << std::put_time(std::localtime(&end_time_t), "%Y-%m-%d %H:%M:%S") << "\n";

            // Write the number of frames saved during the recording
            summary_file << "Total Frames Saved: " << frames_saved_ << "\n";

            summary_file.close();
            RCLCPP_INFO(this->get_logger(), "Saved recording summary to %s", summary_filename.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open recording summary file.");
        }

        RCLCPP_INFO(this->get_logger(), "Stopped recording.");
    }

    // Helper function to save image data as PNG
    void saveImage(const sensor_msgs::msg::Image::SharedPtr msg, const std::string &topic_name, std::string timestamp)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr;
            bool depth_image = false;

            // Handle different encodings appropriately
            if (msg->encoding == sensor_msgs::image_encodings::BGR8)
            {
                // For BGR8 images, directly convert and save
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            else if (msg->encoding == sensor_msgs::image_encodings::MONO8)
            {
                // For grayscale images
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
            }
            else if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
            {
                depth_image = true;
                // For 16-bit unsigned depth images, convert and save
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

                // Save the original 16-bit depth image (preserve data integrity)
                std::string depth_filename = recording_dir_ + "/" + topic_name + "." + timestamp + ".png";
                cv::imwrite(depth_filename, cv_ptr->image); // Save the 16-bit depth image

                // Optionally: Save a scaled 8-bit version of the depth image for visualization
                // cv::Mat depth_image_8bit;
                // cv_ptr->image.convertTo(depth_image_8bit, CV_8UC1, 255.0 / 65535.0); // Scaling to 8-bit
                // std::string visual_filename = recording_dir_ + "/" + topic_name + "_" + std::to_string(msg->header.stamp.sec) + "_depth_visual.png";
                // cv::imwrite(visual_filename, depth_image_8bit); // Save the 8-bit visualization
                // return;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
                return; // Exit if the encoding is not supported
            }

            // Save the image (for both BGR8 and MONO8)
            if (!depth_image)
            {
                std::string filename = recording_dir_ + "/" + topic_name + "." + timestamp + ".png";
                cv::imwrite(filename, cv_ptr->image);
            }
            cv_ptr.reset(); // Free image memory after saving
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CVBridge error: %s", e.what());
        }
    }

    // Helper function to save point cloud data as PCD
    void savePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, std::string timestamp)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        std::string filename = recording_dir_ + "/pointcloud_" + timestamp + ".pcd";
        pcl::io::savePCDFileASCII(filename, cloud);
        cloud.clear(); // Free memory after saving
    }

    // Helper function to save transform data as text
    void saveTransform(const geometry_msgs::msg::TransformStamped::SharedPtr msg, std::string timestamp)
    {
        std::string filename = recording_dir_ + "/real_arm_tf." + timestamp + ".txt";
        std::ofstream file(filename);
        file << msg->transform.translation.x << "\n";
        file << msg->transform.translation.y << "\n";
        file << msg->transform.translation.z << "\n";
        file << msg->transform.rotation.x << "\n";
        file << msg->transform.rotation.y << "\n";
        file << msg->transform.rotation.z << "\n";
        file << msg->transform.rotation.w << "\n";
        file.close(); // Free file resources
    }

    // Helper function to save float data as text
    void saveFloat(const std_msgs::msg::Float32::SharedPtr msg, std::string timestamp)
    {
        std::string filename = recording_dir_ + "/gripper_position." + timestamp + ".txt";
        std::ofstream file(filename);
        file << msg->data << "\n";
        file.close(); // Free file resources
    }

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_1_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_2_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr transform_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr float_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables to store the latest data for each topic
    sensor_msgs::msg::Image::SharedPtr last_image_1_;
    sensor_msgs::msg::Image::SharedPtr last_image_2_;
    sensor_msgs::msg::PointCloud2::SharedPtr last_pointcloud_;
    geometry_msgs::msg::TransformStamped::SharedPtr last_transform_;
    std_msgs::msg::Float32::SharedPtr last_float_;

    // Mutex to protect access to the shared data
    std::mutex data_mutex_;

    // Variables to track state and output paths
    bool is_recording_;
    std::chrono::system_clock::time_point recording_start_time_;
    std::chrono::system_clock::time_point recording_end_time_;
    int frames_saved_;
    std::string recording_dir_;
    std::string image_topic_1_, image_topic_2_, pointcloud_topic_, transform_topic_, float_topic_, bool_topic_;
    std::string output_directory_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataRecorderNode>());
    rclcpp::shutdown();
    return 0;
}
