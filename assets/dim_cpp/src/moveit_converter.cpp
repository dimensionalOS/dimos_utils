#include "xarm_planner/xarm_planner.h"
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/rclcpp.hpp>

class XArmControllerNode : public rclcpp::Node
{
public:
    XArmControllerNode() : Node("xarm_controller_node"), is_executing_(false)
    {
        // Initialize the planner
        node_ = rclcpp::Node::make_shared("xarm_controller_node");
        int dof;
        node_->get_parameter_or("dof", dof, 6);
        std::string robot_type;
        node_->get_parameter_or("robot_type", robot_type, std::string("xarm"));
        std::string group_name = robot_type;
        if (robot_type == "xarm" || robot_type == "lite")
            group_name = robot_type + std::to_string(dof);
        std::string prefix;
        node_->get_parameter_or("prefix", prefix, std::string(""));
        if (prefix != "")
        {
            group_name = prefix + group_name;
        }

        RCLCPP_INFO(node_->get_logger(), "namespace: %s, group_name: %s", node_->get_namespace(), group_name.c_str());

        group_name = "R_xarm7";
        planner_ = std::make_shared<xarm_planner::XArmPlanner>(node_, group_name);
        // planner_gripper_ = std::make_shared<xarm_planner::XArmPlanner>(node_, robot_type + "_gripper");

        // Subscriber for the target pose
        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/curr_target_pose", 10, std::bind(&XArmControllerNode::poseCallback, this, std::placeholders::_1));

        // Subscriber for the gripper position
        // gripper_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        //     "/gripper_position", 10, std::bind(&XArmControllerNode::gripperCallback, this, std::placeholders::_1));

        // Publisher for the robot status
        status_pub_ = this->create_publisher<std_msgs::msg::Int8>("/robot_status", 10);

        // Initialize robot status as free (0)
        publishStatus(0);
    }

private:
    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        if (is_executing_)
        {
            RCLCPP_WARN(this->get_logger(), "Robot is already executing a trajectory.");
            publishStatus(1); // Robot is busy
            return;
        }

        is_executing_ = true;
        publishStatus(1); // Robot is busy

        RCLCPP_INFO(this->get_logger(), "Received new target pose. Planning and executing...");

        // bool success = planner_->plan_and_execute(*msg);
        bool planned = planner_->planPoseTarget(*msg);
        bool executed = planner_->executePath();
        bool success = planned && executed;

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Motion execution successful.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Motion execution failed.");
        }

        is_executing_ = false;
        publishStatus(0); // Robot is free again
    }

    void gripperCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        RCLCPP_WARN(this->get_logger(), "\nReceived new gripper position. Planning and executing...");
        if (is_executing_)
        {
            RCLCPP_WARN(this->get_logger(), "Robot is already executing a trajectory.");
            publishStatus(1); // Robot is busy
            return;
        }

        is_executing_ = true;
        publishStatus(1); // Robot is busy

        // bool success = planner_gripper_->planJointTarget({msg->data});
        std::vector<double> joint_open(6, 0.0);
        std::vector<double> joint_close(6, 0.85);
        std::vector<double> joint_target;
        if (msg->data > 0.0)
        {
            joint_target = joint_close;
        }
        else
        {
            joint_target = joint_open;
        }
        bool planned = planner_gripper_->planJointTarget(joint_target);
        bool executed = planner_gripper_->executePath();
        bool success = planned && executed;

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Motion execution successful.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Motion execution failed.");
        }

        is_executing_ = false;
        publishStatus(0); // Robot is free again
    }

    void publishStatus(int8_t status)
    {
        auto status_msg = std_msgs::msg::Int8();
        status_msg.data = status;
        status_pub_->publish(status_msg);
    }

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<xarm_planner::XArmPlanner> planner_;
    std::shared_ptr<xarm_planner::XArmPlanner> planner_gripper_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gripper_sub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr status_pub_;
    bool is_executing_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<XArmControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}