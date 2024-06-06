#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node_ = std::make_shared<rclcpp::Node>("endeffpose");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_publisher_ =
        node_->create_publisher<geometry_msgs::msg::PoseStamped>("left_endeff_pose", 10);
    // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr endeff_publisher_ =
    //     node_->create_publisher<geometry_msgs::msg::PoseStamped>("endeff_pose_cont", 10);
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_publisher_ =
        node_->create_publisher<geometry_msgs::msg::PoseStamped>("right_endeff_pose", 10);

    const std::string PLANNING_GROUP_LEFT = "left_arm";
    const std::string PLANNING_GROUP_RIGHT = "right_arm";
    moveit::planning_interface::MoveGroupInterface move_group_left(node_, PLANNING_GROUP_LEFT);
    // moveit::planning_interface::MoveGroupInterface move_group_cont(node_, PLANNING_GROUP_LEFT);
    moveit::planning_interface::MoveGroupInterface move_group_right(node_, PLANNING_GROUP_RIGHT);

    std::vector<std::string> joint_names_left = move_group_left.getJointNames();
    std::vector<std::string> joint_names_right = move_group_right.getJointNames();

    std::vector<double> joint_values_left = {-0.685909, 0.873820, -0.947615, -2.378277, 1.429380, 2.320307, 1.38619};
    std::vector<double> joint_values_right = {1.023312, 0.477689, 0.726166, -2.39195, -0.683174, 2.658754, -0.288370};

    move_group_left.setJointValueTarget(joint_names_left, joint_values_left);
    move_group_left.setMaxVelocityScalingFactor(0.3);
    move_group_left.move();

    move_group_right.setJointValueTarget(joint_names_right, joint_values_right);
    move_group_right.setMaxVelocityScalingFactor(0.3);
    move_group_right.move();

    geometry_msgs::msg::PoseStamped left_current_pose;
    // left_current_pose = move_group_left.getCurrentPose();
    geometry_msgs::msg::PoseStamped right_current_pose;
    
    left_current_pose = move_group_left.getCurrentPose();
    right_current_pose = move_group_right.getCurrentPose();

    while (rclcpp::ok())
    {

        left_publisher_->publish(left_current_pose);
        right_publisher_->publish(right_current_pose);

        // geometry_msgs::msg::PoseStamped current_pose_cont;
        // current_pose_cont = move_group_cont.getCurrentPose();
        // endeff_publisher_->publish(current_pose_cont);
    }
    return 0;
}