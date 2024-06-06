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

    // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_publisher_ =
    //     node_->create_publisher<geometry_msgs::msg::PoseStamped>("right_endeff_pose", 10);

    const std::string PLANNING_GROUP_LEFT = "left_arm";
    // const std::string PLANNING_GROUP_RIGHT = "right_arm";
    moveit::planning_interface::MoveGroupInterface move_group_left(node_, PLANNING_GROUP_LEFT);
    // moveit::planning_interface::MoveGroupInterface move_group_right(node_, PLANNING_GROUP_RIGHT);

    geometry_msgs::msg::PoseStamped left_current_pose;
    left_current_pose = move_group_left.getCurrentPose();

    // geometry_msgs::msg::PoseStamped right_current_pose;
    // right_current_pose = move_group_right.getCurrentPose();

    std::vector<std::string> joint_names_left = move_group_left.getJointNames();
    // std::vector<std::string> joint_names_right = move_group_right.getJointNames();

    move_group_left.setPoseTarget(left_current_pose);
    move_group_left.setMaxVelocityScalingFactor(0.3);
    move_group_left.move();

    // move_group_right.setPoseTarget(right_current_pose);
    // move_group_right.setMaxVelocityScalingFactor(0.3);
    // move_group_right.move();

    rclcpp::sleep_for(std::chrono::seconds(2));

    left_current_pose = move_group_left.getCurrentPose();

    // right_current_pose = move_group_right.getCurrentPose();

    while (rclcpp::ok())
    {
        left_publisher_->publish(left_current_pose);
        // right_publisher_->publish(right_current_pose);
    }
    return 0;
}