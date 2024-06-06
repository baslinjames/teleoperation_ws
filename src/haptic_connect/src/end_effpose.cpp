#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node_ = std::make_shared<rclcpp::Node>("endeffpose");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_);
  std::thread([&executor]() { executor.spin(); }).detach();

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_publisher_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>("left_endeff_pose", 10);
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr endeff_publisher_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>("endeff_pose_cont", 10);
  // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_publisher_ =
  // node_->create_publisher<geometry_msgs::msg::PoseStamped>("right_endeff_pose", 10);

  const std::string PLANNING_GROUP_LEFT = "panda_arm";
  // const std::string PLANNING_GROUP_RIGHT = "right_arm";
  moveit::planning_interface::MoveGroupInterface move_group_left(node_, PLANNING_GROUP_LEFT);
  moveit::planning_interface::MoveGroupInterface move_group_cont(node_, PLANNING_GROUP_LEFT);
  // moveit::planning_interface::MoveGroupInterface move_group_right(node_,PLANNING_GROUP_RIGHT);

  // std::vector<std::string> joint_names = move_group_left.getJointNames();
  // std::vector<double> joint_values = { 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785 };
  // std::vector<double> joint_values = { 0.011, -1.108, 0.569, -2.209, 0.6397, 1.271, -0.276 };
  // move_group_left.setJointValueTarget(joint_names, joint_values);
  // move_group_left.setMaxVelocityScalingFactor(0.3);
  // move_group_left.move();

  rclcpp::sleep_for(std::chrono::seconds(2));

  geometry_msgs::msg::PoseStamped left_current_pose;
  left_current_pose = move_group_left.getCurrentPose();
  // geometry_msgs::msg::PoseStamped right_current_pose;

  while (rclcpp::ok())
  {
    // left_current_pose = move_group_left.getCurrentPose();
    // right_current_pose = move_group_right.getCurrentPose();

    left_publisher_->publish(left_current_pose);
    // right_publisher_->publish(right_current_pose);

    geometry_msgs::msg::PoseStamped current_pose_cont;
    current_pose_cont = move_group_cont.getCurrentPose();
    endeff_publisher_->publish(current_pose_cont);
  }
  return 0;
}