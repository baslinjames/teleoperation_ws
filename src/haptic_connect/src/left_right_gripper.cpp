#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <control_msgs/action/gripper_command.hpp>
#include <franka_msgs/action/grasp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "mt_interfaces/msg/gripper.hpp"

using namespace std::placeholders;

class GripperActionClient : public rclcpp::Node
{
public:
    using Gripper_open = control_msgs::action::GripperCommand;
    using Grasp = franka_msgs::action::Grasp;

    explicit GripperActionClient(const rclcpp::NodeOptions &options)
        : Node("gripper_action_client", options)
    {
        this->subs1 = create_subscription<mt_interfaces::msg::Gripper>("gripper",
                                                                       10, std::bind(&GripperActionClient::gripper_callback, this, _1));

        this->client_ptr_left_open = rclcpp_action::create_client<Gripper_open>(this, "left_arm_gripper/gripper_action");
        this->client_ptr_left_close = rclcpp_action::create_client<Grasp>(this, "left_arm_gripper/grasp");

        this->client_ptr_right_open = rclcpp_action::create_client<Gripper_open>(this, "right_arm_gripper/gripper_action");
        this->client_ptr_right_close = rclcpp_action::create_client<Grasp>(this, "right_arm_gripper/grasp");

    }

    // subscriber for knowing which robot gripper is to be controlled
    void gripper_callback(const mt_interfaces::msg::Gripper::SharedPtr msg)
    {
        if (msg->side == 0) // for left arm - two finger gripper
        {
            if (msg->actuation_distance == 0.0) // grasp
            {
                if (!this->client_ptr_left_close->wait_for_action_server())
                {
                    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                    rclcpp::shutdown();
                }

                auto goal_msg = Grasp::Goal();
                goal_msg.width = msg->actuation_distance;
                goal_msg.speed = msg->actuation_speed;
                goal_msg.force = msg->actuation_force;
                goal_msg.epsilon.inner = 0.08;
                goal_msg.epsilon.outer = 0.08;

                RCLCPP_INFO(this->get_logger(), "Sending goal to left gripper to grasp");

                this->client_ptr_left_close->async_send_goal(goal_msg);
            }

            else if (msg->actuation_distance > 0.0) // open the gripper to specified distance
            {
                if (msg->actuation_distance <= 0.025)
                {
                    if (!this->client_ptr_left_open->wait_for_action_server())
                    {
                        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                        rclcpp::shutdown();
                    }

                    auto goal_msg = Gripper_open::Goal();
                    goal_msg.command.position = msg->actuation_distance;
                    goal_msg.command.max_effort = -1;

                    RCLCPP_INFO(this->get_logger(), "Sending goal to left gripper to open");

                    this->client_ptr_left_open->async_send_goal(goal_msg);
                }
            }
        }

        else if (msg->side == 1) // for right arm - three finger gripper
        {
            if (msg->actuation_distance == 0.0) // grasp
            {
                if (!this->client_ptr_right_close->wait_for_action_server())
                {
                    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                    rclcpp::shutdown();
                }

                auto goal_msg = Grasp::Goal();
                goal_msg.width = msg->actuation_distance;
                goal_msg.speed = msg->actuation_speed;
                goal_msg.force = msg->actuation_force;
                goal_msg.epsilon.inner = 0.08;
                goal_msg.epsilon.outer = 0.08;

                RCLCPP_INFO(this->get_logger(), "Sending goal to right gripper to grasp");

                this->client_ptr_right_close->async_send_goal(goal_msg);
            }

            else if (msg->actuation_distance > 0.0) // open the gripper to specified distance
            {
                if (msg->actuation_distance <= 0.041)
                {
                    if (!this->client_ptr_right_open->wait_for_action_server())
                    {
                        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                        rclcpp::shutdown();
                    }

                    auto goal_msg = Gripper_open::Goal();
                    goal_msg.command.position = msg->actuation_distance;
                    goal_msg.command.max_effort = -1;

                    RCLCPP_INFO(this->get_logger(), "Sending goal to right gripper to open");

                    this->client_ptr_right_open->async_send_goal(goal_msg);
                }
            }
        }
        // this->subs.reset();
    }

private:
    rclcpp_action::Client<Gripper_open>::SharedPtr client_ptr_right_open;
    rclcpp_action::Client<Grasp>::SharedPtr client_ptr_right_close;
    rclcpp_action::Client<Gripper_open>::SharedPtr client_ptr_left_open;
    rclcpp_action::Client<Grasp>::SharedPtr client_ptr_left_close;
    rclcpp::Subscription<mt_interfaces::msg::Gripper>::SharedPtr subs1;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.use_intra_process_comms(true);

    auto obj = std::make_shared<GripperActionClient>(node_options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(obj);
    executor.spin();

    rclcpp::shutdown();
}