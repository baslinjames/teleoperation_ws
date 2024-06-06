#include <memory>
#define BOOST_BIND_NO_PLACEHOLDERS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"
#include "mt_interfaces/msg/gripper.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class MinimalSubPub : public rclcpp::Node
{
public:
    MinimalSubPub()
            : Node("minimal_subscriber_publisher")
    {
        publisher_ = this->create_publisher<mt_interfaces::msg::Gripper>("gripper", 10);

        // subscription_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        //     "switch_val", 10, std::bind(&MinimalSubPub::topic_callback, this, _1));

        subscription_1 = this->create_subscription<std_msgs::msg::Int16>(
                "left_val", 10, std::bind(&MinimalSubPub::topic_callback1, this, _1));

        subscription_2 = this->create_subscription<std_msgs::msg::Int16>(
                "right_val", 10, std::bind(&MinimalSubPub::topic_callback2, this, _1));

        timer_ = this->create_wall_timer(
                1000ms, std::bind(&MinimalSubPub::timer_callback, this));
    }

private:
    // void topic_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
    // {
    //     left_gripper_val = msg->data[0];
    //     right_gripper_val = msg->data[1];
    // }

    void topic_callback1(const std_msgs::msg::Int16::SharedPtr msg1)
    {
        left_gripper_val = msg1->data;
    }

    void topic_callback2(const std_msgs::msg::Int16::SharedPtr msg2)
    {
        right_gripper_val = msg2->data;
    }

    void timer_callback()
    {
        // left gripper
        // if (left_gripper_val == 0)
        // {
        //     mt_interfaces::msg::Gripper left_gripper_msg;
        //     left_gripper_msg.side = 0;
        //     left_gripper_msg.actuation_distance = left_gripper_msg.CLOSE_TWO_FINGER;
        //     left_gripper_msg.actuation_speed = left_gripper_msg.IDEAL_GRIPPER_SPEED;
        //     left_gripper_msg.actuation_force = left_gripper_msg.IDEAL_GRIPPER_FORCE;
        //     publisher_->publish(left_gripper_msg);
        // }
        // else
        // {
        //     mt_interfaces::msg::Gripper left_gripper_msg;
        //     left_gripper_msg.side = 0;
        //     left_gripper_msg.actuation_distance = left_gripper_msg.OPEN_TWO_FINGER;
        //     publisher_->publish(left_gripper_msg);
        // }

        // right gripper
        // if (right_gripper_val == 0)
        // {
        //     mt_interfaces::msg::Gripper right_gripper_msg;
        //     right_gripper_msg.side = 1;
        //     right_gripper_msg.actuation_distance = right_gripper_msg.CLOSE_THREE_FINGER;
        //     right_gripper_msg.actuation_speed = right_gripper_msg.IDEAL_GRIPPER_SPEED;
        //     right_gripper_msg.actuation_force = right_gripper_msg.IDEAL_GRIPPER_FORCE;
        //     publisher_->publish(right_gripper_msg);
        // }
        // else
        // {
        //     mt_interfaces::msg::Gripper right_gripper_msg;
        //     right_gripper_msg.side = 1;
        //     right_gripper_msg.actuation_distance = right_gripper_msg.OPEN_THREE_FINGER;
        //     publisher_->publish(right_gripper_msg);
        // }

        // left_gripper
        if (left_gripper_val == 1)
        {
            mt_interfaces::msg::Gripper left_gripper_msg;
            if (left_status)
            {
                left_gripper_msg.side = 0;
                left_gripper_msg.actuation_distance = left_gripper_msg.CLOSE_TWO_FINGER;
                left_gripper_msg.actuation_speed = left_gripper_msg.IDEAL_GRIPPER_SPEED;
                left_gripper_msg.actuation_force = left_gripper_msg.IDEAL_GRIPPER_FORCE;
                left_status = 0;
            }

            else
            {
                left_gripper_msg.side = 0;
                left_gripper_msg.actuation_distance = left_gripper_msg.OPEN_TWO_FINGER;
                left_status = 1;
            }
            publisher_->publish(left_gripper_msg);
            left_gripper_val = 0;
        }

        if (right_gripper_val == 1)
        {
            mt_interfaces::msg::Gripper right_gripper_msg;
            if (right_status)
            {
                right_gripper_msg.side = 1;
                right_gripper_msg.actuation_distance = right_gripper_msg.CLOSE_THREE_FINGER;
                right_gripper_msg.actuation_speed = right_gripper_msg.IDEAL_GRIPPER_SPEED;
                right_gripper_msg.actuation_force = right_gripper_msg.IDEAL_GRIPPER_FORCE;
                right_status = 0;
            }

            else
            {
                right_gripper_msg.side = 1;
                right_gripper_msg.actuation_distance = right_gripper_msg.OPEN_THREE_FINGER;
                right_status = 1;
            }
            publisher_->publish(right_gripper_msg);
            right_gripper_val = 0;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
    // rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_1;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_2;
    rclcpp::Publisher<mt_interfaces::msg::Gripper>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int left_gripper_val, right_gripper_val;
    int left_status = 0;
    int right_status = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubPub>());
    rclcpp::shutdown();
    return 0;
}