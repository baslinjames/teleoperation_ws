#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <math.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std::chrono_literals;
using std::placeholders::_1;

class hapticcode : public rclcpp::Node
{
public:
    hapticcode()
        : Node("haptic_code"), count_(0)
    {
        publisher_1 = this->create_publisher<geometry_msgs::msg::Pose>("haptic_to_robot_pose_left", 10);
        publisher_2 = this->create_publisher<geometry_msgs::msg::Pose>("haptic_to_robot_pose_right", 10);

        timer_1 = this->create_wall_timer(50ms, std::bind(&hapticcode::timer_callback1, this));
        timer_2 = this->create_wall_timer(50ms, std::bind(&hapticcode::timer_callback2, this));

        subscription_1 = this->create_subscription<std_msgs::msg::Float64MultiArray>("left_haptic_pose", 20, std::bind(&hapticcode::topic_callback_1, this, _1));
        subscription_2 = this->create_subscription<std_msgs::msg::Float64MultiArray>("right_haptic_pose", 10, std::bind(&hapticcode::topic_callback_2, this, _1));
        subscription_3 = this->create_subscription<geometry_msgs::msg::PoseStamped>("left_endeff_pose", 10, std::bind(&hapticcode::topic_callback_3, this, _1));
        subscription_4 = this->create_subscription<geometry_msgs::msg::PoseStamped>("right_endeff_pose", 10, std::bind(&hapticcode::topic_callback_4, this, _1));
    }

private:
    double p1_left, p2_left, p3_left, p1_right, p2_right, p3_right, posx_left, posy_left, posz_left, posx_right, posy_right, posz_right;
    double orx_left, ory_left, orz_left, orw_left, orx_right, ory_right, orz_right, orw_right;
    int left_status_button, right_status_button;
    double offset_xl = 0, offset_yl = 0, offset_zl = 0, offset_xr = 0, offset_yr = 0, offset_zr = 0;
    int done = 0;
    bool count_left = false, count_right = false;
    int subcount_left = 0, subcount_right = 0;
    Eigen::Quaterniond offsetQuaternion_l, targetQuaternion_l, offsetQuaternion_r, targetQuaternion_r;

    geometry_msgs::msg::Pose targ_pose_left;
    geometry_msgs::msg::Pose targ_pose_right;
    geometry_msgs::msg::Pose temp_pose_left;
    geometry_msgs::msg::Pose temp_pose_right;

    void topic_callback_1(const std_msgs::msg::Float64MultiArray::SharedPtr msg_1)
    {
        left_status_button = msg_1->data[0];
        p1_left = -(msg_1->data[1]);
        p2_left = -(msg_1->data[2]);
        p3_left = msg_1->data[3];
        orw_left = msg_1->data[4];
        orx_left = msg_1->data[5];
        ory_left = msg_1->data[6];
        orz_left = msg_1->data[7];
    }

    void topic_callback_2(const std_msgs::msg::Float64MultiArray::SharedPtr msg_2)
    {
        right_status_button = msg_2->data[0];
        p1_right = -(msg_2->data[1]);
        p2_right = -(msg_2->data[2]);
        p3_right = msg_2->data[3];
        orw_right = msg_2->data[4];
        orx_right = msg_2->data[5];
        ory_right = msg_2->data[6];
        orz_right = msg_2->data[7];
    }

    void topic_callback_3(const geometry_msgs::msg::PoseStamped::SharedPtr msg_3)
    {
        if (subcount_left == 0)
        {
            temp_pose_left.position.x = msg_3->pose.position.x;
            temp_pose_left.position.y = msg_3->pose.position.y;
            temp_pose_left.position.z = msg_3->pose.position.z;
            temp_pose_left.orientation.w = msg_3->pose.orientation.w;
            temp_pose_left.orientation.x = msg_3->pose.orientation.x;
            temp_pose_left.orientation.y = msg_3->pose.orientation.y;
            temp_pose_left.orientation.z = msg_3->pose.orientation.z;
            subcount_left = 1;
        }
    }

    void topic_callback_4(const geometry_msgs::msg::PoseStamped::SharedPtr msg_4)
    {
        if (subcount_right == 0)
        {
            temp_pose_right.position.x = msg_4->pose.position.x;
            temp_pose_right.position.y = msg_4->pose.position.y;
            temp_pose_right.position.z = msg_4->pose.position.z;
            temp_pose_right.orientation.w = msg_4->pose.orientation.w;
            temp_pose_right.orientation.x = msg_4->pose.orientation.x;
            temp_pose_right.orientation.y = msg_4->pose.orientation.y;
            temp_pose_right.orientation.z = msg_4->pose.orientation.z;
            subcount_right = 1;
        }
    }

    void timer_callback1()
    {
        //------------------------------------------------------------------------------------------------
        // Changing workspace and publishing
        if (left_status_button)
        {
            // Calculating the new pose
            posx_left = (p1_left * 3) - offset_xl;
            posy_left = (p2_left * 3) - offset_yl;
            posz_left = (p3_left * 3) - offset_zl;

            Eigen::Quaterniond existingQuaternion(orw_left, orx_left, ory_left, orz_left);
            targetQuaternion_l = offsetQuaternion_l * existingQuaternion;

            // Publishing left side
            targ_pose_left.position.x = posx_left;
            targ_pose_left.position.y = posy_left;
            targ_pose_left.position.z = posz_left;
            targ_pose_left.orientation.w = targetQuaternion_l.coeffs().w();
            targ_pose_left.orientation.x = targetQuaternion_l.coeffs().x();
            targ_pose_left.orientation.y = targetQuaternion_l.coeffs().y();
            targ_pose_left.orientation.z = targetQuaternion_l.coeffs().z();

            count_left = false;

            publisher_1->publish(targ_pose_left);
        }
        else
        {
            if (!count_left)
            {
                // Storing previous pose
                temp_pose_left.position.x = posx_left;
                temp_pose_left.position.y = posy_left;
                temp_pose_left.position.z = posz_left;
                temp_pose_left.orientation.w = targetQuaternion_l.coeffs().w();
                temp_pose_left.orientation.x = targetQuaternion_l.coeffs().x();
                temp_pose_left.orientation.y = targetQuaternion_l.coeffs().y();
                temp_pose_left.orientation.z = targetQuaternion_l.coeffs().z();

                count_left = true;
            }
            publisher_1->publish(temp_pose_left);

            // Calculating the Offset
            offset_xl = ((p1_left * 3) - temp_pose_left.position.x);
            offset_yl = ((p2_left * 3) - temp_pose_left.position.y);
            offset_zl = ((p3_left * 3) - temp_pose_left.position.z);

            Eigen::Quaterniond hapticMasterQuaternion(temp_pose_left.orientation.w, temp_pose_left.orientation.x, temp_pose_left.orientation.y, temp_pose_left.orientation.z);
            Eigen::Quaterniond robotArmQuaternion(orw_left, orx_left, ory_left, orz_left);

            offsetQuaternion_l = hapticMasterQuaternion * robotArmQuaternion.inverse();
        }
    }

    void timer_callback2()
    {   
        //------------------------------------------------------------------------------------------------
        // Changing workspace and publishing
        if (right_status_button)
        {
            // Calculating the new pose
            posx_right = (p1_right * 3) - offset_xr;
            posy_right = (p2_right * 3) - offset_yr;
            posz_right = (p3_right * 3) - offset_zr;

            Eigen::Quaterniond existingQuaternion(orw_right, orx_right, ory_right, orz_right);
            targetQuaternion_r = offsetQuaternion_r * existingQuaternion;

            // Publishing left side
            targ_pose_right.position.x = posx_right;
            targ_pose_right.position.y = posy_right;
            targ_pose_right.position.z = posz_right;
            targ_pose_right.orientation.w = targetQuaternion_r.coeffs().w();
            targ_pose_right.orientation.x = targetQuaternion_r.coeffs().x();
            targ_pose_right.orientation.y = targetQuaternion_r.coeffs().y();
            targ_pose_right.orientation.z = targetQuaternion_r.coeffs().z();

            count_right = false;

            publisher_2->publish(targ_pose_right);
        }
        else
        {
            if (!count_right)
            {
                // Storing previous pose
                temp_pose_right.position.x = posx_right;
                temp_pose_right.position.y = posy_right;
                temp_pose_right.position.z = posz_right;
                temp_pose_right.orientation.w = targetQuaternion_r.coeffs().w();
                temp_pose_right.orientation.x = targetQuaternion_r.coeffs().x();
                temp_pose_right.orientation.y = targetQuaternion_r.coeffs().y();
                temp_pose_right.orientation.z = targetQuaternion_r.coeffs().z();

                count_right = true;
            }
            publisher_2->publish(temp_pose_right);

            // Calculating the Offset
            offset_xr = ((p1_right * 3) - temp_pose_right.position.x);
            offset_yr = ((p2_right * 3) - temp_pose_right.position.y);
            offset_zr = ((p3_right * 3) - temp_pose_right.position.z);

            Eigen::Quaterniond hapticMasterQuaternion(temp_pose_right.orientation.w, temp_pose_right.orientation.x, temp_pose_right.orientation.y, temp_pose_right.orientation.z);
            Eigen::Quaterniond robotArmQuaternion(orw_right, orx_right, ory_right, orz_right);

            offsetQuaternion_r = hapticMasterQuaternion * robotArmQuaternion.inverse();
        }
    }

    rclcpp::TimerBase::SharedPtr timer_1;
    rclcpp::TimerBase::SharedPtr timer_2;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_1;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_2;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_1;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_2;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_3;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_4;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto robotobj = std::make_shared<hapticcode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(robotobj);
    executor.spin();

    rclcpp::shutdown();

    // happily exit
    printf("\ndone.\n");
    return 0;
}