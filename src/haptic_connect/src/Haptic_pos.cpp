#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <math.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <functional>
#include <memory>

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

        subscription_1 = this->create_subscription<std_msgs::msg::Float64MultiArray>("left_haptic_pose", 10, std::bind(&hapticcode::topic_callback_1, this, _1));
        subscription_2 = this->create_subscription<std_msgs::msg::Float64MultiArray>("right_haptic_pose", 10, std::bind(&hapticcode::topic_callback_2, this, _1));

        timer_1 = this->create_wall_timer(50ms, std::bind(&hapticcode::timer_callback1, this));
    }

private:
    double p4, p5, p6, p7;
    double p1_left, p2_left, p3_left, p1_right, p2_right, p3_right, posx_left, posy_left, posz_left, posx_right, posy_right, posz_right;
    double orienx_left, orieny_left, orienz_left, orienw_left, orienx_right, orieny_right, orienz_right, orienw_right;
    double orx_left, ory_left, orz_left, orw_left1, orw_left2, orx_right, ory_right, orz_right, orw_right1, orw_right2;
    double OldMinx = -0.065;
    double OldMiny = -0.148;  //-0.111;
    double OldMinz = -0.136;  //-0.078;
    double NewRangex = -1.56; // sign change
    double NewRangey = -1.3;  // sign change
    double NewRangez = 1.25;
    double OldRangex = 0.1627; // 0.137;
    double OldRangey = 0.296;  // 0.222744;
    double OldRangez = 0.298;  // 0.2;
    double NewMinx = 0.76;      // should be 0.76 or just change the sign and try for reverse movement, old value = -0.8, new = 0.98
    double NewMiny = 0.5;      // should be 0.65 or just change the sign and try for reverse movement, old value = -0.65 new = 0.65
    double NewMinz = -0.3;
    double r_left[3][3], rot_left[3][3], r_right[3][3], rot_right[3][3];
    double ninetydegx[3][3] = {0, 0, -1, 0, 1, 0, 1, 0, 0};
    int done = 0;
    double angles[6], angle_min[6], angle_max[6], new_angle[3];

    void topic_callback_1(const std_msgs::msg::Float64MultiArray::SharedPtr msg_1)
    {
        p1_left = msg_1->data[0];
        p2_left = msg_1->data[1];
        p3_left = msg_1->data[2];
        r_left[0][0] = msg_1->data[3];
        r_left[0][1] = msg_1->data[4];
        r_left[0][2] = msg_1->data[5];
        r_left[1][0] = msg_1->data[6];
        r_left[1][1] = msg_1->data[7];
        r_left[1][2] = msg_1->data[8];
        r_left[2][0] = msg_1->data[9];
        r_left[2][1] = msg_1->data[10];
        r_left[2][2] = msg_1->data[11];
    }

    void topic_callback_2(const std_msgs::msg::Float64MultiArray::SharedPtr msg_2)
    {
        p1_right = msg_2->data[0];
        p2_right = msg_2->data[1];
        p3_right = msg_2->data[2];
        r_right[0][0] = msg_2->data[3];
        r_right[0][1] = msg_2->data[4];
        r_right[0][2] = msg_2->data[5];
        r_right[1][0] = msg_2->data[6];
        r_right[1][1] = msg_2->data[7];
        r_right[1][2] = msg_2->data[8];
        r_right[2][0] = msg_2->data[9];
        r_right[2][1] = msg_2->data[10];
        r_right[2][2] = msg_2->data[11];
    }

    void timer_callback1()
    {
        // Converting rotation matrix to quaternion for left robot
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                rot_left[i][j] = 0;
                for (int k = 0; k < 3; k++)
                {
                    rot_left[i][j] += r_left[i][k] * ninetydegx[k][j];
                }
            }
        }

        orw_left1 = sqrt(1 + rot_left[0][0] + rot_left[1][1] + rot_left[2][2]);
        orw_left2 = sqrt(1 + rot_left[0][0] + rot_left[1][1] + rot_left[2][2]) / 2;
        orx_left = (rot_left[2][1] - rot_left[1][2]) / (4 * orw_left2);
        ory_left = (rot_left[0][2] - rot_left[2][0]) / (4 * orw_left2);
        orz_left = (rot_left[1][0] - rot_left[0][1]) / (4 * orw_left2);

        // Converting rotation matrix to quaternion for right robot
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                rot_right[i][j] = 0;
                for (int k = 0; k < 3; k++)
                {
                    rot_right[i][j] += r_right[i][k] * ninetydegx[k][j];
                }
            }
        }

        orw_right1 = sqrt(1 + rot_right[0][0] + rot_right[1][1] + rot_right[2][2]);
        orw_right2 = sqrt(1 + rot_right[0][0] + rot_right[1][1] + rot_right[2][2]) / 2;
        orx_right = (rot_right[2][1] - rot_right[1][2]) / (4 * orw_right2);
        ory_right = (rot_right[0][2] - rot_right[2][0]) / (4 * orw_right2);
        orz_right = (rot_right[1][0] - rot_right[0][1]) / (4 * orw_right2);

        // position for left arm
        posx_left = ((((p1_left - OldMinx) * NewRangex) / OldRangex) + NewMinx) - 0.13;
        posy_left = ((((p2_left - OldMiny) * NewRangey) / OldRangey) + NewMiny) + 0.43;
        posz_left = ((((p3_left - OldMinz) * NewRangez) / OldRangez) + NewMinz)- 0.1; //+ 0.11;

        // position for right arm
        posx_right = ((((p1_right - OldMinx) * NewRangex) / OldRangex) + NewMinx) - 0.35; // was 0.40
        posy_right = ((((p2_right - OldMiny) * NewRangey) / OldRangey) + NewMiny) - 0.18;
        posz_right = ((((p3_right - OldMinz) * NewRangez) / OldRangez) + NewMinz);// + 0.15;

        // Publishing left side
        geometry_msgs::msg::Pose targ_pose_left;
        targ_pose_left.position.x = posx_left;
        targ_pose_left.position.y = posy_left;
        targ_pose_left.position.z = posz_left;
        targ_pose_left.orientation.w = -orw_left2;
        targ_pose_left.orientation.x = orx_left;
        targ_pose_left.orientation.y = ory_left;
        targ_pose_left.orientation.z = -orz_left;

        // if subscription is not sending values
//        if (p1_left == 0.0)
//        {
//            targ_pose_left.position.x = NAN;
//            targ_pose_left.position.y = NAN;
//            targ_pose_left.position.z = NAN;
//            targ_pose_left.orientation.w = NAN;
//            targ_pose_left.orientation.x = NAN;
//            targ_pose_left.orientation.y = NAN;
//            targ_pose_left.orientation.z = NAN;
//        }

        publisher_1->publish(targ_pose_left);

        // Publishing right side
        geometry_msgs::msg::Pose targ_pose_right;
        targ_pose_right.position.x = posx_right;
        targ_pose_right.position.y = posy_right;
        targ_pose_right.position.z = posz_right;
        targ_pose_right.orientation.w = -orw_right1;
        targ_pose_right.orientation.x = orx_right;
        targ_pose_right.orientation.y = ory_right;
        targ_pose_right.orientation.z = -orz_right;

        // if subscription is not sending values
//        if (p1_right == 0.0)
//        {
//            targ_pose_right.position.x = NAN;
//            targ_pose_right.position.y = NAN;
//            targ_pose_right.position.z = NAN;
//            targ_pose_right.orientation.w = NAN;
//            targ_pose_right.orientation.x = NAN;
//            targ_pose_right.orientation.y = NAN;
//            targ_pose_right.orientation.z = NAN;
//        }
        publisher_2->publish(targ_pose_right);
    }

    rclcpp::TimerBase::SharedPtr timer_1;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_1;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_2;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_1;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_2;
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