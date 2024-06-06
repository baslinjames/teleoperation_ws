// #include <rclcpp/rclcpp.hpp>
// #include <stdio.h>
// #include <math.h>
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include <std_msgs/msg/float64_multi_array.hpp>
// #include <chrono>
// #include <functional>
// #include <memory>

// using namespace std::chrono_literals;
// using std::placeholders::_1;

// class hapticcode : public rclcpp::Node
// {
// public:
//     hapticcode()
//         : Node("haptic_code"), count_(0)
//     {
//         publisher_1 = this->create_publisher<geometry_msgs::msg::Pose>("haptic_to_robot_pose_left", 10);
//         publisher_2 = this->create_publisher<geometry_msgs::msg::Pose>("haptic_to_robot_pose_right", 10);

//         subscription_1 = this->create_subscription<std_msgs::msg::Float64MultiArray>("left_haptic_pose", 10, std::bind(&hapticcode::topic_callback_1, this, _1));
//         subscription_2 = this->create_subscription<std_msgs::msg::Float64MultiArray>("right_haptic_pose", 10, std::bind(&hapticcode::topic_callback_2, this, _1));

//         timer_1 = this->create_wall_timer(50ms, std::bind(&hapticcode::timer_callback1, this));
//     }

// private:
//     double p4, p5, p6, p7;
//     double p1_left, p2_left, p3_left, p1_right, p2_right, p3_right, posx_left, posy_left, posz_left, posx_right, posy_right, posz_right;
//     double orienx_left, orieny_left, orienz_left, orienw_left, orienx_right, orieny_right, orienz_right, orienw_right;
//     double orx_left, ory_left, orz_left, orw_left1, orw_left2, orx_right, ory_right, orz_right, orw_right1, orw_right2;
//     double OldMinx = -0.065;
//     double OldMiny = -0.148;  //-0.111;
//     double OldMinz = -0.136;  //-0.078;
//     double NewRangex = -1.56; // sign change
//     double NewRangey = -1.3;  // sign change
//     double NewRangez = 1.25;
//     double OldRangex = 0.1627; // 0.137;
//     double OldRangey = 0.296;  // 0.222744;
//     double OldRangez = 0.298;  // 0.2;
//     double NewMinx = 0.9;      // should be 0.76 or just change the sign and try for reverse movement, old value = -0.8, new = 0.98
//     double NewMiny = 0.5;      // should be 0.65 or just change the sign and try for reverse movement, old value = -0.65 new = 0.65
//     double NewMinz = -0.3;
//     double r_left[3][3], rot_left[3][3], r_right[3][3], rot_right[3][3];
//     double ninetydegx[3][3] = {0, 0, -1, 0, 1, 0, 1, 0, 0};
//     int done = 0;
//     double angles[6], angle_min[6], angle_max[6], new_angle[3];

//     void topic_callback_1(const std_msgs::msg::Float64MultiArray::SharedPtr msg_1)
//     {
//         p1_left = msg_1->data[0];
//         p2_left = msg_1->data[1];
//         p3_left = msg_1->data[2];
//         r_left[0][0] = msg_1->data[3];
//         r_left[0][1] = msg_1->data[4];
//         r_left[0][2] = msg_1->data[5];
//         r_left[1][0] = msg_1->data[6];
//         r_left[1][1] = msg_1->data[7];
//         r_left[1][2] = msg_1->data[8];
//         r_left[2][0] = msg_1->data[9];
//         r_left[2][1] = msg_1->data[10];
//         r_left[2][2] = msg_1->data[11];
//     }

//     void topic_callback_2(const std_msgs::msg::Float64MultiArray::SharedPtr msg_2)
//     {
//         p1_right = msg_2->data[0];
//         p2_right = msg_2->data[1];
//         p3_right = msg_2->data[2];
//         r_right[0][0] = msg_2->data[3];
//         r_right[0][1] = msg_2->data[4];
//         r_right[0][2] = msg_2->data[5];
//         r_right[1][0] = msg_2->data[6];
//         r_right[1][1] = msg_2->data[7];
//         r_right[1][2] = msg_2->data[8];
//         r_right[2][0] = msg_2->data[9];
//         r_right[2][1] = msg_2->data[10];
//         r_right[2][2] = msg_2->data[11];
//     }

//     void timer_callback1()
//     {
//         // Converting rotation matrix to quaternion for left robot
//         for (int i = 0; i < 3; i++)
//         {
//             for (int j = 0; j < 3; j++)
//             {
//                 rot_left[i][j] = 0;
//                 for (int k = 0; k < 3; k++)
//                 {
//                     rot_left[i][j] += r_left[i][k] * ninetydegx[k][j];
//                 }
//             }
//         }

//         orw_left1 = sqrt(1 + rot_left[0][0] + rot_left[1][1] + rot_left[2][2]);
//         orw_left2 = sqrt(1 + rot_left[0][0] + rot_left[1][1] + rot_left[2][2]) / 2;
//         orx_left = (rot_left[2][1] - rot_left[1][2]) / (4 * orw_left2);
//         ory_left = (rot_left[0][2] - rot_left[2][0]) / (4 * orw_left2);
//         orz_left = (rot_left[1][0] - rot_left[0][1]) / (4 * orw_left2);

//         // Converting rotation matrix to quaternion for right robot
//         for (int i = 0; i < 3; i++)
//         {
//             for (int j = 0; j < 3; j++)
//             {
//                 rot_right[i][j] = 0;
//                 for (int k = 0; k < 3; k++)
//                 {
//                     rot_right[i][j] += r_right[i][k] * ninetydegx[k][j];
//                 }
//             }
//         }

//         orw_right1 = sqrt(1 + rot_right[0][0] + rot_right[1][1] + rot_right[2][2]);
//         orw_right2 = sqrt(1 + rot_right[0][0] + rot_right[1][1] + rot_right[2][2]) / 2;
//         orx_right = (rot_right[2][1] - rot_right[1][2]) / (4 * orw_right2);
//         ory_right = (rot_right[0][2] - rot_right[2][0]) / (4 * orw_right2);
//         orz_right = (rot_right[1][0] - rot_right[0][1]) / (4 * orw_right2);

//         // position for both arms
//         posx_left = ((((p1_left - OldMinx) * NewRangex) / OldRangex) + NewMinx) + 0.46;
//         posy_left = (((((p2_left - OldMiny) * NewRangey) / OldRangey) + NewMiny) + 0.53);
//         posz_left = ((((p3_left - OldMinz) * NewRangez) / OldRangez) + NewMinz) - 0.3;

//         posx_right = ((((p1_right - OldMinx) * NewRangex) / OldRangex) + NewMinx) + 0.57;
//         posy_right = (((((p2_right - OldMiny) * NewRangey) / OldRangey) + NewMiny) - 0.16);
//         posz_right = ((((p3_right - OldMinz) * NewRangez) / OldRangez) + NewMinz) - 0.3;

//         // Publishing left side
//         geometry_msgs::msg::Pose targ_pose_left;
//         targ_pose_left.position.x = posx_left;
//         targ_pose_left.position.y = posy_left;
//         targ_pose_left.position.z = posz_left;
//         targ_pose_left.orientation.w = -orw_left2;
//         targ_pose_left.orientation.x = orx_left;
//         targ_pose_left.orientation.y = ory_left;
//         targ_pose_left.orientation.z = -orz_left;

//         // if subscription is not sending values
//         if (p1_left == 0.0)
//         {
//             targ_pose_left.position.x = NAN;
//             targ_pose_left.position.y = NAN;
//             targ_pose_left.position.z = NAN;
//             targ_pose_left.orientation.w = NAN;
//             targ_pose_left.orientation.x = NAN;
//             targ_pose_left.orientation.y = NAN;
//             targ_pose_left.orientation.z = NAN;
//         }

//         publisher_1->publish(targ_pose_left);

//         // Publishing right side
//         geometry_msgs::msg::Pose targ_pose_right;
//         targ_pose_right.position.x = posx_right;
//         targ_pose_right.position.y = posy_right;
//         targ_pose_right.position.z = posz_right;
//         targ_pose_right.orientation.w = -orw_right1;
//         targ_pose_right.orientation.x = orx_right;
//         targ_pose_right.orientation.y = ory_right;
//         targ_pose_right.orientation.z = -orz_right;

//         // if subscription is not sending values
//         if (p1_right == 0.0)
//         {
//             targ_pose_right.position.x = NAN;
//             targ_pose_right.position.y = NAN;
//             targ_pose_right.position.z = NAN;
//             targ_pose_right.orientation.w = NAN;
//             targ_pose_right.orientation.x = NAN;
//             targ_pose_right.orientation.y = NAN;
//             targ_pose_right.orientation.z = NAN;
//         }
//         publisher_2->publish(targ_pose_right);
//     }

//     rclcpp::TimerBase::SharedPtr timer_1;
//     rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_1;
//     rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_2;
//     rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_1;
//     rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_2;
//     // rclcpp::Subscription<mt_interfaces::msg::HapticArmsPoses>::SharedPtr subscription_;
//     size_t count_;
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);

//     auto robotobj = std::make_shared<hapticcode>();
//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(robotobj);
//     executor.spin();

//     rclcpp::shutdown();

//     // happily exit
//     printf("\ndone.\n");
//     return 0;
// }

#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <math.h>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include "mt_interfaces/msg/haptic_arms_poses.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std::chrono_literals;
using std::placeholders::_1;
int left_status_button, right_status_button;

class hapticcode : public rclcpp::Node
{
public:
    hapticcode()
        : Node("haptic_code"), count_(0)
    {
        publisher_1 = this->create_publisher<geometry_msgs::msg::Pose>("haptic_to_robot_pose_left", 10);
        publisher_2 = this->create_publisher<geometry_msgs::msg::Pose>("haptic_to_robot_pose_right", 10);

        // subscription_ = this->create_subscription<mt_interfaces::msg::HapticArmsPoses>("haptic_arm_poses", 10, std::bind(&hapticcode::topic_callback, this, _1));

        subscription_1 = this->create_subscription<geometry_msgs::msg::Pose>("left_haptic_pose", 10, std::bind(&hapticcode::topic_callback_1, this, _1));
        subscription_2 = this->create_subscription<geometry_msgs::msg::Pose>("right_haptic_pose", 10, std::bind(&hapticcode::topic_callback_2, this, _1));
        subscription_3 = this->create_subscription<geometry_msgs::msg::PoseStamped>("left_endeff_pose", 10, std::bind(&hapticcode::topic_callback_3, this, _1));
        subscription_4 = this->create_subscription<geometry_msgs::msg::PoseStamped>("right_endeff_pose", 10, std::bind(&hapticcode::topic_callback_4, this, _1));
        subscription_5 = this->create_subscription<std_msgs::msg::Float64MultiArray>("gripper_button", 10, std::bind(&hapticcode::topic_callback_5, this, _1));

        timer_1 = this->create_wall_timer(50ms, std::bind(&hapticcode::timer_callback1, this));
    }

private:
    double p1_left, p2_left, p3_left, p1_right, p2_right, p3_right, posx_left, posy_left, posz_left, posx_right, posy_right, posz_right;
    double orienx_left, orieny_left, orienz_left, orienw_left, orienx_right, orieny_right, orienz_right, orienw_right;
    double orx_left, ory_left, orz_left, orw_left, orx_right, ory_right, orz_right, orw_right;
    double r_left[3][3], rot_left[3][3], r_right[3][3], rot_right[3][3];
    double ninetydegx[3][3] = {0, 0, -1, 0, 1, 0, 1, 0, 0};
    int done = 0;
    int subcount_left = 0, subcount_right = 0;
    bool count_left = false, count_right = false;
    double offset_x_left = 0, offset_y_left = 0, offset_z_left = 0;
    double offset_x_right = 0, offset_y_right = 0, offset_z_right = 0;
    Eigen::Quaterniond offsetQuaternion_left, targetQuaternion_left, offsetQuaternion_right, targetQuaternion_right;

    geometry_msgs::msg::Pose targ_pose_left;
    geometry_msgs::msg::Pose targ_pose_right;
    geometry_msgs::msg::Pose temp_pose_left;
    geometry_msgs::msg::Pose temp_pose_right;

    void topic_callback_1(const geometry_msgs::msg::Pose::SharedPtr msg_1)
    {
        p1_left = msg_1->position.x;
        p2_left = msg_1->position.y;
        p3_left = msg_1->position.z;
        orienx_left = msg_1->orientation.x;
        orieny_left = msg_1->orientation.y;
        orienz_left = msg_1->orientation.z;
        orienw_left = msg_1->orientation.w;
    }

    void topic_callback_2(const geometry_msgs::msg::Pose::SharedPtr msg_2)
    {
        p1_right = msg_2->position.x;
        p2_right = msg_2->position.y;
        p3_right = msg_2->position.z;
        orienx_right = msg_2->orientation.x;
        orieny_right = msg_2->orientation.y;
        orienz_right = msg_2->orientation.z;
        orienw_right = msg_2->orientation.w;
    }

    void topic_callback_3(const geometry_msgs::msg::PoseStamped::SharedPtr msg_3)
    {
        if (subcount_left == 0)
        {
            temp_pose_left.position.x = msg_3->pose.position.x;
            temp_pose_left.position.y = msg_3->pose.position.y;
            temp_pose_left.position.z = msg_3->pose.position.z;
            temp_pose_left.orientation.w = orienw_left;
            temp_pose_left.orientation.x = orienx_left;
            temp_pose_left.orientation.y = orieny_left;
            temp_pose_left.orientation.z = orienz_left;
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
            temp_pose_right.orientation.w = orienw_right;
            temp_pose_right.orientation.x = orienx_right;
            temp_pose_right.orientation.y = orieny_right;
            temp_pose_right.orientation.z = orienz_right;
            subcount_right = 1;
        }
    }

    void topic_callback_5(const std_msgs::msg::Float64MultiArray::SharedPtr msg_5)
    {
        left_status_button = msg_5->data[0];
        right_status_button = msg_5->data[1];
    }

    void timer_callback1()
    {
        // left orientation
        r_left[0][0] = 1 - (2 * orieny_left * orieny_left) - (2 * orienz_left * orienz_left);
        r_left[0][1] = (2 * orienx_left * orieny_left) - (2 * orienw_left * orienz_left);
        r_left[0][2] = (2 * orienx_left * orienz_left) + (2 * orienw_left * orieny_left);

        r_left[1][0] = (2 * orienx_left * orieny_left) + (2 * orienw_left * orienz_left);
        r_left[1][1] = 1 - (2 * orienx_left * orienx_left) - (2 * orienz_left * orienz_left);
        r_left[1][2] = (2 * orieny_left * orienz_left) - (2 * orienw_left * orienx_left);

        r_left[2][0] = (2 * orienx_left * orienz_left) - (2 * orienw_left * orieny_left);
        r_left[2][1] = (2 * orieny_left * orienz_left) + (2 * orienw_left * orienx_left);
        r_left[2][2] = 1 - (2 * orienx_left * orienx_left) - (2 * orieny_left * orieny_left);

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

        orw_left = sqrt(1 + rot_left[0][0] + rot_left[1][1] + rot_left[2][2]) / 2;
        orx_left = (rot_left[2][1] - rot_left[1][2]) / (4 * orw_left);
        ory_left = (rot_left[0][2] - rot_left[2][0]) / (4 * orw_left);
        orz_left = (rot_left[1][0] - rot_left[0][1]) / (4 * orw_left);

        // right orientation
        r_right[0][0] = 1 - (2 * orieny_right * orieny_right) - (2 * orienz_right * orienz_right);
        r_right[0][1] = (2 * orienx_right * orieny_right) - (2 * orienw_right * orienz_right);
        r_right[0][2] = (2 * orienx_right * orienz_right) + (2 * orienw_right * orieny_right);

        r_right[1][0] = (2 * orienx_right * orieny_right) + (2 * orienw_right * orienz_right);
        r_right[1][1] = 1 - (2 * orienx_right * orienx_right) - (2 * orienz_right * orienz_right);
        r_right[1][2] = (2 * orieny_right * orienz_right) - (2 * orienw_right * orienx_right);

        r_right[2][0] = (2 * orienx_right * orienz_right) - (2 * orienw_right * orieny_right);
        r_right[2][1] = (2 * orieny_right * orienz_right) + (2 * orienw_right * orienx_right);
        r_right[2][2] = 1 - (2 * orienx_right * orienx_right) - (2 * orieny_right * orieny_right);

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

        orw_right = sqrt(1 + rot_right[0][0] + rot_right[1][1] + rot_right[2][2]) / 2;
        orx_right = (rot_right[2][1] - rot_right[1][2]) / (4 * orw_right);
        ory_right = (rot_right[0][2] - rot_right[2][0]) / (4 * orw_right);
        orz_right = (rot_right[1][0] - rot_right[0][1]) / (4 * orw_right);

        //-------- Offset function and publishing for left arm -------------------------
        if (left_status_button)
        {
            // Calculating the new pose
            posx_left = (p1_left * 3) - offset_x_left;
            posy_left = (p2_left * 3) - offset_y_left;
            posz_left = (p3_left * 3) - offset_z_left;

            Eigen::Quaterniond existingQuaternion(orw_left, orx_left, ory_left, orz_left);
            targetQuaternion_left = offsetQuaternion_left * existingQuaternion;

            // Publishing left side
            targ_pose_left.position.x = posx_left;
            targ_pose_left.position.y = posy_left;
            targ_pose_left.position.z = posz_left;
            targ_pose_left.orientation.w = targetQuaternion_left.coeffs().w();
            targ_pose_left.orientation.x = targetQuaternion_left.coeffs().x();
            targ_pose_left.orientation.y = targetQuaternion_left.coeffs().y();
            targ_pose_left.orientation.z = targetQuaternion_left.coeffs().z();

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
                temp_pose_left.orientation.w = targetQuaternion_left.coeffs().w();
                temp_pose_left.orientation.x = targetQuaternion_left.coeffs().x();
                temp_pose_left.orientation.y = targetQuaternion_left.coeffs().y();
                temp_pose_left.orientation.z = targetQuaternion_left.coeffs().z();

                count_left = true;
            }
            publisher_1->publish(temp_pose_left);

            // Calculating the Offset
            offset_x_left = ((p1_left * 3) - temp_pose_left.position.x);
            offset_y_left = ((p2_left * 3) - temp_pose_left.position.y);
            offset_z_left = ((p3_left * 3) - temp_pose_left.position.z);

            Eigen::Quaterniond hapticMasterQuaternion(temp_pose_left.orientation.w, temp_pose_left.orientation.x, temp_pose_left.orientation.y, temp_pose_left.orientation.z);
            Eigen::Quaterniond robotArmQuaternion(orw_left, orx_left, ory_left, orz_left);

            offsetQuaternion_left = hapticMasterQuaternion * robotArmQuaternion.inverse();
        }

        //-------- Offset function and publishing for right arm -------------------------
        if (right_status_button)
        {
            // Calculating the new pose
            posx_right = (p1_right * 3) - offset_x_right;
            posy_right = (p2_right * 3) - offset_y_right;
            posz_right = (p3_right * 3) - offset_z_right;

            Eigen::Quaterniond existingQuaternion(orw_right, orx_right, ory_right, orz_right);
            targetQuaternion_right = offsetQuaternion_right * existingQuaternion;

            // Publishing right side
            targ_pose_right.position.x = posx_right;
            targ_pose_right.position.y = posy_right;
            targ_pose_right.position.z = posz_right;
            targ_pose_right.orientation.w = targetQuaternion_right.coeffs().w();
            targ_pose_right.orientation.x = targetQuaternion_right.coeffs().x();
            targ_pose_right.orientation.y = targetQuaternion_right.coeffs().y();
            targ_pose_right.orientation.z = targetQuaternion_right.coeffs().z();

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
                temp_pose_right.orientation.w = targetQuaternion_right.coeffs().w();
                temp_pose_right.orientation.x = targetQuaternion_right.coeffs().x();
                temp_pose_right.orientation.y = targetQuaternion_right.coeffs().y();
                temp_pose_right.orientation.z = targetQuaternion_right.coeffs().z();

                count_right = true;
            }
            publisher_2->publish(temp_pose_right);

            // Calculating the Offset
            offset_x_right = ((p1_right * 3) - temp_pose_right.position.x);
            offset_y_right = ((p2_right * 3) - temp_pose_right.position.y);
            offset_z_right = ((p3_right * 3) - temp_pose_right.position.z);

            Eigen::Quaterniond hapticMasterQuaternion(temp_pose_right.orientation.w, temp_pose_right.orientation.x, temp_pose_right.orientation.y, temp_pose_right.orientation.z);
            Eigen::Quaterniond robotArmQuaternion(orw_right, orx_right, ory_right, orz_right);

            offsetQuaternion_right = hapticMasterQuaternion * robotArmQuaternion.inverse();
        }
    }

    rclcpp::TimerBase::SharedPtr timer_1;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_1;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_2;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_1;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_2;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_3;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_4;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_5;
    // rclcpp::Subscription<mt_interfaces::msg::HapticArmsPoses>::SharedPtr subscription_;
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
