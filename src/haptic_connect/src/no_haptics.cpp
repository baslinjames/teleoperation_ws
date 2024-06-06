#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <memory>
#include <haptic_connect/drdc.h>
#include "haptic_connect/dhdc.h"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>

using std::placeholders::_1;

int main(int argc, char *argv[])
{
    // Optimize console output performance.
    std::cout.rdbuf()->pubsetbuf(nullptr, 512);
    std::cout << std::nounitbuf;
    std::ios_base::sync_with_stdio(false);

    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("Haptic_Initialization");

    // Publisher
    auto left_haptic_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("left_haptic_pose", 10);
    auto right_haptic_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("right_haptic_pose", 10);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]()
                                { executor.spin(); });

    rclcpp::Rate loop_rate(100);

    // initializing variables
    double p1_left, p2_left, p3_left, p4_left, p5_left, p6_left, p7_left;
    double p1_right, p2_right, p3_right, p4_right, p5_right, p6_right, p7_right;
    double orx_left, ory_left, orz_left, orw_left1, orw_left2, orx_right, ory_right, orz_right, orw_right1, orw_right2;
    double r_left[3][3], rot_left[3][3], r_right[3][3], rot_right[3][3];
    double ninetydegy[3][3] = {0, 0, -1, 0, 1, 0, 1, 0, 0};
    int left_status_button, right_status_button;
    int done = 0;

    // Haptic joystick initialization
    printf("Force Dimension - Automatic Initialization %s\n", dhdGetSDKVersionStr());
    printf("Copyright (C) 2001-2022 Force Dimension\n");
    printf("All Rights Reserved.\n\n");

    // required to change asynchronous operation mode
    dhdEnableExpertMode();

    // open the left haptic master
    if (drdOpenID(0) < 0)
    {
        printf("error: cannot open device (%s)\n", dhdErrorGetLastStr());
        dhdSleep(2.0);
        return -1;
    }

    // print out device identifier
    if (!drdIsSupported(0))
    {
        printf("unsupported device\n");
        printf("exiting...\n");
        dhdSleep(2.0);
        drdClose(0);
        return -1;
    }
    printf("%s haptic device detected\n\n", dhdGetSystemName(0));

    // Perform automatic initialization of the device if required.
    if (!drdIsInitialized(0) && drdAutoInit(0) < 0)
    {
        std::cout << "error: failed to initialize device (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }
    else if (drdStart(0) < 0)
    {
        std::cout << "error: failed to start robotic regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // Move to the center of the workspace.
    double positionCenter[DHD_MAX_DOF] = {};
    if (drdMoveTo(positionCenter, true, 0) < 0)
    {
        std::cout << "error: failed to move the device (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    // stop regulation (and leave force enabled)
    drdStop(true, 0);
    //
    //    // open the right haptic master
    //    if (drdOpenID(1) < 0)
    //    {
    //        printf("error: cannot open device (%s)\n", dhdErrorGetLastStr());
    //        dhdSleep(2.0);
    //        return -1;
    //    }
    //
    //    // print out device identifier
    //    if (!drdIsSupported(1))
    //    {
    //        printf("unsupported device\n");
    //        printf("exiting...\n");
    //        dhdSleep(2.0);
    //        drdClose(1);
    //        return -1;
    //    }
    //    printf("%s haptic device detected\n\n", dhdGetSystemName(1));
    //
    //    // perform auto-initialization
    //    printf("initializing...\r");
    //    fflush(stdout);
    //    if (drdAutoInit(1) < 0)
    //    {
    //        printf("error: auto-initialization failed (%s)\n", dhdErrorGetLastStr());
    //        drdClose(1);
    //        dhdSleep(2.0);
    //        return -1;
    //    }
    //
    //    // report success
    //    printf("devices successfully initialized\n\n");
    //
    //    // stop regulation (and leave force enabled)
    //    drdStop(true,1);

    // declaring variables to publish
    std_msgs::msg::Float64MultiArray left_haptic;
    std_msgs::msg::Float64MultiArray right_haptic;

    // haptic loop
    while (rclcpp::ok() && !done)
    {
        left_haptic.data.clear();
        right_haptic.data.clear();

        // // apply zero force for left haptic master
        if (dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0) < DHD_NO_ERROR)
        {
            printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
            done = 1;
        }

        // apply zero force for right haptic master
        //        if (dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1) < DHD_NO_ERROR)
        //        {
        //            printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
        //            done = 1;
        //        }

        // position and orientation from left haptic master
        if (drdGetPositionAndOrientation(&p1_left, &p2_left, &p3_left, &p4_left, &p5_left, &p6_left, &p7_left, r_left, 0) <
            0)
        {
            printf("error: cannot read position and orientation (%s)\n", dhdErrorGetLastStr());
            done = 1;
        }

        // position and orientation from right haptic master
        //        if (drdGetPositionAndOrientation(&p1_right, &p2_right, &p3_right, &p4_right, &p5_right, &p6_right,
        //        &p7_right, r_right, 1) < 0)
        //        {
        //            printf("error: cannot read position and orientation (%s)\n", dhdErrorGetLastStr());
        //            done = 1;
        //        }

        // Converting rotation matrix to quaternion for left robot
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                rot_left[i][j] = 0;
                for (int k = 0; k < 3; k++)
                {
                    rot_left[i][j] += r_left[i][k] * ninetydegy[k][j];
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
                    rot_right[i][j] += r_right[i][k] * ninetydegy[k][j];
                }
            }
        }

        orw_right1 = sqrt(1 + rot_right[0][0] + rot_right[1][1] + rot_right[2][2]);
        orw_right2 = sqrt(1 + rot_right[0][0] + rot_right[1][1] + rot_right[2][2]) / 2;
        orx_right = (rot_right[2][1] - rot_right[1][2]) / (4 * orw_right2);
        ory_right = (rot_right[0][2] - rot_right[2][0]) / (4 * orw_right2);
        orz_right = (rot_right[1][0] - rot_right[0][1]) / (4 * orw_right2);

        // Status of button on left and right haptic master
        left_status_button = dhdGetButton(0, 0);
        // right_status_button = dhdGetButton(0, 1);

        // publish left side if button pressed
        // if(left_status_button)
        // {
        left_haptic.data.push_back(left_status_button);
        left_haptic.data.push_back(p1_left);
        left_haptic.data.push_back(p2_left);
        left_haptic.data.push_back(p3_left);
        left_haptic.data.push_back(orw_left2);
        left_haptic.data.push_back(orx_left);
        left_haptic.data.push_back(ory_left);
        left_haptic.data.push_back(-orz_left);

        left_haptic_pub->publish(left_haptic);
        // }
        // else
        // {
        // left_haptic.data.push_back(NAN);
        // left_haptic.data.push_back(NAN);
        // left_haptic.data.push_back(NAN);
        // left_haptic.data.push_back(NAN);
        // left_haptic.data.push_back(NAN);
        // left_haptic.data.push_back(NAN);
        // left_haptic.data.push_back(NAN);
        // left_haptic.data.push_back(NAN);
        // left_haptic.data.push_back(NAN);
        // left_haptic.data.push_back(NAN);
        // left_haptic.data.push_back(NAN);
        // left_haptic.data.push_back(NAN);
        // left_haptic_pub->publish(left_haptic);
        // }

        // publish right side if button pressed
        //        if(right_status_button)
        //        {
        //            right_haptic.data.push_back(p1_right);
        //            right_haptic.data.push_back(p2_right);
        //            right_haptic.data.push_back(p3_right);
        //            right_haptic.data.push_back(r_right[0][0]);
        //            right_haptic.data.push_back(r_right[0][1]);
        //            right_haptic.data.push_back(r_right[0][2]);
        //            right_haptic.data.push_back(r_right[1][0]);
        //            right_haptic.data.push_back(r_right[1][1]);
        //            right_haptic.data.push_back(r_right[1][2]);
        //            right_haptic.data.push_back(r_right[2][0]);
        //            right_haptic.data.push_back(r_right[2][1]);
        //            right_haptic.data.push_back(r_right[2][2]);
        //            right_haptic_pub->publish(right_haptic);
        //        }
        //        else
        //        {
        //            right_haptic.data.push_back(NAN);
        //            right_haptic.data.push_back(NAN);
        //            right_haptic.data.push_back(NAN);
        //            right_haptic.data.push_back(NAN);
        //            right_haptic.data.push_back(NAN);
        //            right_haptic.data.push_back(NAN);
        //            right_haptic.data.push_back(NAN);
        //            right_haptic.data.push_back(NAN);
        //            right_haptic.data.push_back(NAN);
        //            right_haptic.data.push_back(NAN);
        //            right_haptic.data.push_back(NAN);
        //            right_haptic.data.push_back(NAN);
        //            right_haptic_pub->publish(right_haptic);
        //        }

        loop_rate.sleep();
    }

    rclcpp::shutdown();

    // close the connection
    printf("cleaning up...                                                           \n");
    drdClose(0);
    //    drdClose(1);

    // happily exit
    printf("\ndone.\n");
    return 0;
}