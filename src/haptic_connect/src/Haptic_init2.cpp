#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <memory>
#include <haptic_connect/drdc.h>
#include "haptic_connect/dhdc.h"
#include <std_msgs/msg/float64_multi_array.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("Haptic_Initialization");

  // Publisher
  auto left_haptic_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("left_haptic_pose", 10);
  auto right_haptic_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("right_haptic_pose", 10);

  rclcpp::Rate loop_rate(100);

  // initializing variables
  double p1_left, p2_left, p3_left, p4_left, p5_left, p6_left, p7_left;
  double p1_right, p2_right, p3_right, p4_right, p5_right, p6_right, p7_right;
  double orx_left, ory_left, orz_left, orw_left1, orw_left2, orx_right, ory_right, orz_right, orw_right1, orw_right2;
  double r_left[3][3], rot_left[3][3], r_right[3][3], rot_right[3][3];
  double ninetydegx[3][3] = { 0, 0, -1, 0, 1, 0, 1, 0, 0 };
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

  // perform auto-initialization
  printf("initializing...\r");
  fflush(stdout);
  if (drdAutoInit(0) < 0)
  {
    printf("error: auto-initialization failed (%s)\n", dhdErrorGetLastStr());
    drdClose(0);
    dhdSleep(2.0);
    return -1;
  }

  // stop regulation (and leave force enabled)
  drdStop(true, 0);
  //
  // open the right haptic master
  if (drdOpenID(1) < 0)
  {
    printf("error: cannot open device (%s)\n", dhdErrorGetLastStr());
    dhdSleep(2.0);
    return -1;
  }

  // print out device identifier
  if (!drdIsSupported(1))
  {
    printf("unsupported device\n");
    printf("exiting...\n");
    dhdSleep(2.0);
    drdClose(1);
    return -1;
  }
  printf("%s haptic device detected\n\n", dhdGetSystemName(1));

  // perform auto-initialization
  printf("initializing...\r");
  fflush(stdout);
  if (drdAutoInit(1) < 0)
  {
    printf("error: auto-initialization failed (%s)\n", dhdErrorGetLastStr());
    drdClose(1);
    dhdSleep(2.0);
    return -1;
  }

  // report success
  printf("devices successfully initialized\n\n");

  // stop regulation (and leave force enabled)
  drdStop(true, 1);

  // declaring variables to publish
  std_msgs::msg::Float64MultiArray left_haptic;
  std_msgs::msg::Float64MultiArray right_haptic;

  // haptic loop
  while (rclcpp::ok() && !done)
  {
    left_haptic.data.clear();
    right_haptic.data.clear();

    // apply zero force for left haptic master
    if (dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0) < DHD_NO_ERROR)
    {
      printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
      done = 1;
    }

    // apply zero force for right haptic master
    if (dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1) < DHD_NO_ERROR)
    {
      printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
      done = 1;
    }

    // position and orientation from left haptic master
    if (drdGetPositionAndOrientation(&p1_left, &p2_left, &p3_left, &p4_left, &p5_left, &p6_left, &p7_left, r_left, 0) <
        0)
    {
      printf("error: cannot read position and orientation (%s)\n", dhdErrorGetLastStr());
      done = 1;
    }

    // position and orientation from right haptic master
    if (drdGetPositionAndOrientation(&p1_right, &p2_right, &p3_right, &p4_right, &p5_right, &p6_right, &p7_right,
                                     r_right, 1) < 0)
    {
      printf("error: cannot read position and orientation (%s)\n", dhdErrorGetLastStr());
      done = 1;
    }

    // Status of button on left and right haptic master
    left_status_button = dhdGetButton(0, 0);
    right_status_button = dhdGetButton(0, 1);

    // publish left side if button pressed
    if (left_status_button)
    {
      // left_haptic.data.push_back(left_status_button);
      left_haptic.data.push_back(p1_left);
      left_haptic.data.push_back(p2_left);
      left_haptic.data.push_back(p3_left);
      left_haptic.data.push_back(r_left[0][0]);
      left_haptic.data.push_back(r_left[0][1]);
      left_haptic.data.push_back(r_left[0][2]);
      left_haptic.data.push_back(r_left[1][0]);
      left_haptic.data.push_back(r_left[1][1]);
      left_haptic.data.push_back(r_left[1][2]);
      left_haptic.data.push_back(r_left[2][0]);
      left_haptic.data.push_back(r_left[2][1]);
      left_haptic.data.push_back(r_left[2][2]);

      left_haptic_pub->publish(left_haptic);
    }
    else
    {
      left_haptic.data.push_back(NAN);
      left_haptic.data.push_back(NAN);
      left_haptic.data.push_back(NAN);
      left_haptic.data.push_back(NAN);
      left_haptic.data.push_back(NAN);
      left_haptic.data.push_back(NAN);
      left_haptic.data.push_back(NAN);
      left_haptic.data.push_back(NAN);
      left_haptic.data.push_back(NAN);
      left_haptic.data.push_back(NAN);
      left_haptic.data.push_back(NAN);
      left_haptic.data.push_back(NAN);
      left_haptic_pub->publish(left_haptic);
    }

    // publish right side if button pressed
    if (right_status_button)
    {
      right_haptic.data.push_back(p1_right);
      right_haptic.data.push_back(p2_right);
      right_haptic.data.push_back(p3_right);
      right_haptic.data.push_back(r_right[0][0]);
      right_haptic.data.push_back(r_right[0][1]);
      right_haptic.data.push_back(r_right[0][2]);
      right_haptic.data.push_back(r_right[1][0]);
      right_haptic.data.push_back(r_right[1][1]);
      right_haptic.data.push_back(r_right[1][2]);
      right_haptic.data.push_back(r_right[2][0]);
      right_haptic.data.push_back(r_right[2][1]);
      right_haptic.data.push_back(r_right[2][2]);
      right_haptic_pub->publish(right_haptic);
    }
    else
    {
      right_haptic.data.push_back(NAN);
      right_haptic.data.push_back(NAN);
      right_haptic.data.push_back(NAN);
      right_haptic.data.push_back(NAN);
      right_haptic.data.push_back(NAN);
      right_haptic.data.push_back(NAN);
      right_haptic.data.push_back(NAN);
      right_haptic.data.push_back(NAN);
      right_haptic.data.push_back(NAN);
      right_haptic.data.push_back(NAN);
      right_haptic.data.push_back(NAN);
      right_haptic.data.push_back(NAN);
      right_haptic_pub->publish(right_haptic);
    }
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  // close the connection
  printf("cleaning up...                                                           \n");
  drdClose(0);
  drdClose(1);

  // happily exit
  printf("\ndone.\n");
  return 0;
}