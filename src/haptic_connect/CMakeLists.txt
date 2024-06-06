cmake_minimum_required(VERSION 3.5)
project(haptic_connect)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(std_msgs)
find_package(geometry_msgs)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_ros_planning_interface)
find_package(moveit_servo)
find_package(moveit_msgs)
find_package(moveit)
find_package(mt_interfaces)
find_package(franka_msgs)
find_package(rviz_visual_tools)
find_package(serial_driver)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(Haptic_pos src/Haptic_pos.cpp)
target_include_directories(Haptic_pos PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

add_executable(Haptic_init src/Haptic_init.cpp)
target_include_directories(Haptic_init PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(Haptic_init
        /home/meditel/ICRA_ws/src/haptic_connect/include/haptic_connect/libdrd.so.3)

add_executable(Haptic_init2 src/Haptic_init2.cpp)
target_include_directories(Haptic_init2 PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(Haptic_init2
        /home/meditel/ICRA_ws/src/haptic_connect/include/haptic_connect/libdrd.so.3)

add_executable(Haptic_init_UGV src/Haptic_init_UGV.cpp)
target_include_directories(Haptic_init_UGV PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(Haptic_init_UGV
        /home/meditel/ICRA_ws/src/haptic_connect/include/haptic_connect/libdrd.so.3)

add_executable(left_pos_tracking src/left_pos_tracking.cpp)
target_include_directories(left_pos_tracking PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

add_executable(right_pos_tracking src/right_pos_tracking.cpp)
target_include_directories(right_pos_tracking PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

add_executable(Haptic_pose_UGV src/Haptic_pose_UGV.cpp)
target_include_directories(Haptic_pose_UGV PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# add_executable(pose_tracking_leftrobotUGV src/pose_tracking_leftrobotUGV.cpp)
# target_include_directories(pose_tracking_leftrobotUGV PUBLIC
#         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#         $<INSTALL_INTERFACE:include>)

# add_executable(pose_tracking_rightrobotUGV src/pose_tracking_rightrobotUGV.cpp)
# target_include_directories(pose_tracking_rightrobotUGV PUBLIC
#         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#         $<INSTALL_INTERFACE:include>)

add_executable(end_effpose src/end_effpose.cpp)
target_include_directories(end_effpose PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

add_executable(end_eff_pose_left_right src/end_eff_pose_left_right.cpp)
target_include_directories(end_eff_pose_left_right PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

add_executable(endeff_pose_UGV src/endeff_pose_UGV.cpp)
target_include_directories(endeff_pose_UGV PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

add_executable(left_right_gripper src/left_right_gripper.cpp)
target_include_directories(left_right_gripper PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

add_executable(gripper_sub src/gripper_sub.cpp)
target_include_directories(gripper_sub PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

add_executable(external_force src/external_force.cpp)
target_include_directories(external_force PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(external_force /usr/lib64/libfranka.so)

add_executable(dynamic_pose src/dynamic_pose.cpp)
target_include_directories(dynamic_pose PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

add_executable(normal_pose_tracking src/normal_pose_tracking.cpp)
target_include_directories(normal_pose_tracking PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

add_executable(FTsensor src/FTsensor.cpp)
target_include_directories(FTsensor PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

add_executable(no_haptics src/no_haptics.cpp)
target_include_directories(no_haptics PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
        
target_link_libraries(no_haptics
        /home/meditel/ICRA_ws/src/haptic_connect/include/haptic_connect/libdrd.so.3)

ament_target_dependencies(Haptic_pos rclcpp geometry_msgs std_msgs)
ament_target_dependencies(Haptic_init rclcpp geometry_msgs std_msgs Eigen3)
ament_target_dependencies(Haptic_init2 rclcpp geometry_msgs std_msgs)
ament_target_dependencies(left_pos_tracking rclcpp geometry_msgs std_msgs moveit moveit_ros_planning_interface moveit moveit_msgs moveit_servo)
# ament_target_dependencies(pose_tracking_leftrobotUGV rclcpp geometry_msgs std_msgs moveit moveit_ros_planning_interface moveit moveit_msgs moveit_servo)
# ament_target_dependencies(pose_tracking_rightrobotUGV rclcpp geometry_msgs std_msgs moveit moveit_ros_planning_interface moveit moveit_msgs moveit_servo)
ament_target_dependencies(right_pos_tracking rclcpp geometry_msgs std_msgs moveit moveit_ros_planning_interface moveit moveit_msgs moveit_servo)
ament_target_dependencies(Haptic_pose_UGV rclcpp geometry_msgs std_msgs Eigen3 mt_interfaces)
ament_target_dependencies(end_effpose rclcpp geometry_msgs moveit moveit_msgs moveit_ros_planning_interface)
ament_target_dependencies(end_eff_pose_left_right rclcpp geometry_msgs moveit moveit_msgs moveit_ros_planning_interface)
ament_target_dependencies(endeff_pose_UGV rclcpp geometry_msgs moveit moveit_msgs moveit_ros_planning_interface)
ament_target_dependencies(left_right_gripper rclcpp rclcpp_action rclcpp_components control_msgs franka_msgs sensor_msgs mt_interfaces)
ament_target_dependencies(gripper_sub rclcpp std_msgs mt_interfaces)
ament_target_dependencies(external_force rclcpp)
ament_target_dependencies(dynamic_pose rclcpp geometry_msgs std_msgs moveit moveit_msgs moveit_ros_planning_interface)
ament_target_dependencies(normal_pose_tracking rclcpp geometry_msgs std_msgs moveit moveit_ros_planning_interface moveit moveit_msgs moveit_servo)
ament_target_dependencies(FTsensor rclcpp std_msgs serial_driver)
ament_target_dependencies(no_haptics rclcpp geometry_msgs std_msgs Eigen3)
ament_target_dependencies(Haptic_init_UGV rclcpp geometry_msgs std_msgs )


install(TARGETS
        Haptic_pos
        Haptic_init
        Haptic_init2
        left_pos_tracking
        right_pos_tracking
        Haptic_pose_UGV
        # pose_tracking_leftrobotUGV
        # pose_tracking_rightrobotUGV
        end_effpose
        end_eff_pose_left_right
        left_right_gripper
        gripper_sub
        external_force
        dynamic_pose
        normal_pose_tracking
        FTsensor
        no_haptics
        Haptic_init_UGV
        endeff_pose_UGV
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
