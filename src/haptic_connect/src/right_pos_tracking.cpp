#define BOOST_BIND_NO_PLACEHOLDERS
// ROS
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <moveit_servo/servo.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std::chrono_literals;
using std::placeholders::_1;
geometry_msgs::msg::Pose p_right;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.pose_tracking_demo");
double v[7];
bool flag = false;

// Class for monitoring status of moveit_servo
class StatusMonitor
{
public:
    StatusMonitor(const rclcpp::Node::SharedPtr &node, const std::string &topic)
    {
        sub_ = node->create_subscription<std_msgs::msg::Int8>(topic, rclcpp::SystemDefaultsQoS(),
                                                              [this](const std_msgs::msg::Int8::ConstSharedPtr msg)
                                                              {
                                                                  return statusCB(msg);
                                                              });
    }

private:
    void statusCB(const std_msgs::msg::Int8::ConstSharedPtr &msg)
    {
        moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
        if (latest_status != status_)
        {
            status_ = latest_status;
            const auto &status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
            RCLCPP_INFO_STREAM(LOGGER, "Servo status: " << status_str);
        }
    }

    moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_;
};

class robotservo : public rclcpp::Node
{
public:
    robotservo() : Node("Haptic_pose")
    {
        subscription_1 = this->create_subscription<geometry_msgs::msg::Pose>("haptic_to_robot_pose_right",
                                                                             10, std::bind(&robotservo::topic_callback1, this, _1));
    }

private:
    void topic_callback1(const geometry_msgs::msg::Pose::SharedPtr msgg) {
        v[0] = msgg->position.x;
        v[1] = msgg->position.y;
        v[2] = msgg->position.z;
        v[3] = msgg->orientation.w;
        v[4] = msgg->orientation.x;
        v[5] = msgg->orientation.y;
        v[6] = msgg->orientation.z;
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_1;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("right_pose_tracking_demo");

    auto hapobj = std::make_shared<robotservo>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(hapobj);
    std::thread executor_thread([&executor]()
                                { executor.spin(); });

    auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node, LOGGER);

    if (servo_parameters == nullptr)
    {
        RCLCPP_FATAL(LOGGER, "Could not get servo parameters!");
        exit(EXIT_FAILURE);
    }

    // Load the planning scene monitor
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
    planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
    if (!planning_scene_monitor->getPlanningScene())
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Error in setting up the PlanningSceneMonitor.");
        exit(EXIT_FAILURE);
    }

    planning_scene_monitor->providePlanningSceneService();
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startWorldGeometryMonitor(
            planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
            planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
            false /* skip octomap monitor */);
    planning_scene_monitor->startStateMonitor(servo_parameters->joint_topic);
    planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

    // Wait for Planning Scene Monitor to setup
    if (!planning_scene_monitor->waitForCurrentRobotState(node->now(), 5.0 /* seconds */))
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Error waiting for current robot state in PlanningSceneMonitor.");
        exit(EXIT_FAILURE);
    }

    // const std::string PLANNING_GROUP = "right_arm";
    // moveit::planning_interface::MoveGroupInterface move_group_interface(node, PLANNING_GROUP);
    // std::vector<std::string> joint_names = move_group_interface.getJointNames();
//    std::vector<double> joint_values = {0.0,-0.785,0.0,-2.356,0.0,1.571,0.785};
//    std::vector<double> joint_values = {-0.367,-0.083,0.954,-2.039,0.044,2.124,0.34};
    // std::vector<double> joint_values = {0.719,-0.168,-0.201,-2.251,-0.055,2.095,0.34};
    // move_group_interface.setJointValueTarget(joint_names, joint_values);
    // move_group_interface.setMaxVelocityScalingFactor(0.3);
    // move_group_interface.move();

    rclcpp::sleep_for(std::chrono::seconds(4));

    // Create the pose tracker
    moveit_servo::PoseTracking tracker(node, servo_parameters, planning_scene_monitor);

    // Make a publisher for sending pose commands
    auto target_pose_pub =
            node->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", rclcpp::SystemDefaultsQoS());

    // Subscribe to servo status (and log it when it changes)
    StatusMonitor status_monitor(node, servo_parameters->status_topic);

    Eigen::Vector3d lin_tol{0.000001, 0.000001, 0.000001};
    double rot_tol = 0.01;

    // Get the current EE transform
    geometry_msgs::msg::TransformStamped current_ee_tf;
    tracker.getCommandFrameTransform(current_ee_tf);

    // Convert it to a Pose
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = current_ee_tf.header.frame_id;

    // Run the pose tracking in a new thread
    std::thread move_to_pose_thread([&tracker, &lin_tol, &rot_tol]
                                    {
                                        moveit_servo::PoseTrackingStatusCode tracking_status =
                                                tracker.moveToPose(lin_tol, rot_tol, 100000.0  /*target pose timeout*/ );
                                        RCLCPP_INFO_STREAM(LOGGER, "Pose tracker exited with status: "
                                                << moveit_servo::POSE_TRACKING_STATUS_CODE_MAP.at(tracking_status)); });

    rclcpp::WallRate loop_rate_1(200);

    while (rclcpp::ok())
    {
        target_pose.pose.position.x = v[0];
        target_pose.pose.position.y = v[1];
        target_pose.pose.position.z = v[2];
        target_pose.pose.orientation.w = v[3];
        target_pose.pose.orientation.x = v[4];
        target_pose.pose.orientation.y = v[5];
        target_pose.pose.orientation.z = v[6];

        target_pose.header.stamp = node->now();
        target_pose_pub->publish(target_pose);

        loop_rate_1.sleep();
    }

    // Make sure the tracker is stopped and clean up
    move_to_pose_thread.join();

    // Kill executor thread before shutdown
    executor.cancel();
    executor_thread.join();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}