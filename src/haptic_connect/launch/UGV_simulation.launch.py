import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            Shutdown)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    # Get parameters for the Servo node
    left_servo_yaml = load_yaml(
        "robot_config", "config/left_panda_simulated_config_pose_tracking.yaml"
    )
    left_servo_params = {"moveit_servo": left_servo_yaml}

    right_servo_yaml = load_yaml(
        "robot_config", "config/right_panda_simulated_config_pose_tracking.yaml"
    )
    right_servo_params = {"moveit_servo": right_servo_yaml}

    # Get parameters for pose tracking
    left_pose_tracking_yaml = load_yaml("robot_config", "config/left_pose_tracking_settings.yaml")
    left_pose_tracking_params = {"moveit_servo": left_pose_tracking_yaml}

    right_pose_tracking_yaml = load_yaml("robot_config", "config/right_pose_tracking_settings.yaml")
    right_pose_tracking_params = {"moveit_servo": right_pose_tracking_yaml}

    # planning_context
    # For simulated robots
    robot_description_config = load_file(
        "robot_config", "config/ugv_system.urdf"
    )
    robot_description = {"robot_description": robot_description_config}

    robot_description_semantic_config =load_file(
        "robot_config", "config/ugv_system_single.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        'robot_config', 'config/kinematics.yaml'
    )

    left_joint_limits_yaml = {
        "robot_description_planning": load_yaml(
            "robot_config", "config/left_joint_limits.yaml"
        )
    }

    right_joint_limits_yaml = {
        "robot_description_planning": load_yaml(
            "robot_config", "config/right_joint_limits.yaml"
        )
    }

    # Planning Functionality
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        'franka_moveit_config', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    left_moveit_controllers_yaml = load_yaml(
        "robot_config", "config/left_moveit_controllers.yaml"
    )
    right_moveit_controllers_yaml = load_yaml(
        "robot_config", "config/right_moveit_controllers.yaml"
    )
    left_moveit_controllers = {
        "moveit_simple_controller_manager": left_moveit_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    right_moveit_controllers = {
        "moveit_simple_controller_manager": right_moveit_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Start the actual move_group node/action server
    left_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            left_moveit_controllers,
            planning_scene_monitor_parameters,
            left_servo_params,
            left_pose_tracking_params,
        ],
    )

    right_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            right_moveit_controllers,
            planning_scene_monitor_parameters,
            right_servo_params,
            right_pose_tracking_params,
        ],
    )

    # RViz
    rviz_config_file = (
            get_package_share_directory("robot_config") + "/config/moveit_config.rviz"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            left_servo_params,
            right_servo_params,
            left_pose_tracking_params,
            right_pose_tracking_params,
            left_joint_limits_yaml,
            right_joint_limits_yaml,
        ],
    )

    # Haptic Initialization
    haptic_init = Node(
        package="haptic_connect",
        executable="Haptic_init_UGV",
        output="screen",
    )

    end_effector_pose = Node(
        package="haptic_connect",
        executable="endeff_pose_UGV",
        output="screen",
        parameters=[robot_description, robot_description_semantic],
    )

    haptic_pos = TimerAction(
        period=15.0,
        actions=[Node(
            package="haptic_connect",
            executable="Haptic_pose_UGV",
            output="screen",
        )],
    )

    left_pose_tracking = TimerAction(
        period=22.0,
        actions=[Node(
            package="haptic_connect",
            executable="pose_tracking_leftrobotUGV",
            remappings=[('target_pose', 'left_arm/target_pose')],
            output="screen",
            parameters=[left_servo_params, robot_description, robot_description_semantic, left_pose_tracking_params,
                        left_joint_limits_yaml],
        )],
    )

    right_pose_tracking = TimerAction(
        period=22.0,
        actions=[Node(
            package="haptic_connect",
            executable="pose_tracking_rightrobotUGV",
            remappings=[('target_pose', 'right_arm/target_pose')],
            output="screen",
            parameters=[right_servo_params, robot_description, robot_description_semantic, left_pose_tracking_params,
                        right_joint_limits_yaml],
        )],
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory('robot_config'),
        'config',
        'UGV_simulation_controller.yaml',
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_path],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        on_exit=Shutdown(),
    )

    # Load controllers
    load_controllers = []
    for controller in ['left_arm_controller', 'right_arm_controller', 'joint_state_broadcaster']:
        load_controllers += [
            ExecuteProcess(
                cmd=['ros2 run controller_manager spawner.py {}'.format(controller)],
                shell=True,
                output='screen',
            )
        ]

    return LaunchDescription(
        [
            rviz_node,
            robot_state_publisher,
            left_move_group_node,
            right_move_group_node,
            ros2_control_node,
            haptic_init,
            haptic_pos,
            end_effector_pose,
            left_pose_tracking,
            right_pose_tracking,
        ]
        + load_controllers)
