"""
A launch file for running the motion planning python api tutorial
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    planning_scene_monitor_parameters = { "publish_planning_scene": True, "publish_geometry_updates": True, "publish_state_updates": True, 
                                         "publish_transforms_updates": True, "publish_robot_description":True, "publish_robot_description_semantic":True}

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="grasp01_description", package_name="grasp01_moveit2"
        )
        .robot_description(file_path="config/grasp01_description.urdf.xacro")
        .robot_description_semantic(file_path="config/grasp01_description.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")  # <--- LOADS CONTROLLERS
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .moveit_cpp(
            file_path=get_package_share_directory("grasp01_robotics_experiments")
            + "/config/config.yaml"
        )
        .to_moveit_configs()
    )
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="grasp01_hardware",
        description="ROS 2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )

    grasp01_moveit_py = DeclareLaunchArgument(
        "grasp01_moveit_py",
        default_value="grasp01_moveit_py",
        description="Python for grasp01 using moveit_py",
    )

    moveit_py_node = Node(
        name="moveit_py",
        package="grasp01_robotics_experiments",
        executable=LaunchConfiguration("grasp01_moveit_py"),
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("grasp01_moveit2"), "config", rviz_base]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            planning_scene_monitor_parameters,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # Load eye-to-hand calibration data from YAML
    eye_to_hand_yaml_path = os.path.join(
        get_package_share_directory("grasp01_robotics_experiments"),
        "config",
        "eye_to_hand.yaml"
    )
    
    # Create the static transform publisher for camera to robot
    with open(eye_to_hand_yaml_path, 'r') as file:
        calibration_data = yaml.safe_load(file)
        
    # Extract transform arguments from the loaded YAML
    if "static_transform_publisher_args" in calibration_data:
        # Use the pre-formatted arguments string
        tf_args = calibration_data["static_transform_publisher_args"].split()
    else:
        # Extract from the transforms structure
        transform = calibration_data["transforms"][0]["transform"]
        t = transform["translation"]
        r = transform["rotation"]
        parent_frame = calibration_data["transforms"][0]["header"]["frame_id"]
        child_frame = calibration_data["transforms"][0]["child_frame_id"]
        
        tf_args = [
            str(t["x"]), str(t["y"]), str(t["z"]),
            str(r["x"]), str(r["y"]), str(r["z"]), str(r["w"]),
            parent_frame, child_frame
        ]
    
    camera_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_to_robot_tf",
        arguments=tf_args,
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    planning_scene_monitor_parameters],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("grasp01_moveit2"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            planning_scene_monitor_parameters,
            ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    load_controllers = []
    for controller in [
        "grasp01_arm_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="log",
            )
        ]

    return LaunchDescription(
        [
            grasp01_moveit_py,
            ros2_control_hardware_type,
            rviz_config_arg,
            moveit_py_node,
            robot_state_publisher,
            ros2_control_node,
            rviz_node,
            camera_tf_node,  # Added camera to robot transform
            move_group_node,
        ]
        + load_controllers
    )