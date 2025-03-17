from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_grasp01_gazebo = get_package_share_directory('grasp01_gazebo')

    xacro_file = os.path.join(pkg_grasp01_gazebo, 'urdf', 'grasp01_gazebo.urdf.xacro')
   
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True
    }

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="grasp01_description",
            package_name="grasp01_moveit2"
        )
        .robot_description(xacro_file)
        .robot_description_semantic(file_path="config/grasp01_description.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )

    # Process Xacro from the grasp01_gazebo package
    robot_desc = Command(['xacro ', xacro_file])

    # Path to the empty world (updated to SDF for Ionic)
    world_file = os.path.join(pkg_grasp01_gazebo, 'worlds', 'empty.sdf')

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
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

    # Launch Gazebo Ionic with the custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_file]  # -r runs sim, -v 4 for verbose output
        }.items()
    )

    # Publish robot state
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'~/robot_description': robot_desc, 'use_sim_time': True},
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    planning_scene_monitor_parameters],
        output='screen'
    )

    ros2_controllers_path = os.path.join(pkg_grasp01_gazebo, 'config', 'ros2_gazebo_controllers.yaml')

    # Load controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            planning_scene_monitor_parameters,
            ros2_controllers_path,
            {"hardware_plugins": ["gz_ros2_control/GazeboSimSystem"]}
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output='screen'
    )

    # Spawn controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['grasp01_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    impedance_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['grasp01_impedance_control', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Spawn robot in Gazebo Ionic
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-name', 'grasp01_description'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        impedance_controller_spawner,
        spawn_entity,
        rviz_config_arg,
        move_group_node,
        rviz_node
    ])