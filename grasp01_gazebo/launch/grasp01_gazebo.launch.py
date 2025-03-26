from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Get the package directory
    pkg_grasp01_gazebo = get_package_share_directory('grasp01_gazebo')

    # Define the xacro file for robot description
    xacro_file = os.path.join(pkg_grasp01_gazebo, 'urdf', 'grasp01_gazebo.urdf.xacro')
    robot_desc = Command(['xacro ', xacro_file])

    # Define the world file for Gazebo
    world_file = os.path.join(pkg_grasp01_gazebo, 'worlds', 'empty.world')

    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="grasp01_description",
            package_name="grasp01_moveit2"
        )
        .robot_description(file_path=xacro_file)
        .robot_description_semantic(file_path="config/grasp01_description.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Planning scene monitor parameters
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
        "use_sim_time": True  # Ensure MoveIt uses simulation time
    }

    # Declare RViz configuration argument
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("grasp01_moveit2"), "config", rviz_base]
    )

    # Include Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',  # Reduce log noise
            'pause': 'false',    # Start unpaused
        }.items()
    )

    # Launch robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen'
    )

    # Spawn controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    impedance_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['grasp01_impedance_control', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Spawn robot entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'grasp01_description', '-x', '0', '-y', '0', '-z', '1.2'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Launch MoveIt move_group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},  # Ensure MoveIt uses simulation time
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Launch RViz node
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
            {'use_sim_time': True},  # Ensure RViz uses simulation time
        ],
    )

    # Return the launch description
    return LaunchDescription([
        rviz_config_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        impedance_controller_spawner,
        move_group_node,
        rviz_node
    ])