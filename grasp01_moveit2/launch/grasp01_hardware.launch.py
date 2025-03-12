from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# MoveIt imports
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch
)

# For file/directory handling
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_robot_description(context):
    # Get package share directory
    grasp01_moveit2_pkg = get_package_share_directory("grasp01_moveit2")
    xacro_file = os.path.join(grasp01_moveit2_pkg, "config", "grasp01_description.urdf.xacro")
    initial_positions_file = os.path.join(grasp01_moveit2_pkg, "config", "initial_positions.yaml")

    # Verify the initial_positions_file exists
    if not os.path.exists(initial_positions_file):
        raise FileNotFoundError(f"Initial positions file not found: {initial_positions_file}")

    # Evaluate LaunchConfigurations with the provided context
    ifname = LaunchConfiguration("ifname").perform(context)
    cycle_time = LaunchConfiguration("cycle_time").perform(context)
    timeout = LaunchConfiguration("timeout").perform(context)

    # Create mappings dictionary with resolved values
    xacro_mappings = {
        "initial_positions_file": initial_positions_file,
        "ifname": ifname,
        "cycle_time": cycle_time,
        "timeout": timeout,
    }

    # Process the Xacro file
    robot_description_content = xacro.process_file(xacro_file, mappings=xacro_mappings).toxml()
    return {"robot_description": robot_description_content}

def generate_robot_state_publisher(context, *args, **kwargs):
    # Publish the URDF on /robot_description
    robot_description = generate_robot_description(context)
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

def generate_hardware_nodes(context, *args, **kwargs):
    """
    This function creates:
    1) robot_state_publisher node
    2) ros2_control_node (controller_manager)
    3) spawner for joint_state_broadcaster
    4) spawner for your grasp01_arm_controller
    """
    robot_description = generate_robot_description(context)
    grasp01_moveit2_pkg = get_package_share_directory("grasp01_moveit2")

    # 1) robot_state_publisher
    rsp_node = generate_robot_state_publisher(context)

    # 2) ros2_control_node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            os.path.join(grasp01_moveit2_pkg, "config", "ros2_controllers.yaml"),
        ],
        output="screen",
    )

    # 3) spawner for the joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 4) spawner for your main arm controller
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["grasp01_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return [rsp_node, ros2_control_node, joint_state_broadcaster_spawner, controller_spawner]

def generate_launch_description():
    # 1) Declare hardware-related launch arguments
    launch_args = [
        DeclareLaunchArgument("ifname", default_value="can0"),
        DeclareLaunchArgument("cycle_time", default_value="1"),
        DeclareLaunchArgument("timeout", default_value="0"),
    ]

    # 2) Build MoveIt config with explicit URDF and SRDF
    #    Also load moveit_controllers.yaml for controller definitions
    grasp01_moveit2_pkg = get_package_share_directory("grasp01_moveit2")
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="grasp01_description",
            package_name="grasp01_moveit2"
        )
        .robot_description(file_path="config/grasp01_description.urdf.xacro")
        .robot_description_semantic(file_path="config/grasp01_description.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")  # <--- LOADS CONTROLLERS
        .to_moveit_configs()
    )

    # 3) Launch the MoveIt move_group node
    move_group_launch = generate_move_group_launch(moveit_config)

    # 4) Launch MoveIt RViz
    rviz_launch = generate_moveit_rviz_launch(moveit_config)

    # 5) Hardware nodes (via an OpaqueFunction)
    hardware_action = OpaqueFunction(function=generate_hardware_nodes)

    # Combine everything into one LaunchDescription
    ld = LaunchDescription(launch_args)
    ld.add_action(move_group_launch)  # starts move_group node
    ld.add_action(rviz_launch)        # starts RViz with MoveIt plugin
    ld.add_action(hardware_action)    # starts hardware-related nodes

    return ld

if __name__ == "__main__":
    generate_launch_description()
