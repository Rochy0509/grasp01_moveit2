import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare hardware-related launch arguments required by your hardware bringup
    ifname_arg = DeclareLaunchArgument(
        "ifname",
        default_value="can0",
        description="Interface name for hardware communication"
    )
    cycle_time_arg = DeclareLaunchArgument(
        "cycle_time",
        default_value="1",
        description="Cycle time for hardware control"
    )
    timeout_arg = DeclareLaunchArgument(
        "timeout",
        default_value="0",
        description="Timeout value for hardware control"
    )

    # Get the package share directory for the grasp01 hardware launch file
    grasp01_pkg_share = get_package_share_directory("grasp01_moveit2")
    hardware_launch_file = os.path.join(grasp01_pkg_share, "launch", "grasp01_hardware.launch.py")

    # Include your hardware launch file which boots up the robot (state publisher, ros2_control, etc.)
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hardware_launch_file)
    )

    # Spawn the impedance controller similar to the Franka impedance launch.
    # Adjust the controller name ("joint_impedance_example_controller") if necessary to match your configuration.
    impedance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["grasp01_impedance_control", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Build the final launch description with the declared arguments,
    # the hardware bringup, and the impedance controller spawner.
    ld = LaunchDescription([
        ifname_arg,
        cycle_time_arg,
        timeout_arg,
    ])

    ld.add_action(hardware_launch)
    ld.add_action(impedance_controller_spawner)

    return ld

if __name__ == '__main__':
    generate_launch_description()
