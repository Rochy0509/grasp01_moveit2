from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    grasp01_subscriber = Node(
        package='demo_grasp01_moveit',
        executable='grasp01_subscriber',
        name='grasp01_subscriber',
        output='screen'
    )

    return LaunchDescription([
        grasp01_subscriber
    ])