from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start temperature node
        Node(
            package='turtlebot3_flywheels_and_temp',
            executable='amg8833_publisher',
            output='screen',
            name='temperature_node',
        ),
        # Start flywheel node
        Node(
            package='turtlebot3_flywheels_and_temp',
            executable='flywheel_node',
            output='screen',
            name='flywheel_node',
        ),
    ])
