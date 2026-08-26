from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wingbreaker_uav',
            executable='flight_node',
            name='flight_node',
            output='screen'
        ),
        Node(
            package='wingbreaker_uav',
            executable='engagement_node',
            name='engagement_node',
            output='screen'
        ),
        Node(
            package='wingbreaker_uav',
            executable='safety_node',
            name='safety_node',
            output='screen'
        ),
        Node(
            package='wingbreaker_uav',
            executable='brain',
            name='brain',
            output='screen'
        ),
        Node(
            package='wingbreaker_uav',
            executable='detector',
            name='detector',
            output='screen'
        ),
    ])
