"""Pi camera mode launch.

Runs the Wingbreaker stack without PX4 SITL / Gazebo / camera_bridge /
detector. The Raspberry Pi provides the camera stream and detections via
HTTP; pi_bridge republishes them as /camera/image_raw and detections.

Usage:
    ros2 launch wingbreaker_uav pi_mode.launch.py pi_host:=192.168.1.42
    ros2 launch wingbreaker_uav pi_mode.launch.py pi_host:=192.168.1.42 pi_port:=5000
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pi_host = LaunchConfiguration('pi_host')
    pi_port = LaunchConfiguration('pi_port')

    config = PathJoinSubstitution([
        FindPackageShare('wingbreaker_uav'), 'config', 'pi_mode.yaml'])

    gate = Node(
        package='wingbreaker_uav', executable='wait_for_topic',
        name='wait_for_camera',
        arguments=['/camera/image_raw', '180', 'sensor_msgs/msg/Image'],
        output='screen')

    pi_bridge = Node(
        package='wingbreaker_uav', executable='pi_bridge',
        name='pi_bridge',
        parameters=[config, {'pi_host': pi_host, 'pi_port': pi_port}],
        output='screen')

    flight = Node(package='wingbreaker_uav', executable='flight_node',
                  name='flight_node', parameters=[config], output='screen')
    safety = Node(package='wingbreaker_uav', executable='safety_node',
                  name='safety_node', parameters=[config], output='screen')
    engagement = Node(package='wingbreaker_uav', executable='engagement_node',
                      name='engagement_node', output='screen')
    brain = Node(package='wingbreaker_uav', executable='brain',
                 name='brain', parameters=[config], output='screen')
    dashboard = Node(package='wingbreaker_uav', executable='web_dashboard',
                     name='web_dashboard', parameters=[config], output='screen')

    app_nodes = [pi_bridge, flight, safety, engagement, brain, dashboard]

    return LaunchDescription([
        DeclareLaunchArgument(
            'pi_host', default_value='192.168.1.100',
            description='Raspberry Pi IP or hostname serving /stream and /detections'),
        DeclareLaunchArgument(
            'pi_port', default_value='5000',
            description='Raspberry Pi HTTP port'),
        pi_bridge,
        gate,
        RegisterEventHandler(OnProcessExit(target_action=gate,
                                           on_exit=app_nodes)),
    ])
