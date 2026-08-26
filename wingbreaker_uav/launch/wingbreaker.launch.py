"""Full-stack launch: PX4 SITL + Gazebo + camera bridge + interceptor nodes.

Usage:
    ros2 launch wingbreaker_uav wingbreaker.launch.py
    ros2 launch wingbreaker_uav wingbreaker.launch.py intruder_mode:=static
    ros2 launch wingbreaker_uav wingbreaker.launch.py approval_mode:=llm
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, \
    RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PX4_DIR = os.path.expanduser('~/PX4-Autopilot')
UAV_DIR = os.path.expanduser('~/UAV_Project')
WORLD = 'default'
CAMERA_GZ_TOPIC = (
    f'/world/{WORLD}/model/zam_uav_v2/link/camera_link/sensor/camera/image')


def generate_launch_description():
    intruder_mode = LaunchConfiguration('intruder_mode')
    approval_mode = LaunchConfiguration('approval_mode')

    config = PathJoinSubstitution([
        FindPackageShare('wingbreaker_uav'), 'config', 'uav.yaml'])
    intruder_script = os.path.join(
        get_package_share_directory('wingbreaker_uav'),
        'scripts', 'intruder_sim.py')

    # --- simulation backend ---
    px4_env = {
        **os.environ,
        'PX4_SYS_AUTOSTART': '4031',
        'PX4_SIM_MODEL': 'gz_zam_uav_v2',
        'PX4_GZ_WORLD': WORLD,
        'PX4_GZ_MODEL_POSE': '0,0,0.3',
        # make sure OUR copies of the custom models win resolution order
        'GZ_SIM_RESOURCE_PATH':
            os.path.join(UAV_DIR, 'sim', 'models') + ':' +
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    }

    px4 = ExecuteProcess(
        cmd=['./build/px4_sitl_default/bin/px4'],
        cwd=PX4_DIR, env=px4_env, output='screen')

    intruder = ExecuteProcess(
        cmd=['python3', intruder_script,
             '--mode', intruder_mode, '--world', WORLD],
        output='screen')

    camera_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='camera_bridge',
        arguments=[f'{CAMERA_GZ_TOPIC}@sensor_msgs/msg/Image@gz.msgs.Image'],
        remappings=[(CAMERA_GZ_TOPIC, '/camera/image_raw')],
        output='screen')

    # --- gated application nodes ---
    gate = Node(
        package='wingbreaker_uav', executable='wait_for_topic',
        name='wait_for_camera',
        arguments=['/camera/image_raw', '180'],
        output='screen')

    flight = Node(package='wingbreaker_uav', executable='flight_node',
                  name='flight_node', parameters=[config], output='screen')
    safety = Node(package='wingbreaker_uav', executable='safety_node',
                  name='safety_node', parameters=[config], output='screen')
    detector = Node(package='wingbreaker_uav', executable='detector',
                    name='detector', parameters=[config], output='screen')
    engagement = Node(package='wingbreaker_uav', executable='engagement_node',
                      name='engagement_node', output='screen')
    brain = Node(package='wingbreaker_uav', executable='brain',
                 name='brain', parameters=[config], output='screen')
    dashboard = Node(package='wingbreaker_uav', executable='web_dashboard',
                     name='web_dashboard', parameters=[config], output='screen')
    llm = Node(package='wingbreaker_uav', executable='intercept_llm',
               name='intercept_llm', parameters=[config], output='screen',
               condition=IfCondition(LaunchConfiguration('run_llm')))

    app_nodes = [flight, safety, detector, engagement, brain, dashboard, llm]

    return LaunchDescription([
        DeclareLaunchArgument(
            'intruder_mode', default_value='flying',
            description='flying | static | none'),
        DeclareLaunchArgument(
            'approval_mode', default_value='human',
            description='human | llm | auto'),
        DeclareLaunchArgument(
            'run_llm', default_value='false',
            description='start the intercept_llm node (set true with '
                        'approval_mode:=llm)'),
        px4,
        intruder,
        camera_bridge,
        gate,
        RegisterEventHandler(OnProcessExit(target_action=gate,
                                           on_exit=app_nodes)),
    ])
