"""Optional launch for the QGC-in-browser noVNC view.

    ros2 launch wingbreaker_uav qgc_novnc.launch.py

Runs novnc_bridge.py (plain script): headless QGC under Xvfb + x11vnc +
websockify/noVNC on port 6080. Nothing else depends on this.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    script = os.path.join(
        get_package_share_directory('wingbreaker_uav'),
        'scripts', 'novnc_bridge.py')
    return LaunchDescription([
        ExecuteProcess(cmd=['python3', script], output='screen'),
    ])
