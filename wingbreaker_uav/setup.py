from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wingbreaker_uav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), ['config/uav.yaml', 'config/pi_mode.yaml']),
        (os.path.join('share', package_name, 'scripts'),
            ['scripts/novnc_bridge.py', 'scripts/intruder_sim.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zam',
    maintainer_email='zamalsh77@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'flight_node = wingbreaker_uav.flight_node:main',
            'detector = wingbreaker_uav.detector:main',
            'brain = wingbreaker_uav.brain:main',
            'engagement_node = wingbreaker_uav.engagement_node:main',
            'safety_node = wingbreaker_uav.safety_node:main',
            'intercept_llm = wingbreaker_uav.intercept_llm:main',
            'pi_bridge = wingbreaker_uav.pi_bridge:main',
            'web_dashboard = wingbreaker_uav.web_dashboard:main',
            'wait_for_topic = wingbreaker_uav.wait_for_topic:main',
        ],
    },
)
