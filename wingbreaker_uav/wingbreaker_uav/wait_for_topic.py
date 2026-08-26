"""Wait until a ROS 2 topic actually delivers a message, then exit 0.

Used by the launch file to gate application startup on the simulation being
ready (e.g. camera images flowing before the detector starts).

Note: this deliberately waits for real DATA, not merely a publisher. The
ros_gz_bridge registers its ROS publisher at startup even when the gz side
has nothing to forward, so publisher-count checks pass instantly and hide
broken camera pipelines.
"""

import importlib
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class WaitForTopic(Node):

    def __init__(self, topic, timeout_s, msg_type):
        super().__init__('wait_for_topic')
        self.topic = topic
        self.deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_s
        self.received = False
        self.sub = self.create_subscription(
            msg_type, topic, self._on_msg, qos_profile_sensor_data)
        self.get_logger().info(
            'Waiting for data on %s (timeout %.0fs)...' % (topic, timeout_s))

    def _on_msg(self, msg):
        self.received = True

    def poll(self):
        if self.received:
            self.get_logger().info('%s is streaming - continuing' % self.topic)
            raise SystemExit(0)
        now = self.get_clock().now().nanoseconds / 1e9
        if now > self.deadline:
            self.get_logger().error(
                'Timed out waiting for data on %s' % self.topic)
            raise SystemExit(1)


def _load_msg_type(type_str):
    """Resolve 'pkg/msg/Name' to the message class."""
    pkg, _, name = type_str.split('/')
    module = importlib.import_module('%s.msg' % pkg)
    return getattr(module, name)


def main(args=None):
    topic = sys.argv[1] if len(sys.argv) > 1 else '/camera/image_raw'
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0
    type_str = sys.argv[3] if len(sys.argv) > 3 else 'sensor_msgs/msg/Image'
    msg_type = _load_msg_type(type_str)
    rclpy.init(args=args)
    node = WaitForTopic(topic, timeout, msg_type)
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.5)
            node.poll()
    except SystemExit as e:
        sys.exit(int(e.code))
    except KeyboardInterrupt:
        sys.exit(1)


if __name__ == '__main__':
    main()
