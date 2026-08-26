"""Wait until a ROS 2 topic has at least one publisher, then exit 0.

Used by the launch file to gate application startup on the simulation being
ready (e.g. camera images flowing before the detector starts).
"""

import sys

import rclpy
from rclpy.node import Node


class WaitForTopic(Node):

    def __init__(self, topic, timeout_s):
        super().__init__('wait_for_topic')
        self.topic = topic
        self.deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_s
        self.get_logger().info(
            'Waiting for publishers on %s (timeout %.0fs)...'
            % (topic, timeout_s))

    def poll(self):
        if self.count_publishers(self.topic) > 0:
            self.get_logger().info('%s is live - continuing' % self.topic)
            raise SystemExit(0)
        now = self.get_clock().now().nanoseconds / 1e9
        if now > self.deadline:
            self.get_logger().error(
                'Timed out waiting for %s' % self.topic)
            raise SystemExit(1)


def main(args=None):
    topic = sys.argv[1] if len(sys.argv) > 1 else '/camera/image_raw'
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0
    rclpy.init(args=args)
    node = WaitForTopic(topic, timeout)
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=1.0)
            node.poll()
    except SystemExit as e:
        sys.exit(int(e.code))
    except KeyboardInterrupt:
        sys.exit(1)


if __name__ == '__main__':
    main()
