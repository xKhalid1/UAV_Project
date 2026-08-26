import random

import rclpy
from rclpy.node import Node

from wingbreaker_interfaces.msg import IntruderDetection


class Detector(Node):
    def __init__(self):
        super().__init__('detector')
        self.pub = self.create_publisher(IntruderDetection, 'detections', 10)
        self.timer = self.create_timer(60.0, self.scan)   # once per minute
        self.base_lat = 31.5000
        self.base_lon = 34.5000
        self.get_logger().info('Detector online - scanning airspace...')

    def scan(self):
        msg = IntruderDetection()
        msg.latitude = self.base_lat + random.uniform(-0.005, 0.005)
        msg.longitude = self.base_lon + random.uniform(-0.005, 0.005)
        msg.altitude = 65.0
        msg.confidence = 0.95
        msg.threat_level = 'HIGH'
        self.pub.publish(msg)
        self.get_logger().info(
            'INTRUDER at (%.5f, %.5f) conf=%.2f [%s]'
            % (msg.latitude, msg.longitude, msg.confidence, msg.threat_level))


def main(args=None):
    rclpy.init(args=args)
    node = Detector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()