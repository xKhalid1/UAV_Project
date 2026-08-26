"""Safety node - battery monitor on a SECONDARY MAVSDK connection.

Publishes battery_level and safe_to_fly. Runs its own gateway on port 14541
so a stalled primary connection cannot hide battery state.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from wingbreaker_uav.drone_gateway import DroneGateway


class SafetyNode(Node):

    def __init__(self):
        super().__init__('safety_node')

        self.declare_parameter('system_address', 'udpin://0.0.0.0:14541')
        self.declare_parameter('low_threshold', 20.0)
        address = str(self.get_parameter('system_address').value)
        self.low_threshold = float(self.get_parameter('low_threshold').value)

        self.battery_pub = self.create_publisher(Float32, 'battery_level', 10)
        self.safe_pub = self.create_publisher(Bool, 'safe_to_fly', 10)

        self.last_safe = None
        self.gw = DroneGateway(system_address=address, name='safety')
        self.gw.start()

        self.timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info(
            'Safety node online - reading battery via %s' % address)

    def publish_status(self):
        pct = self.gw.battery_pct
        bat_msg = Float32()
        bat_msg.data = float(pct)
        self.battery_pub.publish(bat_msg)

        safe = pct < 0.0 or pct > self.low_threshold   # unknown battery = safe
        safe_msg = Bool()
        safe_msg.data = safe
        self.safe_pub.publish(safe_msg)

        if safe != self.last_safe:
            if safe:
                self.get_logger().info('Battery %.0f%% - SAFE' % max(pct, 0.0))
            else:
                self.get_logger().warn(
                    'Battery %.0f%% - LOW! Not safe to chase' % pct)
            self.last_safe = safe


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
