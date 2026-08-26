import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger


class EngagementNode(Node):
    def __init__(self):
        super().__init__('engagement_node')
        self.authorized = False   # kept for later, not required in auto mode

        self.auth_srv = self.create_service(
            Trigger, 'authorize', self.authorize_callback)
        self.fire_srv = self.create_service(
            Trigger, 'fire', self.fire_callback)

        self.get_logger().info('Engagement node ready (auto-fire mode)')

    def authorize_callback(self, request, response):
        self.authorized = True
        self.get_logger().warn('*** FIRING AUTHORIZED BY OPERATOR ***')
        response.success = True
        response.message = 'Fire authorized'
        return response

    def fire_callback(self, request, response):
        # AUTO-FIRE: no authorization required (autonomous engagement)
        self.get_logger().warn('>>> FIRING - intruder taken down (SIMULATED) <<<')
        response.success = True
        response.message = 'Intruder engaged'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = EngagementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()