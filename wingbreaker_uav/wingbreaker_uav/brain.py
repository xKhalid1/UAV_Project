import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from wingbreaker_interfaces.action import FlyToGPS
from wingbreaker_interfaces.msg import IntruderDetection
from std_srvs.srv import Trigger
from std_msgs.msg import Bool


class Brain(Node):
    def __init__(self):
        super().__init__('brain')
        self.cb_group = ReentrantCallbackGroup()

        # action client -> flight node
        self.fly_client = ActionClient(
            self, FlyToGPS, 'fly_to_gps', callback_group=self.cb_group)
        # service client -> engagement node
        self.fire_client = self.create_client(
            Trigger, 'fire', callback_group=self.cb_group)

        # subscriber -> detector
        self.sub = self.create_subscription(
            IntruderDetection, 'detections', self.on_detection, 10,
            callback_group=self.cb_group)
        # subscriber -> safety node
        self.safe_to_fly = True
        self.safety_sub = self.create_subscription(
            Bool, 'safe_to_fly', self.on_safety, 10,
            callback_group=self.cb_group)

        self.waypoints = [
            (31.5010, 34.5000, 65.0),
            (31.5010, 34.5010, 65.0),
            (31.5000, 34.5010, 65.0),
            (31.5000, 34.5000, 65.0),
        ]
        self.current_wp = 0
        self.state = 'PATROL'
        self.current_goal_handle = None
        self.busy = False
        self.cooldown_until = 0.0

        self.timer = self.create_timer(
            2.0, self.patrol_tick, callback_group=self.cb_group)
        self.get_logger().info('Brain online - state: PATROL')

    # ---------- SAFETY ----------
    def on_safety(self, msg):
        self.safe_to_fly = msg.data

    # ---------- PATROL ----------
    def patrol_tick(self):
        if self.state != 'PATROL' or self.busy:
            return
        lat, lon, alt = self.waypoints[self.current_wp]
        self.get_logger().info('PATROL: flying to waypoint %d' % self.current_wp)
        self.send_goal(lat, lon, alt, is_patrol=True)

    # ---------- DETECTION -> chase ----------
    def on_detection(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if now < self.cooldown_until:
            return
        if not self.safe_to_fly:
            self.get_logger().warn('Intruder seen but battery LOW - not chasing')
            return
        if msg.confidence >= 0.85 and self.state == 'PATROL':
            self.get_logger().warn(
                '>>> INTRUDER conf=%.2f - abandoning patrol, CHASING'
                % msg.confidence)
            self.state = 'CHASE'
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
            self.send_goal(msg.latitude, msg.longitude, msg.altitude,
                           is_patrol=False)

    # ---------- send flight goal ----------
    def send_goal(self, lat, lon, alt, is_patrol):
        self.busy = True
        goal = FlyToGPS.Goal()
        goal.latitude = lat
        goal.longitude = lon
        goal.altitude = alt
        self.fly_client.wait_for_server()
        future = self.fly_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self.goal_response(f, is_patrol))

    def goal_response(self, future, is_patrol):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.busy = False
            return
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self.reached(f, is_patrol))

    def reached(self, future, is_patrol):
        self.busy = False
        self.current_goal_handle = None

        if self.state == 'CHASE' and not is_patrol:
            self.get_logger().warn('>>> Reached intruder - FIRING')
            self.fire_at_target()
        elif is_patrol and self.state == 'PATROL':
            self.get_logger().info('Reached waypoint %d' % self.current_wp)
            self.current_wp = (self.current_wp + 1) % len(self.waypoints)

    # ---------- FIRE ----------
    def fire_at_target(self):
        if not self.fire_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Engagement node unavailable')
            self.back_to_patrol()
            return
        req = Trigger.Request()
        future = self.fire_client.call_async(req)
        future.add_done_callback(self.fire_response)

    def fire_response(self, future):
        response = future.result()
        self.get_logger().warn('>>> ENGAGEMENT: %s' % response.message)
        self.back_to_patrol()

    def back_to_patrol(self):
        now = self.get_clock().now().nanoseconds / 1e9
        self.cooldown_until = now + 10.0
        self.state = 'PATROL'
        self.get_logger().info('Target engaged - resuming PATROL (10s cooldown)')


def main(args=None):
    rclpy.init(args=args)
    node = Brain()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()