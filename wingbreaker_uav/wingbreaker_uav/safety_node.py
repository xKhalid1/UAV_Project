import asyncio
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32
from mavsdk import System


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # publishers: battery level and safe-to-fly flag
        self.battery_pub = self.create_publisher(Float32, 'battery_level', 10)
        self.safe_pub = self.create_publisher(Bool, 'safe_to_fly', 10)

        self.low_threshold = 20.0   # below this % = not safe to chase
        self.battery_pct = 100.0
        self.last_safe = None       # track safety changes for logging

        # asyncio loop in a background thread for MAVSDK
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

        self.drone = System()
        asyncio.run_coroutine_threadsafe(self._monitor_battery(), self.loop)

        # publish the safety status once per second
        self.timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info('Safety node online - reading real battery...')

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    async def _monitor_battery(self):
        await self.drone.connect(system_address="udpin://0.0.0.0:14541")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info('Safety: drone connected')
                break
        async for battery in self.drone.telemetry.battery():
            pct = battery.remaining_percent
            if pct <= 1.0:          # value is a 0-1 fraction
                pct = pct * 100.0
            self.battery_pct = pct

    def publish_status(self):
        # publish battery level (every second - the brain needs current data)
        bat_msg = Float32()
        bat_msg.data = float(self.battery_pct)
        self.battery_pub.publish(bat_msg)

        # publish safe-to-fly flag (every second)
        safe = self.battery_pct > self.low_threshold
        safe_msg = Bool()
        safe_msg.data = safe
        self.safe_pub.publish(safe_msg)

        # only LOG when the safe/unsafe status changes (keeps the terminal quiet)
        if safe != self.last_safe:
            if safe:
                self.get_logger().info(
                    'Battery %.0f%% - SAFE' % self.battery_pct)
            else:
                self.get_logger().warn(
                    'Battery %.0f%% - LOW! Not safe to chase' % self.battery_pct)
            self.last_safe = safe


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()