import asyncio
import threading
import math

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from wingbreaker_interfaces.action import FlyToGPS
from mavsdk import System


class FlightNode(Node):
    def __init__(self):
        super().__init__('flight_node')

        # asyncio loop in a background thread for MAVSDK
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

        self.drone = System()
        self.connected = False
        self.airborne = False   # have we taken off yet?

        # connect to the drone in the background
        asyncio.run_coroutine_threadsafe(self._connect(), self.loop)

        # the action server
        self.server = ActionServer(
            self, FlyToGPS, 'fly_to_gps', self.execute_callback)
        self.get_logger().info('Flight node starting - connecting to drone...')

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    async def _connect(self):
        await self.drone.connect(system_address="udpin://0.0.0.0:14540")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info('Drone connected!')
                break
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.get_logger().info('Position OK - ready to fly')
                self.connected = True
                break

    # ---------- action callback: brain sends a GPS goal ----------
    def execute_callback(self, goal_handle):
        lat = goal_handle.request.latitude
        lon = goal_handle.request.longitude
        alt = goal_handle.request.altitude
        self.get_logger().info(
            'Goal: fly to (%.6f, %.6f) alt %.1f' % (lat, lon, alt))

        future = asyncio.run_coroutine_threadsafe(
            self._fly_to(lat, lon, alt, goal_handle), self.loop)
        ok = future.result()   # wait for flight to finish

        goal_handle.succeed()
        result = FlyToGPS.Result()
        result.success = ok
        result.message = 'Arrived' if ok else 'Flight failed'
        return result

    async def _fly_to(self, lat, lon, alt, goal_handle):
        if not self.connected:
            self.get_logger().warn('Drone not connected yet')
            return False

        try:
            # arm + takeoff only on the first goal
            if not self.airborne:
                await self.drone.action.arm()
                await self.drone.action.set_takeoff_altitude(alt)
                await self.drone.action.takeoff()
                await asyncio.sleep(8)
                self.airborne = True

            # fly to the GPS location (yaw 0 = facing north)
            await self.drone.action.goto_location(lat, lon, alt, 0.0)

            # stream feedback: distance to target until close enough
            feedback = FlyToGPS.Feedback()
            while True:
                async for pos in self.drone.telemetry.position():
                    d = self._distance(
                        pos.latitude_deg, pos.longitude_deg, lat, lon)
                    feedback.distance_remaining = float(d)
                    goal_handle.publish_feedback(feedback)
                    break
                if d < 2.0:   # within 2 meters = arrived
                    break
                await asyncio.sleep(1.0)

            return True

        except Exception as e:
            self.get_logger().warn('Flight error: %s' % str(e))
            return False

    def _distance(self, lat1, lon1, lat2, lon2):
        # rough distance in meters between two GPS points
        R = 6371000.0
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = (math.sin(dlat/2)**2 +
             math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
             math.sin(dlon/2)**2)
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))


def main(args=None):
    rclpy.init(args=args)
    node = FlightNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()