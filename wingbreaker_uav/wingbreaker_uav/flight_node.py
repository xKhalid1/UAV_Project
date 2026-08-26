"""Flight control node - FlyToGPS action server over the shared gateway.

Owns the PRIMARY MAVSDK connection (udpin://0.0.0.0:14540) and publishes
VehicleStatus telemetry for the rest of the stack.
"""

import threading

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from wingbreaker_interfaces.action import FlyToGPS
from wingbreaker_interfaces.msg import VehicleStatus

from wingbreaker_uav.drone_gateway import DroneGateway, distance_m


class FlightNode(Node):

    def __init__(self):
        super().__init__('flight_node')

        self.declare_parameter('system_address', 'udpin://0.0.0.0:14540')
        self.declare_parameter('status_rate_hz', 2.0)
        self.declare_parameter('arrival_radius_m', 40.0)
        address = str(self.get_parameter('system_address').value)
        rate = float(self.get_parameter('status_rate_hz').value)
        # fixed-wing L1 control loiters around targets - it will NOT converge
        # to a few meters; accept arrival at a realistic radius
        self.arrival_radius = float(self.get_parameter('arrival_radius_m').value)

        self.gw = DroneGateway(system_address=address, name='flight',
                               grpc_port=50051)
        self.gw.start()

        self.status_pub = self.create_publisher(VehicleStatus, 'vehicle_status', 10)
        self.status_timer = self.create_timer(1.0 / rate, self.publish_status)

        # ReentrantCallbackGroup so cancel requests are processed while a
        # goal callback is blocking - otherwise cancels queue behind it.
        self.server = ActionServer(self, FlyToGPS, 'fly_to_gps',
                                   self.execute_callback,
                                   callback_group=ReentrantCallbackGroup())
        # serializes actual flight execution: goals queue here while cancels
        # stay responsive (cancel handling does not need this lock)
        self._flight_lock = threading.Lock()
        self.get_logger().info(
            'Flight node starting - gateway connecting to %s ...' % address)

    # ---------- telemetry ----------
    def publish_status(self):
        gw = self.gw
        msg = VehicleStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.connected = gw.connected
        msg.armed = gw.armed
        msg.airborne = gw.in_air
        pos = gw.position
        if pos:
            msg.latitude_deg = pos['lat']
            msg.longitude_deg = pos['lon']
            msg.relative_altitude_m = float(pos['rel_alt'])
        else:
            msg.latitude_deg = 0.0
            msg.longitude_deg = 0.0
            msg.relative_altitude_m = 0.0
        msg.ground_speed_m_s = float(gw.ground_speed or 0.0)
        msg.heading_deg = float(gw.heading_deg or 0.0)
        msg.battery_percent = float(gw.battery_pct)
        msg.num_satellites = int(gw.num_sats)
        msg.gps_fix_type = 3 if gw.connected else 0
        self.status_pub.publish(msg)

    # ---------- action ----------
    def execute_callback(self, goal_handle):
        lat = goal_handle.request.latitude
        lon = goal_handle.request.longitude
        alt = goal_handle.request.altitude
        self.get_logger().info(
            'Goal: fly to (%.6f, %.6f) alt %.1f' % (lat, lon, alt))

        with self._flight_lock:
            return self._execute_flight(goal_handle, lat, lon, alt)

    def _execute_flight(self, goal_handle, lat, lon, alt):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = FlyToGPS.Result()
            result.success = False
            result.message = 'Canceled'
            return result

        ok = False
        try:
            if not self.gw.armed or not self.gw.in_air:
                self.get_logger().info('Takeoff needed - arming/taking off')
                self.gw.call(self.gw.arm_and_takeoff(alt), timeout=90.0)

            self.gw.call(self.gw.goto(lat, lon, alt), timeout=10.0)

            # stream feedback until within arrival radius (or canceled)
            feedback = FlyToGPS.Feedback()
            import time
            deadline = time.time() + 300.0
            while time.time() < deadline:
                if goal_handle.is_cancel_requested:
                    self.get_logger().warn('Goal canceled - aborting flight')
                    ok = False
                    break
                pos = self.gw.position
                if pos:
                    d = distance_m(pos['lat'], pos['lon'], lat, lon)
                    feedback.distance_remaining = float(d)
                    goal_handle.publish_feedback(feedback)
                    if d < self.arrival_radius:
                        ok = True
                        break
                time.sleep(1.0)
            if not ok and not (
                    goal_handle.is_cancel_requested):
                self.get_logger().warn('Arrival timeout - goal failed')
        except Exception as e:      # noqa: BLE001
            self.get_logger().warn('Flight error: %s' % e)
            ok = False

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = FlyToGPS.Result()
            result.success = False
            result.message = 'Canceled'
            return result

        goal_handle.succeed()
        result = FlyToGPS.Result()
        result.success = ok
        result.message = 'Arrived' if ok else 'Flight failed'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FlightNode()
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
