"""Flight control node - FlyToGPS action server over the shared gateway.

Owns the PRIMARY MAVSDK connection (udpin://0.0.0.0:14540) and publishes
VehicleStatus telemetry for the rest of the stack.
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from wingbreaker_interfaces.action import FlyToGPS
from wingbreaker_interfaces.msg import VehicleStatus

from wingbreaker_uav.drone_gateway import DroneGateway, distance_m


class FlightNode(Node):

    def __init__(self):
        super().__init__('flight_node')

        self.declare_parameter('system_address', 'udpin://0.0.0.0:14540')
        self.declare_parameter('status_rate_hz', 2.0)
        address = str(self.get_parameter('system_address').value)
        rate = float(self.get_parameter('status_rate_hz').value)

        self.gw = DroneGateway(system_address=address, name='flight')
        self.gw.start()

        self.status_pub = self.create_publisher(VehicleStatus, 'vehicle_status', 10)
        self.status_timer = self.create_timer(1.0 / rate, self.publish_status)

        self.server = ActionServer(self, FlyToGPS, 'fly_to_gps',
                                   self.execute_callback)
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

        ok = False
        try:
            if not self.gw.in_air:
                self.get_logger().info('First goal - arming and taking off')
                self.gw.call(self.gw.arm_and_takeoff(alt), timeout=60.0)

            self.gw.call(self.gw.goto(lat, lon, alt), timeout=10.0)

            # stream feedback until within arrival radius
            feedback = FlyToGPS.Feedback()
            import time
            deadline = time.time() + 300.0
            while time.time() < deadline:
                pos = self.gw.position
                if pos:
                    d = distance_m(pos['lat'], pos['lon'], lat, lon)
                    feedback.distance_remaining = float(d)
                    goal_handle.publish_feedback(feedback)
                    if d < 5.0:
                        ok = True
                        break
                time.sleep(1.0)
            if not ok:
                self.get_logger().warn('Arrival timeout - goal failed')
        except Exception as e:      # noqa: BLE001
            self.get_logger().warn('Flight error: %s' % e)
            ok = False

        goal_handle.succeed()
        result = FlyToGPS.Result()
        result.success = ok
        result.message = 'Arrived' if ok else 'Flight failed'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FlightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
