"""Interceptor mission supervisor (brain).

State machine:
    IDLE -> CONNECTING -> PATROL -> LOCK -> DECISION -> ENGAGE -> REPORT -> PATROL
                                                                    \
                                     (denied / timeout) -------->----^

* PATROL      : cycles configured waypoints via the fly_to_gps action.
* LOCK        : intruder detected above threshold - abandon patrol, fly to the
                estimated intruder position, then orbit it.
* DECISION    : waits for interception approval (human via /request_interception,
                LLM via the separate intercept_llm node, or auto mode).
* ENGAGE      : calls the engagement node's /fire service.
* REPORT      : publishes InterceptReport, cools down, resumes patrol.

Approval modes (`approval_mode` param):
    auto  - engage immediately after lock (no approval step)
    human - wait for a /request_interception call with approve=true
    llm   - wait for the intercept_llm node to decide (same service)
"""

import math
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from wingbreaker_interfaces.action import FlyToGPS
from wingbreaker_interfaces.msg import (
    InterceptReport,
    IntruderDetection,
    MissionState,
    VehicleStatus,
)
from wingbreaker_interfaces.srv import RequestInterception


class Brain(Node):

    def __init__(self):
        super().__init__('brain')
        self.cb = ReentrantCallbackGroup()

        # ---- parameters ----
        # NOTE: ROS 2 params do not support nested arrays - waypoints are a
        # flat list [lat0, lon0, lat1, lon1, ...]
        self.declare_parameter('waypoints', [
            47.39830, 8.54560, 47.39830, 8.54660,
            47.39770, 8.54660, 47.39770, 8.54560])
        self.declare_parameter('patrol_alt', 65.0)
        self.declare_parameter('detection_conf_threshold', 0.85)
        self.declare_parameter('orbit_radius_m', 40.0)
        self.declare_parameter('orbit_points', 8)
        self.declare_parameter('decision_timeout_s', 30.0)
        self.declare_parameter('cooldown_s', 10.0)
        self.declare_parameter('approval_mode', 'human')

        wp_flat = list(self.get_parameter('waypoints').value or [])
        self.waypoints = [
            (float(wp_flat[i]), float(wp_flat[i + 1]))
            for i in range(0, len(wp_flat) - 1, 2)]
        self.patrol_alt = float(self.get_parameter('patrol_alt').value)
        self.conf_threshold = float(
            self.get_parameter('detection_conf_threshold').value)
        self.orbit_radius = float(self.get_parameter('orbit_radius_m').value)
        self.orbit_points = int(self.get_parameter('orbit_points').value)
        self.decision_timeout = float(
            self.get_parameter('decision_timeout_s').value)
        self.cooldown_s = float(self.get_parameter('cooldown_s').value)
        self.approval_mode = str(self.get_parameter('approval_mode').value)

        # ---- state ----
        self.state = MissionState.IDLE
        self.detail = 'starting'
        self.current_wp = 0
        self.busy = False
        self.safe_to_fly = True
        self.vehicle_status = None
        self.target = None             # dict(lat, lon, alt, conf)
        self.approval_result = None    # bool | None
        self.decision_deadline = None
        self.cooldown_until = 0.0
        self.orbit_idx = 0
        self._lock = threading.Lock()

        # ---- IO ----
        self.fly_client = ActionClient(
            self, FlyToGPS, 'fly_to_gps', callback_group=self.cb)
        self.fire_client = self.create_client(
            Trigger, 'fire', callback_group=self.cb)

        self.pub_state = self.create_publisher(
            MissionState, 'mission_state', 10)
        self.pub_report = self.create_publisher(
            InterceptReport, 'intercept_reports', 10)

        self.create_subscription(
            IntruderDetection, 'detections', self.on_detection, 10,
            callback_group=self.cb)
        self.create_subscription(
            VehicleStatus, 'vehicle_status', self.on_status, 10,
            callback_group=self.cb)
        self.create_subscription(
            Bool, 'safe_to_fly', self.on_safety, 10, callback_group=self.cb)

        self.auth_srv = self.create_service(
            RequestInterception, 'request_interception',
            self.on_request_interception, callback_group=self.cb)

        self.tick_timer = self.create_timer(
            1.0, self.tick, callback_group=self.cb)
        self._set_state(MissionState.IDLE, 'waiting to start')

    # ---------- state helper ----------
    def _set_state(self, state, detail=''):
        with self._lock:
            self.state = state
            self.detail = detail
        msg = MissionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = state
        msg.detail = detail
        self.pub_state.publish(msg)
        names = {MissionState.IDLE: 'IDLE', MissionState.CONNECTING: 'CONNECTING',
                 MissionState.PATROL: 'PATROL', MissionState.LOCK: 'LOCK',
                 MissionState.DECISION: 'DECISION', MissionState.ENGAGE: 'ENGAGE',
                 MissionState.REPORT: 'REPORT', MissionState.ERROR: 'ERROR'}
        self.get_logger().info('STATE: %s (%s)' % (names.get(state, state),
                                                   detail))

    # ---------- subscriptions ----------
    def on_safety(self, msg):
        self.safe_to_fly = msg.data

    def on_status(self, msg):
        self.vehicle_status = msg

    def on_detection(self, msg):
        if self.state != MissionState.PATROL:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now < self.cooldown_until:
            return
        if not self.safe_to_fly:
            self.get_logger().warn('Intruder seen but battery LOW - not chasing')
            return
        if msg.confidence < self.conf_threshold:
            return
        self.target = {'lat': msg.latitude, 'lon': msg.longitude,
                       'alt': msg.altitude, 'conf': msg.confidence}
        self.get_logger().warn(
            '>>> INTRUDER conf=%.2f - abandoning patrol, LOCKING' % msg.confidence)
        self._cancel_current_goal()
        self.orbit_idx = 0
        self.busy = False
        self._set_state(MissionState.LOCK, 'flying to intruder')

    # ---------- approval service ----------
    def on_request_interception(self, request, response):
        if self.state != MissionState.DECISION:
            response.success = False
            response.message = 'not in DECISION state (state=%d)' % self.state
            return response
        self.approval_result = bool(request.approve)
        self.approved_by = request.approver or 'unknown'
        response.success = True
        response.message = ('approved' if request.approve else 'denied')
        self.get_logger().warn(
            '*** INTERCEPTION %s by %s ***'
            % (response.message.upper(), self.approved_by))
        return response

    # ---------- main tick ----------
    def tick(self):
        st = self.state
        if st == MissionState.IDLE:
            if self.vehicle_status and self.vehicle_status.connected:
                self._set_state(MissionState.PATROL, 'vehicle connected')
            else:
                self._set_state(MissionState.CONNECTING, 'connecting...')
            return
        if st == MissionState.CONNECTING:
            if self.vehicle_status and self.vehicle_status.connected:
                self._set_state(MissionState.PATROL, 'connected')
            return
        if st == MissionState.PATROL:
            self._tick_patrol()
        elif st == MissionState.LOCK:
            self._tick_lock()
        elif st == MissionState.DECISION:
            self._tick_decision()
        elif st == MissionState.REPORT:
            now = self.get_clock().now().nanoseconds / 1e9
            if now >= self.cooldown_until:
                self.busy = False
                self.target = None
                self._set_state(MissionState.PATROL, 'resuming patrol')

    # ---------- PATROL ----------
    def _tick_patrol(self):
        if self.busy or not self.fly_client.wait_for_server(timeout_sec=0.1):
            return
        lat, lon = self.waypoints[self.current_wp]
        self.get_logger().info('PATROL: flying to waypoint %d' % self.current_wp)
        self._send_goal(lat, lon, self.patrol_alt, on_done=self._on_wp_reached)

    def _on_wp_reached(self, success):
        if success and self.state == MissionState.PATROL:
            self.current_wp = (self.current_wp + 1) % len(self.waypoints)

    # ---------- LOCK ----------
    def _tick_lock(self):
        if self.busy or self.target is None:
            return
        t = self.target
        self._send_goal(t['lat'], t['lon'], t['alt'],
                        on_done=lambda ok: self._on_target_reached(ok))

    def _on_target_reached(self, success):
        if self.state != MissionState.LOCK:
            return
        if not success:
            self.get_logger().warn('Lock approach failed - resuming patrol')
            self._finish_report(detected=True, intercepted=False,
                                approved_by='none', message='lock failed')
            return
        self.orbit_idx = 0
        if self.approval_mode == 'auto':
            self.approval_result = True
            self.approved_by = 'auto'
            self._set_state(MissionState.ENGAGE, 'auto-approved')
        else:
            self.approval_result = None
            self.decision_deadline = (self.get_clock().now().nanoseconds / 1e9
                                      + self.decision_timeout)
            self._set_state(MissionState.DECISION,
                            'awaiting interception approval (%s)'
                            % self.approval_mode)

    # ---------- DECISION (orbit while waiting) ----------
    def _orbit_point(self, idx):
        t = self.target
        ang = 2.0 * math.pi * idx / max(1, self.orbit_points)
        dlat = (self.orbit_radius * math.cos(ang)) / 111320.0
        dlon = (self.orbit_radius * math.sin(ang)) / (
            111320.0 * math.cos(math.radians(t['lat'])))
        return t['lat'] + dlat, t['lon'] + dlon, t['alt']

    def _tick_decision(self):
        # approval arrived?
        if self.approval_result is not None:
            approver = getattr(self, 'approved_by', 'unknown')
            if self.approval_result:
                self._set_state(MissionState.ENGAGE, 'approved by %s' % approver)
                ok = self._fire()
                self._finish_report(
                    detected=True, intercepted=ok, approved_by=approver,
                    message='target engaged' if ok else 'fire failed')
            else:
                self._finish_report(detected=True, intercepted=False,
                                    approved_by=approver,
                                    message='interception denied')
            return
        # timeout?
        now = self.get_clock().now().nanoseconds / 1e9
        if now > self.decision_deadline:
            self.get_logger().warn('Decision timeout - aborting engagement')
            self._finish_report(detected=True, intercepted=False,
                                approved_by='timeout', message='timed out')
            return
        # keep orbiting the target
        if self.busy or not self.fly_client.wait_for_server(timeout_sec=0.1):
            return
        lat, lon, alt = self._orbit_point(self.orbit_idx)
        self.orbit_idx = (self.orbit_idx + 1) % self.orbit_points
        self._send_goal(lat, lon, alt, on_done=None)

    # ---------- goal plumbing ----------
    def _cancel_current_goal(self):
        gh = getattr(self, 'current_goal_handle', None)
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception:       # noqa: BLE001
                pass
            self.current_goal_handle = None

    def _send_goal(self, lat, lon, alt, on_done):
        self.busy = True
        goal = FlyToGPS.Goal()
        goal.latitude = float(lat)
        goal.longitude = float(lon)
        goal.altitude = float(alt)
        self.fly_client.wait_for_server()
        fut = self.fly_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._goal_response(f, on_done))

    def _goal_response(self, future, on_done):
        gh = future.result()
        if not gh.accepted:
            self.busy = False
            if on_done:
                on_done(False)
            return
        self.current_goal_handle = gh
        res_fut = gh.get_result_async()
        res_fut.add_done_callback(
            lambda f: self._goal_result(f, on_done))

    def _goal_result(self, future, on_done):
        self.busy = False
        self.current_goal_handle = None
        ok = bool(future.result().result.success)
        if on_done:
            on_done(ok)

    # ---------- ENGAGE + REPORT ----------
    def _fire(self):
        if not self.fire_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Engagement node unavailable')
            return False
        fut = self.fire_client.call_async(Trigger.Request())
        deadline = time.time() + 5.0
        while not fut.done() and time.time() < deadline:
            time.sleep(0.1)
        if not fut.done():
            return False
        resp = fut.result()
        if resp is None:
            return False
        self.get_logger().warn('>>> ENGAGEMENT: %s' % resp.message)
        return bool(resp.success)

    def _finish_report(self, detected, intercepted, approved_by, message):
        t = self.target or {}
        rep = InterceptReport()
        rep.header.stamp = self.get_clock().now().to_msg()
        rep.detected = detected
        rep.intercepted = intercepted
        rep.location = 'LOCK' if t else 'n/a'
        rep.latitude_deg = float(t.get('lat', 0.0))
        rep.longitude_deg = float(t.get('lon', 0.0))
        rep.altitude_m = float(t.get('alt', 0.0))
        rep.detection_confidence = float(t.get('conf', 0.0))
        rep.approved_by = approved_by
        rep.message = message
        self.pub_report.publish(rep)
        self.get_logger().warn(
            'REPORT: detected=%s intercepted=%s by=%s (%.5f, %.5f alt %.0f)'
            % (detected, intercepted, approved_by, rep.latitude_deg,
               rep.longitude_deg, rep.altitude_m))

        if intercepted:
            self._set_state(MissionState.REPORT, 'target engaged')
        else:
            self._set_state(MissionState.REPORT, message)
        self.cooldown_until = (self.get_clock().now().nanoseconds / 1e9
                               + self.cooldown_s)


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
