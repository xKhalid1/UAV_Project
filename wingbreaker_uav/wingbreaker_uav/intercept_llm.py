"""LLM interception approver - OPTIONAL, runs as a separate node.

Subscribes to mission_state; when the brain enters DECISION it evaluates the
pending interception and answers via the /request_interception service with
approver="llm".

Providers:
    mock   - deterministic policy: approve iff confidence >= threshold.
             No network, no API key. Default while no real LLM is wired in.
    openai / anthropic / ollama - placeholders: raise NotImplementedError with
             clear instructions so wiring a real provider later is a small,
             contained change in `_ask_llm`.

Run only when brain's approval_mode is "llm". The base system never imports
this file.
"""

import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from wingbreaker_interfaces.msg import IntruderDetection, MissionState
from wingbreaker_interfaces.srv import RequestInterception


class InterceptLLM(Node):

    def __init__(self):
        super().__init__('intercept_llm')
        self.cb = ReentrantCallbackGroup()

        self.declare_parameter('provider', 'mock')
        self.declare_parameter('model', 'gpt-4o-mini')
        self.declare_parameter('api_key_env', 'OPENAI_API_KEY')
        self.declare_parameter('confidence_threshold', 0.85)
        self.declare_parameter('decision_delay_s', 2.0)

        self.provider = str(self.get_parameter('provider').value)
        self.model = str(self.get_parameter('model').value)
        self.api_key_env = str(self.get_parameter('api_key_env').value)
        self.conf_threshold = float(
            self.get_parameter('confidence_threshold').value)
        self.decision_delay = float(self.get_parameter('decision_delay_s').value)

        self.latest_detection = None
        self.pending_since = None
        self.decided_for = None      # timestamp of DECISION entry we answered

        self.create_subscription(
            MissionState, 'mission_state', self.on_state, 10,
            callback_group=self.cb)
        self.create_subscription(
            IntruderDetection, 'detections', self.on_detection, 10,
            callback_group=self.cb)
        self.client = self.create_client(
            RequestInterception, 'request_interception',
            callback_group=self.cb)

        self.poll_timer = self.create_timer(0.5, self.poll, callback_group=self.cb)
        self.get_logger().info(
            'InterceptLLM online - provider=%s model=%s threshold=%.2f'
            % (self.provider, self.model, self.conf_threshold))

    # ---------- subscriptions ----------
    def on_detection(self, msg):
        self.latest_detection = msg

    def on_state(self, msg):
        if msg.state == MissionState.DECISION:
            if self.pending_since is None:
                self.pending_since = time.time()
                self.get_logger().info(
                    'DECISION pending - will evaluate in %.1fs'
                    % self.decision_delay)

    # ---------- decision loop ----------
    def poll(self):
        if self.pending_since is None:
            return
        if time.time() - self.pending_since < self.decision_delay:
            return
        # answer once per DECISION episode
        episode = self.pending_since
        self.pending_since = None
        if not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('request_interception service unavailable')
            return

        det = self.latest_detection
        conf = float(det.confidence) if det else 0.0
        approve = self._decide(conf)

        req = RequestInterception.Request()
        req.approve = bool(approve)
        req.approver = 'llm'
        fut = self.client.call_async(req)
        fut.add_done_callback(lambda f: self._on_answer(f, conf, approve))
        _ = episode  # episodes are distinguished by state transitions

    def _decide(self, confidence):
        """Provider dispatch. Mock policy: approve iff conf >= threshold."""
        if self.provider == 'mock':
            reason = ('confidence %.2f >= %.2f' % (confidence, self.conf_threshold)
                      if confidence >= self.conf_threshold else
                      'confidence %.2f < %.2f' % (confidence, self.conf_threshold))
            self.get_logger().warn(
                '[MockLLM %s] %s -> %s' % (self.model, reason,
                                           'APPROVE' if confidence >= self.conf_threshold
                                           else 'DENY'))
            return confidence >= self.conf_threshold
        return bool(self._ask_llm(confidence))

    def _ask_llm(self, confidence):
        """Real provider hook - implement here when an API key is available."""
        raise NotImplementedError(
            "provider '%s' not wired yet. Implement _ask_llm() using env var "
            "'%s' for the API key, or set provider=mock."
            % (self.provider, self.api_key_env))

    def _on_answer(self, future, confidence, approve):
        try:
            resp = future.result()
            self.get_logger().info(
                'Decision sent (conf=%.2f approve=%s): %s'
                % (confidence, approve, resp.message))
        except Exception as e:      # noqa: BLE001
            self.get_logger().warn('Service call failed: %s' % e)


def main(args=None):
    rclpy.init(args=args)
    node = InterceptLLM()
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
