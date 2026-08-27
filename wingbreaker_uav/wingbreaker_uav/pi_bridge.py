"""Pi bridge node.

Pulls MJPEG frames and JSON detections from a Raspberry Pi running
pi_drone_detector.py and republishes them on the standard Wingbreaker
ROS topics so web_dashboard and brain work unchanged.

Endpoints consumed (all on http://<pi_host>:<pi_port>):
    GET /stream      -> multipart/x-mixed-replace MJPEG
    GET /detections  -> JSON {ts, fps, image_width, image_height, detections: [...]}

Publishes:
    sensor_msgs/Image  on <camera_topic>  (default /camera/image_raw)
    IntruderDetection  on <detections_topic> (default detections)
"""

import json
import math
import threading
import time
import urllib.error
import urllib.request

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from wingbreaker_interfaces.msg import IntruderDetection, VehicleStatus


def _destination(lat_deg, lon_deg, bearing_deg, range_m):
    """Destination point given start, bearing and range (matches detector.py)."""
    m_per_deg_lat = 111320.0
    m_per_deg_lon = 111320.0 * math.cos(math.radians(lat_deg))
    br = math.radians(bearing_deg)
    # guard against cos(lat) -> 0 near the poles
    if abs(m_per_deg_lon) < 1e-6:
        m_per_deg_lon = 1e-6 if m_per_deg_lon >= 0 else -1e-6
    return (lat_deg + range_m * math.cos(br) / m_per_deg_lat,
            lon_deg + range_m * math.sin(br) / m_per_deg_lon)


def _estimate_gps(bbox_center_x, image_width, vehicle_status,
                  hfov_rad, assume_range_m):
    """Map a pixel x-coordinate to an estimated target lat/lon.

    Replicates detector.Detector._estimate_gps exactly.
    Returns (lat, lon) or None if no vehicle_status.
    """
    st = vehicle_status
    if st is None or not st.connected:
        return None
    nx = (bbox_center_x / float(image_width) - 0.5) * 2.0
    bearing_offset = math.degrees(math.atan(nx * math.tan(hfov_rad / 2)))
    heading = st.heading_deg if st.heading_deg else 0.0
    bearing = (heading + bearing_offset) % 360.0
    lat, lon = _destination(st.latitude_deg, st.longitude_deg,
                            bearing, assume_range_m)
    return lat, lon


class PiBridge(Node):

    def __init__(self):
        super().__init__('pi_bridge')

        self.declare_parameter('pi_host', '192.168.1.100')
        self.declare_parameter('pi_port', 5000)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('detections_topic', 'detections')
        self.declare_parameter('vehicle_status_topic', 'vehicle_status')
        self.declare_parameter('poll_hz', 5.0)
        self.declare_parameter('target_classes', ['drone'])
        self.declare_parameter('hfov_deg', 114.6)
        self.declare_parameter('assume_range_m', 60.0)
        self.declare_parameter('stream_reconnect_delay_s', 2.0)

        self._pi_host = str(self.get_parameter('pi_host').value)
        self._pi_port = int(self.get_parameter('pi_port').value)
        self._camera_topic = str(self.get_parameter('camera_topic').value)
        self._detections_topic = str(self.get_parameter('detections_topic').value)
        self._vehicle_status_topic = str(
            self.get_parameter('vehicle_status_topic').value)
        self._poll_hz = float(self.get_parameter('poll_hz').value)
        self._target_classes = [
            c for c in list(self.get_parameter('target_classes').value or [])
            if c]
        self._hfov_rad = math.radians(
            float(self.get_parameter('hfov_deg').value))
        self._assume_range_m = float(
            self.get_parameter('assume_range_m').value)
        self._reconnect_delay = float(
            self.get_parameter('stream_reconnect_delay_s').value)

        self._bridge = CvBridge()
        self._vehicle_status = None
        self._vehicle_lock = threading.Lock()
        self._stop = threading.Event()

        # publishers
        self._image_pub = self.create_publisher(
            Image, self._camera_topic, 10)
        self._detection_pub = self.create_publisher(
            IntruderDetection, self._detections_topic, 10)

        # vehicle status cache
        self.create_subscription(
            VehicleStatus, self._vehicle_status_topic,
            self._on_vehicle_status, 10)

        self._stream_thread = threading.Thread(
            target=self._stream_loop, daemon=True)
        self._stream_thread.start()

        self._poll_thread = threading.Thread(
            target=self._poll_loop, daemon=True)
        self._poll_thread.start()

        base = 'http://%s:%d' % (self._pi_host, self._pi_port)
        self.get_logger().info(
            'PiBridge online - pi=%s camera_topic=%s detections_topic=%s '
            'poll_hz=%.1f' % (base, self._camera_topic,
                              self._detections_topic, self._poll_hz))

    # ------------------------------------------------------------------
    # vehicle status
    # ------------------------------------------------------------------

    def _on_vehicle_status(self, msg):
        with self._vehicle_lock:
            self._vehicle_status = msg

    # ------------------------------------------------------------------
    # MJPEG stream consumer
    # ------------------------------------------------------------------

    def _stream_loop(self):
        url = 'http://%s:%d/stream' % (self._pi_host, self._pi_port)
        consecutive_failures = 0
        while not self._stop.is_set():
            try:
                req = urllib.request.Request(url)
                with urllib.request.urlopen(req, timeout=5) as resp:
                    self.get_logger().info('Connected to Pi MJPEG stream: %s' % url)
                    consecutive_failures = 0
                    self._consume_mjpeg(resp)
            except Exception as exc:  # noqa: BLE001
                consecutive_failures += 1
                if consecutive_failures <= 3 or consecutive_failures % 10 == 0:
                    self.get_logger().warn(
                        'MJPEG stream error (%d): %s - reconnecting in %.1fs'
                        % (consecutive_failures, exc, self._reconnect_delay))
                if self._stop.wait(self._reconnect_delay):
                    break

    def _consume_mjpeg(self, resp):
        """Parse multipart MJPEG by scanning for JPEG SOI/EOI markers."""
        buf = b''
        chunk_size = 4096
        while not self._stop.is_set():
            chunk = resp.read(chunk_size)
            if not chunk:
                raise ConnectionError('MJPEG stream closed by Pi')
            buf += chunk
            # extract all complete JPEGs in the buffer
            while True:
                soi = buf.find(b'\xff\xd8')
                if soi == -1:
                    # no start marker - keep at most last 2 bytes in case
                    # the marker is split across chunks
                    if len(buf) > 2:
                        buf = buf[-2:]
                    break
                eoi = buf.find(b'\xff\xd9', soi)
                if eoi == -1:
                    # incomplete JPEG - keep from SOI onwards
                    if soi > 0:
                        buf = buf[soi:]
                    break
                jpeg_bytes = buf[soi:eoi + 2]
                buf = buf[eoi + 2:]
                self._publish_jpeg(jpeg_bytes)

    def _publish_jpeg(self, jpeg_bytes):
        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return
        try:
            msg = self._bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('cv_bridge encode failed: %s' % exc)
            return
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'pi_camera'
        self._image_pub.publish(msg)

    # ------------------------------------------------------------------
    # detections poller
    # ------------------------------------------------------------------

    def _poll_loop(self):
        url = 'http://%s:%d/detections' % (self._pi_host, self._pi_port)
        period = 1.0 / max(0.1, self._poll_hz)
        warned_no_vehicle = False
        while not self._stop.is_set():
            t0 = time.time()
            try:
                req = urllib.request.Request(url)
                with urllib.request.urlopen(req, timeout=3) as resp:
                    payload = json.loads(resp.read().decode('utf-8'))
                self._handle_detections_payload(payload, warned_no_vehicle)
                warned_no_vehicle = False
            except urllib.error.URLError as exc:
                self.get_logger().warn(
                    'Detections poll failed: %s' % exc, throttle_duration_sec=5.0)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    'Detections poll error: %s' % exc, throttle_duration_sec=5.0)
            elapsed = time.time() - t0
            remaining = period - elapsed
            if remaining > 0:
                self._stop.wait(remaining)

    def _handle_detections_payload(self, payload, warned_no_vehicle):
        detections = payload.get('detections') or []
        image_width = int(payload.get('image_width') or 640)

        with self._vehicle_lock:
            vs = self._vehicle_status

        # optional target class filter (mirrors detector.py)
        target = [c.lower() for c in self._target_classes] if self._target_classes else []

        for det in detections:
            label = str(det.get('label', 'unknown'))
            if target and label.lower() not in target:
                continue
            conf = float(det.get('confidence', 0.0))
            roi = det.get('roi') or {}
            x = int(roi.get('x', 0))
            y = int(roi.get('y', 0))
            w = int(roi.get('w', 0))
            h = int(roi.get('h', 0))

            # GPS estimate
            cx = x + w / 2.0
            est = _estimate_gps(cx, image_width, vs,
                                self._hfov_rad, self._assume_range_m)
            if est is not None:
                lat, lon = est
                vs_alt = float(vs.relative_altitude_m) if vs else 0.0
            else:
                if not warned_no_vehicle:
                    self.get_logger().info(
                        'No vehicle_status yet - publishing detections with lat/lon=0')
                    warned_no_vehicle = True
                lat, lon = 0.0, 0.0
                vs_alt = 0.0

            threat = det.get('threat_level')
            if not threat:
                if conf >= 0.85:
                    threat = 'HIGH'
                elif conf >= 0.5:
                    threat = 'MEDIUM'
                else:
                    threat = 'LOW'

            m = IntruderDetection()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'pi_detector'
            m.latitude = float(lat)
            m.longitude = float(lon)
            m.altitude = float(vs_alt)
            m.confidence = float(conf)
            m.threat_level = str(threat)
            m.roi.x_offset = x
            m.roi.y_offset = y
            m.roi.width = max(0, w)
            m.roi.height = max(0, h)
            m.roi.do_rectify = False
            self._detection_pub.publish(m)
            self.get_logger().info(
                'PI_DETECTION [%s] conf=%.2f [%s] box=(%d,%d,%d,%d) gps=(%.5f,%.5f)'
                % (label, conf, threat, x, y, w, h, lat, lon))

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
