"""Intruder detector node.

Two modes (config-driven):

* Real mode (`use_simulated: false`): subscribes to the gimbal camera image,
  runs a YOLO model (ultralytics) on frames and converts the strongest
  detection into an estimated GPS position of the intruder.
* Simulated mode (`use_simulated: true`, default while the trained model is
  not yet in place): publishes synthetic intruder positions on a timer so the
  whole chase/approve/report pipeline can be exercised end-to-end.

Pixel -> GPS estimate (real mode):
  The camera looks along the vehicle nose. The horizontal pixel offset of the
  detection center maps to a bearing offset via the camera HFOV; the target is
  then placed at `assume_range_m` along that bearing from the vehicle's own
  GPS position. Altitude is assumed equal to the vehicle's relative altitude.
  This is deliberately simple - refine later with depth or telemetry fusion.
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from wingbreaker_interfaces.msg import IntruderDetection, VehicleStatus


class Detector(Node):

    def __init__(self):
        super().__init__('detector')

        # ---- parameters ----
        def p(name):
            return self.declare_parameter(name)

        self.camera_topic = str(p('camera_topic').value)
        self.model_path = str(p('model_path').value)
        self.conf_threshold = float(p('confidence_threshold').value)
        self.target_classes = [c for c in
                               list(p('target_classes').value or []) if c]
        self.use_simulated = bool(p('use_simulated').value)
        self.sim_interval = float(p('sim_interval_s').value)
        self.sim_base_lat = float(p('sim_base_lat').value)
        self.sim_base_lon = float(p('sim_base_lon').value)
        self.assume_range_m = float(p('assume_range_m').value)
        self.hfov_rad = math.radians(float(p('hfov_deg').value))

        # ---- state ----
        self.bridge = CvBridge()
        self.vehicle_status = None     # latest VehicleStatus
        self.model = None
        self.frame_stride = 3          # run YOLO every Nth frame
        self._frame_count = 0

        # ---- IO ----
        self.status_sub = self.create_subscription(
            VehicleStatus, 'vehicle_status', self._on_status, 10)
        self.pub = self.create_publisher(IntruderDetection, 'detections', 10)

        if self.use_simulated:
            self.timer = self.create_timer(self.sim_interval, self._scan_sim)
            self.get_logger().warn(
                'Detector in SIMULATED mode - publishing synthetic '
                'intruders every %.0fs' % self.sim_interval)
        else:
            self._load_model()
            self.image_sub = self.create_subscription(
                Image, self.camera_topic, self._on_image,
                qos_profile_sensor_data)
            self.get_logger().info(
                'Detector online - model=%s topic=%s'
                % (self.model_path, self.camera_topic))

    # ---------- helpers ----------
    def _load_model(self):
        from ultralytics import YOLO
        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info('YOLO model loaded: %s' % self.model_path)
        except Exception as e:      # noqa: BLE001
            self.get_logger().error('Failed to load YOLO model: %s' % e)

    def _on_status(self, msg):
        self.vehicle_status = msg

    def _estimate_gps(self, bbox_center_x, image_width):
        """Map a pixel x-coordinate to an estimated target lat/lon."""
        st = self.vehicle_status
        if st is None or not st.connected:
            return None
        nx = (bbox_center_x / float(image_width) - 0.5) * 2.0
        bearing_offset = math.degrees(math.atan(nx * math.tan(self.hfov_rad / 2)))
        heading = st.heading_deg if st.heading_deg else 0.0
        bearing = (heading + bearing_offset) % 360.0
        lat, lon = self._destination(st.latitude_deg, st.longitude_deg,
                                     bearing, self.assume_range_m)
        return lat, lon

    @staticmethod
    def _destination(lat_deg, lon_deg, bearing_deg, range_m):
        m_per_deg_lat = 111320.0
        m_per_deg_lon = 111320.0 * math.cos(math.radians(lat_deg))
        br = math.radians(bearing_deg)
        return (lat_deg + range_m * math.cos(br) / m_per_deg_lat,
                lon_deg + range_m * math.sin(br) / m_per_deg_lon)

    # ---------- real YOLO path ----------
    def _on_image(self, msg):
        self._frame_count += 1
        if self._frame_count % self.frame_stride != 0 or self.model is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:      # noqa: BLE001
            self.get_logger().warn('cv_bridge failed: %s' % e)
            return

        results = self.model.predict(frame, verbose=False,
                                     conf=self.conf_threshold)
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return

        names = results[0].names
        best = None
        for box in boxes:
            cls_name = names[int(box.cls[0])]
            conf = float(box.conf[0])
            if self.target_classes and cls_name.lower() not in \
                    [c.lower() for c in self.target_classes]:
                continue
            if best is None or conf > best[1]:
                best = (box, conf, cls_name)

        if best is None:
            return
        box, conf, cls_name = best
        x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
        cx = (x1 + x2) / 2.0
        est = self._estimate_gps(cx, frame.shape[1])
        if est is None:
            self.get_logger().warn(
                'Detection seen but no vehicle_status yet - skipping')
            return
        lat, lon = est
        st = self.vehicle_status
        alt = float(st.relative_altitude_m) if st is not None else 0.0
        self._publish(lat, lon, alt, conf, cls_name)

    # ---------- simulated path ----------
    def _scan_sim(self):
        rng = np.random.default_rng()
        lat = self.sim_base_lat + float(rng.uniform(-0.004, 0.004))
        lon = self.sim_base_lon + float(rng.uniform(-0.004, 0.004))
        self._publish(lat, lon, 65.0, 0.95, 'sim_intruder')

    # ---------- shared ----------
    def _publish(self, lat, lon, alt, conf, label):
        m = IntruderDetection()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'detector'
        m.latitude = float(lat)
        m.longitude = float(lon)
        m.altitude = float(alt)
        m.confidence = float(conf)
        m.threat_level = 'HIGH' if conf >= 0.85 else (
            'MEDIUM' if conf >= 0.5 else 'LOW')
        self.pub.publish(m)
        self.get_logger().info(
            'INTRUDER [%s] at (%.5f, %.5f) conf=%.2f [%s]'
            % (label, lat, lon, conf, m.threat_level))


def main(args=None):
    import rclpy
    from rclpy import init, shutdown, spin

    init()
    node = Detector()
    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
