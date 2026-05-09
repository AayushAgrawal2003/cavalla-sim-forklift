#!/usr/bin/env python3
"""ArUco distance guard for line following.

Watches for ArUco markers ``4, 5, 6, 7`` in a forward-facing camera
feed, estimates each detected marker's range from the camera using the
loaded intrinsics, and publishes the **average** distance over the
markers currently visible.

Intrinsics source (in priority order):

1. ``calib_file`` -- a YAML in standard ROS ``camera_info`` format
   (``camera_matrix.data`` length-9 row-major,
   ``distortion_coefficients.data``).  When set, this overrides the
   topic.
2. ``camera_info_topic`` -- live ``sensor_msgs/CameraInfo`` from the
   camera driver.  ``luxonis_cam_pipeline`` publishes
   ``/<camera>/camera_info`` using the CAM_A (colour) intrinsics
   scaled to the published frame size.

When ``distance_threshold_m`` is positive and the measured average is
below it, the guard publishes ``below_threshold = True`` on
``/aruco_distance_guard/below_threshold``. ``line_follower_controller``
subscribes to that topic and aborts the active line-follow run.

Threshold is set via ``cavalla automation aruco-calib``. ``-1.0`` (the
shipped default) disables the guard.
"""

from __future__ import annotations

import json
import math
import os
import threading
from typing import Optional

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, String


def _sensor_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


_ARUCO_DICTS: dict[str, int] = {
    'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
    'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
    'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
    'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
    'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
    'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
    'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
    'DICT_5X5_1000': cv2.aruco.DICT_5X5_1000,
    'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
    'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
    'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
    'DICT_6X6_1000': cv2.aruco.DICT_6X6_1000,
    'DICT_7X7_50': cv2.aruco.DICT_7X7_50,
    'DICT_7X7_100': cv2.aruco.DICT_7X7_100,
    'DICT_7X7_250': cv2.aruco.DICT_7X7_250,
    'DICT_7X7_1000': cv2.aruco.DICT_7X7_1000,
    'DICT_ARUCO_ORIGINAL': cv2.aruco.DICT_ARUCO_ORIGINAL,
}


class ArucoDistanceGuard(Node):
    def __init__(self) -> None:
        super().__init__('aruco_distance_guard')

        self.declare_parameter('image_topic', '/front_low/rgb/raw')
        self.declare_parameter('camera_info_topic', '/front_low/camera_info')
        self.declare_parameter('distance_topic', '/aruco_distance_guard/distance')
        self.declare_parameter('below_threshold_topic', '/aruco_distance_guard/below_threshold')
        self.declare_parameter('state_topic', '/aruco_distance_guard/state')
        self.declare_parameter('overlay_topic', '/aruco_distance_guard/image_overlay')
        self.declare_parameter('overlay_enabled', True)
        self.declare_parameter('rotate_180', False)

        self.declare_parameter('aruco_dict', 'DICT_5X5_50')
        self.declare_parameter('marker_ids', [4, 5, 6, 7])
        self.declare_parameter('marker_length_m', 0.05)
        self.declare_parameter('corner_refine_method', 'APRILTAG')

        self.declare_parameter('calib_file', '')
        self.declare_parameter('distance_threshold_m', -1.0)
        self.declare_parameter('publish_rate_hz', 20.0)

        self._bridge = CvBridge()
        self._lock = threading.Lock()

        self._latest_distance_m: float = float('nan')
        self._latest_marker_ids: list[int] = []
        self._latest_image_size: tuple[int, int] = (0, 0)

        self._aruco_detector: Optional[cv2.aruco.ArucoDetector] = None
        self._aruco_dict_name: str = ''
        self._aruco_refine_name: str = ''
        self._build_aruco_detector()

        self._camera_matrix: Optional[np.ndarray] = None
        self._dist_coeffs: Optional[np.ndarray] = None
        self._calib_source: str = 'none'
        self._load_calibration_from_yaml()

        qos = _sensor_qos()
        image_topic = str(self.get_parameter('image_topic').value)
        info_topic = str(self.get_parameter('camera_info_topic').value).strip()
        if self._camera_matrix is None and info_topic:
            self.create_subscription(CameraInfo, info_topic, self._on_camera_info, qos)
            self.get_logger().info(
                f'no calib_file set; subscribing to {info_topic} for intrinsics'
            )

        self.create_subscription(Image, image_topic, self._on_image, qos)
        self._pub_distance = self.create_publisher(
            Float32, str(self.get_parameter('distance_topic').value), 10
        )
        self._pub_below = self.create_publisher(
            Bool, str(self.get_parameter('below_threshold_topic').value), 10
        )
        self._pub_state = self.create_publisher(
            String, str(self.get_parameter('state_topic').value), 10
        )
        self._pub_overlay = self.create_publisher(
            Image, str(self.get_parameter('overlay_topic').value), qos
        )

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / max(1.0, rate), self._on_timer)

        self.get_logger().info(
            f'aruco_distance_guard ready: {image_topic} -> '
            f'{self.get_parameter("distance_topic").value} '
            f'(threshold={self.get_parameter("distance_threshold_m").value} m, '
            f'ids={list(self.get_parameter("marker_ids").value)}, '
            f'aruco_dict={self._aruco_dict_name}, '
            f'calib_source={self._calib_source})'
        )

    def _build_aruco_detector(self) -> None:
        dict_name = str(self.get_parameter('aruco_dict').value).strip()
        dict_id = _ARUCO_DICTS.get(dict_name)
        if dict_id is None:
            self.get_logger().error(
                f'unknown aruco_dict "{dict_name}"; falling back to DICT_5X5_50'
            )
            dict_name = 'DICT_5X5_50'
            dict_id = _ARUCO_DICTS[dict_name]
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        params = cv2.aruco.DetectorParameters()
        refine = str(self.get_parameter('corner_refine_method').value).strip().upper()
        refine_map = {
            'NONE': cv2.aruco.CORNER_REFINE_NONE,
            'SUBPIX': cv2.aruco.CORNER_REFINE_SUBPIX,
            'CONTOUR': cv2.aruco.CORNER_REFINE_CONTOUR,
            'APRILTAG': cv2.aruco.CORNER_REFINE_APRILTAG,
        }
        params.cornerRefinementMethod = refine_map.get(
            refine, cv2.aruco.CORNER_REFINE_APRILTAG
        )
        if refine not in refine_map:
            refine = 'APRILTAG'
        self._aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        self._aruco_dict_name = dict_name
        self._aruco_refine_name = refine

    def _load_calibration_from_yaml(self) -> None:
        path = str(self.get_parameter('calib_file').value).strip()
        if not path:
            return
        if not os.path.isfile(path):
            self.get_logger().error(
                f'calib_file "{path}" not found; will try camera_info topic instead'
            )
            return
        try:
            with open(path, 'r') as fh:
                data = yaml.safe_load(fh) or {}
        except Exception as exc:
            self.get_logger().error(f'failed to read calib_file {path}: {exc}')
            return

        cm_block = data.get('camera_matrix') or {}
        cm_data = cm_block.get('data') if isinstance(cm_block, dict) else None
        dc_block = data.get('distortion_coefficients') or {}
        dc_data = dc_block.get('data') if isinstance(dc_block, dict) else None
        if not cm_data or len(cm_data) != 9:
            self.get_logger().error(
                f'calib_file {path}: camera_matrix.data must be length-9'
            )
            return
        if dc_data is None:
            dc_data = [0.0, 0.0, 0.0, 0.0, 0.0]

        self._camera_matrix = np.asarray(cm_data, dtype=np.float64).reshape(3, 3)
        self._dist_coeffs = np.asarray(dc_data, dtype=np.float64).reshape(-1)
        self._calib_source = f'file:{path}'
        self.get_logger().info(
            f'loaded calib from {path}: '
            f'fx={self._camera_matrix[0, 0]:.1f} fy={self._camera_matrix[1, 1]:.1f} '
            f'cx={self._camera_matrix[0, 2]:.1f} cy={self._camera_matrix[1, 2]:.1f}'
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        try:
            k = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        except (ValueError, TypeError):
            return
        if k[0, 0] <= 0.0 or k[1, 1] <= 0.0:
            return
        d = np.asarray(msg.d, dtype=np.float64).reshape(-1) if msg.d else np.zeros(5)
        first = self._camera_matrix is None
        self._camera_matrix = k
        self._dist_coeffs = d
        self._calib_source = (
            f'topic:{self.get_parameter("camera_info_topic").value}'
        )
        if first:
            self.get_logger().info(
                f'received camera_info: fx={k[0, 0]:.1f} fy={k[1, 1]:.1f} '
                f'cx={k[0, 2]:.1f} cy={k[1, 2]:.1f} D={d.tolist()}'
            )

    def _on_image(self, msg: Image) -> None:
        frame = self._decode_to_bgr(msg)
        if frame is None:
            return

        if bool(self.get_parameter('rotate_180').value):
            frame = cv2.rotate(frame, cv2.ROTATE_180)

        h, w = frame.shape[:2]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        try:
            corners, ids, _ = self._aruco_detector.detectMarkers(gray)
        except cv2.error as e:
            self.get_logger().warn(f'ArUco detect failed: {e}')
            corners, ids = [], None

        wanted_ids = [int(v) for v in self.get_parameter('marker_ids').value]
        marker_length = float(self.get_parameter('marker_length_m').value)

        per_marker: list[tuple[int, float, np.ndarray]] = []
        if (
            ids is not None
            and len(ids) > 0
            and self._camera_matrix is not None
            and self._dist_coeffs is not None
        ):
            ids_flat = ids.flatten().tolist()
            for marker_id in wanted_ids:
                if marker_id not in ids_flat:
                    continue
                idx = ids_flat.index(marker_id)
                quad = corners[idx][0]
                tvec = self._estimate_marker_tvec(quad, marker_length)
                if tvec is None:
                    continue
                dist = float(np.linalg.norm(tvec))
                per_marker.append((marker_id, dist, quad))

        if per_marker:
            avg = float(np.mean([d for _, d, _ in per_marker]))
            seen = [m for m, _, _ in per_marker]
        else:
            avg = float('nan')
            seen = []

        with self._lock:
            self._latest_distance_m = avg
            self._latest_marker_ids = seen
            self._latest_image_size = (int(w), int(h))

        if bool(self.get_parameter('overlay_enabled').value):
            overlay = self._draw_overlay(frame, per_marker, avg)
            try:
                ov_msg = self._bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
                ov_msg.header = msg.header
                self._pub_overlay.publish(ov_msg)
            except Exception as e:
                self.get_logger().warn(f'overlay publish failed: {e}')

    def _estimate_marker_tvec(
        self, quad: np.ndarray, marker_length_m: float
    ) -> Optional[np.ndarray]:
        if marker_length_m <= 0.0:
            return None
        half = float(marker_length_m) * 0.5
        obj_pts = np.array(
            [
                [-half,  half, 0.0],
                [ half,  half, 0.0],
                [ half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float64,
        )
        img_pts = np.asarray(quad, dtype=np.float64).reshape(-1, 2)
        try:
            ok, _rvec, tvec = cv2.solvePnP(
                obj_pts,
                img_pts,
                self._camera_matrix,
                self._dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
        except cv2.error as e:
            self.get_logger().warn(f'solvePnP failed: {e}')
            return None
        if not ok:
            return None
        return tvec.reshape(3)

    def _on_timer(self) -> None:
        with self._lock:
            distance = self._latest_distance_m
            seen = list(self._latest_marker_ids)
            w, h = self._latest_image_size

        threshold = float(self.get_parameter('distance_threshold_m').value)
        threshold_active = threshold > 0.0
        below = (
            threshold_active
            and not math.isnan(distance)
            and distance < threshold
        )

        self._pub_distance.publish(Float32(data=float(distance)))
        self._pub_below.publish(Bool(data=bool(below)))

        payload = {
            't': self.get_clock().now().nanoseconds * 1e-9,
            'distance_m': None if math.isnan(distance) else round(distance, 4),
            'markers_detected': seen,
            'wanted_ids': [int(v) for v in self.get_parameter('marker_ids').value],
            'threshold_m': threshold,
            'threshold_active': threshold_active,
            'below_threshold': bool(below),
            'image_width': int(w),
            'image_height': int(h),
            'aruco_dict': self._aruco_dict_name,
            'calib_loaded': bool(self._camera_matrix is not None),
            'calib_source': self._calib_source,
        }
        self._pub_state.publish(String(data=json.dumps(payload)))

    def _decode_to_bgr(self, msg: Image) -> Optional[np.ndarray]:
        encoding = (msg.encoding or '').lower()
        h = int(msg.height)
        w = int(msg.width)
        try:
            if encoding == 'nv12':
                expected = w * h * 3 // 2
                buf = np.frombuffer(msg.data, dtype=np.uint8)
                if buf.size < expected:
                    return None
                yuv = buf[:expected].reshape((h * 3 // 2, w))
                return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
            if encoding in ('bgr8', ''):
                return self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if encoding == 'rgb8':
                rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            if encoding in ('bgra8', 'rgba8'):
                arr = self._bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
                if encoding == 'rgba8':
                    arr = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
                else:
                    arr = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
                return arr
            if encoding == 'mono8':
                gray = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            return self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(
                f'image decode failed (encoding={msg.encoding!r}): {e}'
            )
            return None

    def _draw_overlay(
        self,
        frame: np.ndarray,
        per_marker: list[tuple[int, float, np.ndarray]],
        avg_distance_m: float,
    ) -> np.ndarray:
        out = frame.copy()
        h, w = out.shape[:2]

        for marker_id, dist, quad in per_marker:
            pts = quad.astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(out, [pts], True, (0, 255, 0), 2)
            cx = int(round(float(np.mean(quad[:, 0]))))
            cy = int(round(float(np.mean(quad[:, 1]))))
            label = f'id={marker_id} {dist:.2f}m'
            cv2.putText(out, label, (cx + 6, cy - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(out, label, (cx + 6, cy - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        threshold = float(self.get_parameter('distance_threshold_m').value)
        if math.isnan(avg_distance_m):
            hud = (
                f'avg=---  thr='
                f'{("DISABLED" if threshold <= 0.0 else f"{threshold:.2f}m")}  '
                f'detected={[m for m, _, _ in per_marker] or "none"}  '
                f'calib={self._calib_source}'
            )
            colour = (255, 255, 255)
        else:
            below = threshold > 0.0 and avg_distance_m < threshold
            hud = (
                f'avg={avg_distance_m:.2f}m  '
                f'thr={("DISABLED" if threshold <= 0.0 else f"{threshold:.2f}m")}  '
                f'detected={[m for m, _, _ in per_marker]}  '
                f'{"BELOW" if below else "ok"}'
            )
            colour = (0, 0, 255) if below else (0, 255, 0)
        cv2.putText(out, hud, (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, hud, (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 1, cv2.LINE_AA)
        return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoDistanceGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
