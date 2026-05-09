#!/usr/bin/env python3
"""Yellow guide / stop line detector for the drop-off approach.

Subscribes to *both* candidate cameras (`/fork_low/rgb/raw` and
`/front_low/rgb/raw`) and processes whichever one is selected via the
`active_camera` parameter (`fork_low` or `front_low`). Switching at
runtime requires no node restart:

    ros2 param set /line_detector active_camera front_low

Each camera has independent rotation, desired-guide column, and sign
parameters because:

* The two cameras are mounted on opposite ends of the truck and look in
  opposite world directions, so the image-frame sign of `guide_offset_px`
  and `guide_yaw_rad` flips between them for the same physical
  misalignment. The per-camera `*_sign_*` parameters absorb that flip
  inside the detector so the controller never has to know which camera
  produced a given LineState.
* `fork_low` is mounted upside-down; `front_low` is upright.
* `fork_low` has an x-offset from the truck centerline; `front_low` is
  centered.

Publishes:

* `/drop_off/line_state` (drop_off_msgs/LineState) -- structured detector
  output consumed by `line_follower_controller`.
* `/drop_off/image_overlay` (sensor_msgs/Image, BGR8) -- the original
  frame with the guide line drawn green and the stop line drawn red so
  the operator can visually confirm detection in rqt_image_view before
  ever enabling motion.

Pipeline:
    1. Optional ROI crop (top/bottom fractions).
    2. HSV mask for yellow tape (params hue/sat/val).
    3. Morphological open + close to clean up.
    4. Connected components -> per-blob PCA (`cv2.fitLine`) -> classify:
         * vertical-ish + biggest  -> guide line
         * horizontal-ish          -> stop line
    5. From the guide line we compute:
         * `guide_yaw_rad` -- signed deviation from image-up (vertical).
         * `guide_offset_px` -- signed horizontal pixel distance from
           where the guide intersects the bottom of the ROI to the
           configured `desired_guide_pixel_col`. The desired column is
           NOT necessarily image-center: it absorbs the camera's x
           offset from the forklift centerline (see plan).
    6. From the stop line we compute `stop_distance_px` -- pixel
       distance from image-bottom up to the stop-line centroid. The
       follower's `StopDecider` decides "should we stop?" from this.

All thresholds are ROS parameters with sensible defaults; bench tuning
is expected to adjust HSV/ROI/kernel sizes for the actual lighting and
tape used.

Frame convention: image-pixel coordinates, origin top-left, +x right,
+y down. `guide_yaw_rad` follows the convention documented in
`LineState.msg` (image-up reference, clockwise positive).
"""

from __future__ import annotations

import math
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image

from drop_off_msgs.msg import LineState


def _sensor_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class LineDetector(Node):
    def __init__(self) -> None:
        super().__init__('line_detector')

        # ---- I/O parameters ----
        # Camera selection. The detector subscribes to both topics
        # unconditionally (cheap, BEST_EFFORT QoS) and only processes
        # frames from the camera selected by `active_camera`.
        self.declare_parameter('active_camera', 'fork_low')
        self.declare_parameter('fork_low_topic', '/fork_low/rgb/raw')
        self.declare_parameter('front_low_topic', '/front_low/rgb/raw')

        self.declare_parameter('line_state_topic', '/drop_off/line_state')
        self.declare_parameter('overlay_topic', '/drop_off/image_overlay')
        self.declare_parameter('overlay_enabled', True)

        # Per-camera rotation. fork_low is mounted upside-down on the
        # truck; front_low is upright. Applied right after decode so the
        # rest of the pipeline (and the overlay) is always "image bottom
        # = nearest the robot, image up = further away".
        self.declare_parameter('rotate_180_fork_low', True)
        self.declare_parameter('rotate_180_front_low', False)

        # ---- ROI (fractions of image height; 0=top, 1=bottom) ----
        self.declare_parameter('roi_top_frac', 0.0)
        self.declare_parameter('roi_bottom_frac', 1.0)

        # Horizontal detection band (fractions of full image width).
        # Pixels outside this band are zeroed in the yellow mask before
        # connected components, so wall paint, far-field clutter, and
        # warehouse floor paint near the edges of the frame can't
        # become guide / stop candidates. Default: middle 50% of the
        # image width (1/4 trimmed from each side). Set to 0.0 / 1.0 to
        # disable.
        self.declare_parameter('detect_left_frac', 0.25)
        self.declare_parameter('detect_right_frac', 0.75)

        # ---- HSV thresholds for yellow tape ----
        # OpenCV hue is [0, 180]. Yellow centred around ~25-30. Keep
        # this band tight to avoid grabbing tan floor / wood / cardboard
        # as false positives; if a real piece of tape is being missed
        # because of specular highlights or shadows, prefer enabling
        # `recover_specular` below or widening the spatial search
        # regions rather than dropping S/V here.
        self.declare_parameter('hue_low', 18)
        self.declare_parameter('hue_high', 35)
        self.declare_parameter('sat_min', 80)
        self.declare_parameter('val_min', 80)

        # Secondary mask for the back_out yellow stop tape. Detected
        # in parallel with the primary (red) mask but only
        # horizontal-ish blobs are classified, populating
        # LineState.yellow_stop_detected / yellow_stop_distance_px.
        self.declare_parameter('yellow_hue_low', 18)
        self.declare_parameter('yellow_hue_high', 35)
        self.declare_parameter('yellow_sat_min', 80)
        self.declare_parameter('yellow_val_min', 80)

        # ---- Specular highlight recovery ----
        # Disabled by default: it tends to over-recover on busy bench
        # scenes. Turn on if reflective tape is being broken into
        # multiple pieces by white spots.
        self.declare_parameter('recover_specular', False)
        self.declare_parameter('spec_val_min', 200)
        self.declare_parameter('spec_sat_max', 60)
        self.declare_parameter('spec_attach_px', 8)

        # ---- Morphology ----
        self.declare_parameter('kernel_px', 5)
        self.declare_parameter('close_kernel_px', 5)
        # Throw away tiny specks and components that aren't tape-like.
        self.declare_parameter('min_blob_area_px', 300)

        # ---- Per-line spatial gating (post-rotation, full-image fracs) ----
        # A blob candidate is only considered for the corresponding
        # role if its centroid y is at or below this fraction of the
        # full image height. 0.0 = top of frame, 1.0 = bottom.
        # Defaults: guide may live in lower 2/3, stop only in lower 1/2.
        # Keeps tape further from the camera (visually higher) from
        # being misread.
        self.declare_parameter('guide_search_top_frac', 1.0 / 3.0)
        self.declare_parameter('stop_search_top_frac', 0.5)

        # ---- Stop recovery from a guide-attached cross arm ----
        # A short perpendicular stop strip (e.g. 1/5 image-width) that
        # touches the guide tape merges into a single connected
        # component whose principal axis is the long guide -- so the
        # cross arm gets classified as part of the guide, never as a
        # stop. After identifying the guide, we mask out a strip of
        # `guide_spine_min_width_px` (or `2.5 * perp_std`, whichever is
        # bigger) along its principal axis and re-run connected
        # components on the residual; any horizontal-ish blob there
        # gets a fresh shot at being the stop.
        self.declare_parameter('recover_stop_from_guide', True)
        self.declare_parameter('guide_spine_min_width_px', 12)

        # ---- Classification thresholds (degrees from the reference axis) ----
        self.declare_parameter('guide_yaw_tolerance_deg', 35.0)
        self.declare_parameter('stop_yaw_tolerance_deg', 25.0)
        # Minimum aspect ratio (length / width via PCA) to be considered a
        # "line" rather than a blob. Helps reject reflections.
        self.declare_parameter('min_line_aspect', 2.5)

        # ---- Geometry tied to the camera/forklift mounting ----
        # Pixel column where the guide line should appear when the
        # forklift is properly aligned. -1 = image-centre at runtime
        # (use this when the camera is on the truck centerline).
        # Override with the actual pixel column to encode an x-offset
        # of the camera relative to the centerline.
        self.declare_parameter('desired_guide_pixel_col_fork_low', -1)
        self.declare_parameter('desired_guide_pixel_col_front_low', -1)

        # Per-camera sign convention. fork_low looks toward the rear /
        # conveyor (the direction of travel during drop-off), so the
        # image-frame signs of offset/yaw match the controller's
        # convention directly (+1). front_low looks the opposite way,
        # so the same physical misalignment shows up with the opposite
        # sign in its image -- we negate here so the controller sees a
        # consistent sign regardless of camera. Verify empirically
        # during the in-place yaw test and flip if the steering
        # correction goes the wrong way.
        self.declare_parameter('offset_sign_fork_low', 1.0)
        self.declare_parameter('yaw_sign_fork_low', 1.0)
        self.declare_parameter('offset_sign_front_low', -1.0)
        self.declare_parameter('yaw_sign_front_low', -1.0)

        # Optional homography (3x3, row-major, 9 floats) mapping pixel
        # -> ground-plane metres. If all zeros, `guide_offset_m` is NaN.
        self.declare_parameter('homography_pixel_to_ground', [0.0] * 9)

        # ---- EMA smoothing on (guide_offset_px, guide_yaw_rad) ----
        # alpha in [0, 1]: higher = trust the new measurement more.
        # 1.0 disables smoothing. Reset whenever the guide drops out for
        # longer than `ema_reset_gap_sec` so we don't carry a stale state
        # across re-acquisition.
        self.declare_parameter('ema_alpha', 0.4)
        self.declare_parameter('ema_reset_gap_sec', 0.3)
        self._ema_offset_px: Optional[float] = None
        self._ema_yaw_rad: Optional[float] = None
        self._ema_last_stamp: Optional[float] = None

        self._bridge = CvBridge()

        # ---- pubs/subs ----
        qos = _sensor_qos()
        fork_low_topic = str(self.get_parameter('fork_low_topic').value)
        front_low_topic = str(self.get_parameter('front_low_topic').value)
        self._sub_fork_low = self.create_subscription(
            Image,
            fork_low_topic,
            lambda msg: self._on_image(msg, camera='fork_low'),
            qos,
        )
        self._sub_front_low = self.create_subscription(
            Image,
            front_low_topic,
            lambda msg: self._on_image(msg, camera='front_low'),
            qos,
        )
        self._pub_state = self.create_publisher(
            LineState,
            self.get_parameter('line_state_topic').value,
            10,
        )
        self._pub_overlay = self.create_publisher(
            Image,
            self.get_parameter('overlay_topic').value,
            qos,
        )

        self.get_logger().info(
            f'line_detector ready: active_camera='
            f'{self.get_parameter("active_camera").value} '
            f'(fork_low={fork_low_topic}, front_low={front_low_topic}) -> '
            f'{self.get_parameter("line_state_topic").value} '
            f'(overlay: {self.get_parameter("overlay_topic").value})'
        )

    # ------------------------------------------------------------------
    def _on_image(self, msg: Image, camera: str = 'fork_low') -> None:
        active = str(self.get_parameter('active_camera').value)
        if active != camera:
            return

        frame = self._decode_to_bgr(msg)
        if frame is None:
            return

        rotate_param = f'rotate_180_{camera}'
        if bool(self.get_parameter(rotate_param).value):
            frame = cv2.rotate(frame, cv2.ROTATE_180)

        h, w = frame.shape[:2]
        top_frac = float(self.get_parameter('roi_top_frac').value)
        bot_frac = float(self.get_parameter('roi_bottom_frac').value)
        top_frac = max(0.0, min(0.99, top_frac))
        bot_frac = max(top_frac + 0.01, min(1.0, bot_frac))
        y0 = int(round(top_frac * h))
        y1 = int(round(bot_frac * h))
        roi = frame[y0:y1, :, :]

        mask = self._yellow_mask(roi)

        # Horizontal detection band: zero out the columns of `mask`
        # that fall outside [x_left, x_right). Done in mask-space
        # (cheaper than re-cropping the ROI) so all downstream
        # coordinates remain in full-image x and no offset arithmetic
        # has to know about the band.
        left_frac = max(0.0, min(1.0, float(
            self.get_parameter('detect_left_frac').value
        )))
        right_frac = max(0.0, min(1.0, float(
            self.get_parameter('detect_right_frac').value
        )))
        if right_frac < left_frac:
            left_frac, right_frac = right_frac, left_frac
        x_left = int(round(left_frac * w))
        x_right = int(round(right_frac * w))
        if x_left > 0:
            mask[:, :x_left] = 0
        if x_right < w:
            mask[:, x_right:] = 0

        guide, stop = self._classify_blobs(mask, y0, h)

        # Build the line-state message in *full-image* pixel coordinates
        # (we shift y by `y0` to undo the ROI crop) so the controller
        # doesn't need to know the ROI.
        state = LineState()
        state.header = msg.header  # preserve camera stamp / frame_id
        state.image_width = int(w)
        state.image_height = int(h)
        state.guide_offset_m = float('nan')

        desired_col = int(self.get_parameter(
            f'desired_guide_pixel_col_{camera}'
        ).value)
        if desired_col < 0:
            desired_col = w // 2

        offset_sign = float(self.get_parameter(f'offset_sign_{camera}').value)
        yaw_sign = float(self.get_parameter(f'yaw_sign_{camera}').value)

        if guide is not None:
            # `guide` returns line params in ROI pixel coords. Convert
            # the line's intersection with the bottom row of the ROI to
            # full-image coords for the offset computation.
            vx, vy, x0_l, y0_l = guide['line']
            # x at the bottom of the ROI (y = roi_h - 1, ROI coords).
            roi_h = roi.shape[0]
            x_bottom = self._line_x_at_y(vx, vy, x0_l, y0_l, roi_h - 1)
            offset_px = float(x_bottom - desired_col)
            yaw_rad = self._line_yaw_from_vertical(vx, vy)

            signed_offset_px = offset_sign * offset_px
            signed_yaw_rad = yaw_sign * float(yaw_rad)

            # EMA smoothing. Reset if the guide was lost for more than
            # `ema_reset_gap_sec` so we re-anchor on a fresh measurement
            # instead of dragging in a stale value.
            alpha = max(0.0, min(1.0, float(self.get_parameter('ema_alpha').value)))
            reset_gap = float(self.get_parameter('ema_reset_gap_sec').value)
            now_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if (
                self._ema_offset_px is None
                or self._ema_last_stamp is None
                or (now_s - self._ema_last_stamp) > reset_gap
            ):
                self._ema_offset_px = signed_offset_px
                self._ema_yaw_rad = signed_yaw_rad
            else:
                self._ema_offset_px = (
                    alpha * signed_offset_px + (1.0 - alpha) * self._ema_offset_px
                )
                self._ema_yaw_rad = (
                    alpha * signed_yaw_rad + (1.0 - alpha) * self._ema_yaw_rad
                )
            self._ema_last_stamp = now_s

            state.guide_detected = True
            state.guide_offset_px = float(self._ema_offset_px)
            state.guide_yaw_rad = float(self._ema_yaw_rad)

            # Project to ground metres if a homography is configured.
            H = np.asarray(
                self.get_parameter('homography_pixel_to_ground').value,
                dtype=np.float64,
            )
            if H.size == 9 and np.any(np.abs(H) > 1e-9):
                H = H.reshape(3, 3)
                # Bottom-of-ROI point in *full-image* coords: y = y0 + (roi_h-1).
                pt = np.array([x_bottom, y0 + (roi_h - 1), 1.0])
                ground = H @ pt
                if abs(ground[2]) > 1e-9:
                    state.guide_offset_m = offset_sign * float(
                        ground[0] / ground[2]
                    )
        else:
            state.guide_detected = False
            state.guide_offset_px = 0.0
            state.guide_yaw_rad = 0.0

        if stop is not None:
            # Centroid in ROI -> full-image y. stop_distance_px = full-image
            # bottom (y = h-1) minus stop centroid y.
            cy_full = stop['cy'] + y0
            state.stop_detected = True
            state.stop_distance_px = float((h - 1) - cy_full)
        else:
            state.stop_detected = False
            state.stop_distance_px = 0.0

        # Yellow back_out stop tape: same morphology + horizontal-band gating
        # as the red mask, but only the perpendicular stop classification
        # (no guide). Independent of red detection.
        yellow_mask = self._secondary_yellow_mask(roi)
        if x_left > 0:
            yellow_mask[:, :x_left] = 0
        if x_right < w:
            yellow_mask[:, x_right:] = 0
        _, yellow_stop = self._scan_blobs(
            yellow_mask, y0, self._read_classify_params(h),
        )
        if yellow_stop is not None:
            cy_full = yellow_stop['cy'] + y0
            state.yellow_stop_detected = True
            state.yellow_stop_distance_px = float((h - 1) - cy_full)
        else:
            state.yellow_stop_detected = False
            state.yellow_stop_distance_px = 0.0

        self._pub_state.publish(state)

        if bool(self.get_parameter('overlay_enabled').value):
            guide_top_frac = max(
                0.0, min(1.0, float(self.get_parameter('guide_search_top_frac').value))
            )
            stop_top_frac = max(
                0.0, min(1.0, float(self.get_parameter('stop_search_top_frac').value))
            )
            overlay = self._draw_overlay(
                frame, y0, y1, guide, stop, state, desired_col,
                guide_top_frac=guide_top_frac,
                stop_top_frac=stop_top_frac,
                detect_left_frac=left_frac,
                detect_right_frac=right_frac,
                camera=camera,
            )
            try:
                ov_msg = self._bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
                ov_msg.header = msg.header
                self._pub_overlay.publish(ov_msg)
            except Exception as e:
                self.get_logger().warn(f'overlay publish failed: {e}')

    # ------------------------------------------------------------------
    def _decode_to_bgr(self, msg: Image) -> Optional[np.ndarray]:
        """Convert an incoming sensor_msgs/Image to a BGR8 numpy array.

        Handles the common encodings we care about on this platform,
        including NV12 (which cv_bridge cannot convert directly to BGR8):
        the on-bench `shm_to_ros_bridge.py` publishes
        `/fork_low/rgb/raw` as NV12 at 640x480.
        """
        encoding = (msg.encoding or '').lower()
        h = int(msg.height)
        w = int(msg.width)
        try:
            if encoding == 'nv12':
                expected = w * h * 3 // 2
                buf = np.frombuffer(msg.data, dtype=np.uint8)
                if buf.size < expected:
                    self.get_logger().warn(
                        f'nv12 frame too small: {buf.size} < {expected} '
                        f'({w}x{h})'
                    )
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

    # ------------------------------------------------------------------
    def _yellow_mask(self, bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        hue_low = int(self.get_parameter('hue_low').value)
        hue_high = int(self.get_parameter('hue_high').value)
        sat_min = int(self.get_parameter('sat_min').value)
        val_min = int(self.get_parameter('val_min').value)

        if hue_low > hue_high:
            # Hue wraps around 180 (e.g. red: hue_low=170, hue_high=10).
            mask_lo = cv2.inRange(
                hsv,
                np.array([hue_low, sat_min, val_min], dtype=np.uint8),
                np.array([180, 255, 255], dtype=np.uint8),
            )
            mask_hi = cv2.inRange(
                hsv,
                np.array([0, sat_min, val_min], dtype=np.uint8),
                np.array([hue_high, 255, 255], dtype=np.uint8),
            )
            primary = mask_lo | mask_hi
        else:
            primary = cv2.inRange(
                hsv,
                np.array([hue_low, sat_min, val_min], dtype=np.uint8),
                np.array([hue_high, 255, 255], dtype=np.uint8),
            )

        if bool(self.get_parameter('recover_specular').value):
            spec_val_min = int(self.get_parameter('spec_val_min').value)
            spec_sat_max = int(self.get_parameter('spec_sat_max').value)
            attach_px = max(1, int(self.get_parameter('spec_attach_px').value))

            specular = cv2.inRange(
                hsv,
                np.array([0, 0, spec_val_min], dtype=np.uint8),
                np.array([179, spec_sat_max, 255], dtype=np.uint8),
            )
            attach_kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (attach_px * 2 + 1, attach_px * 2 + 1),
            )
            primary_neighbourhood = cv2.dilate(primary, attach_kernel, iterations=1)
            specular_near = cv2.bitwise_and(specular, primary_neighbourhood)
            mask = cv2.bitwise_or(primary, specular_near)
        else:
            mask = primary

        open_k = max(1, int(self.get_parameter('kernel_px').value))
        close_k = max(1, int(self.get_parameter('close_kernel_px').value))
        open_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (open_k, open_k)
        )
        close_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (close_k, close_k)
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
        return mask

    # ------------------------------------------------------------------
    def _secondary_yellow_mask(self, bgr: np.ndarray) -> np.ndarray:
        """Build the yellow tape mask used for back_out stop detection.

        Same morphology pipeline as the primary mask but with its own HSV
        thresholds. No specular recovery (yellow tape is rarely confused
        with white in our scenes; turn it on by adding params if needed).
        """
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        hue_low = int(self.get_parameter('yellow_hue_low').value)
        hue_high = int(self.get_parameter('yellow_hue_high').value)
        sat_min = int(self.get_parameter('yellow_sat_min').value)
        val_min = int(self.get_parameter('yellow_val_min').value)

        if hue_low > hue_high:
            mask = cv2.inRange(
                hsv,
                np.array([hue_low, sat_min, val_min], dtype=np.uint8),
                np.array([180, 255, 255], dtype=np.uint8),
            ) | cv2.inRange(
                hsv,
                np.array([0, sat_min, val_min], dtype=np.uint8),
                np.array([hue_high, 255, 255], dtype=np.uint8),
            )
        else:
            mask = cv2.inRange(
                hsv,
                np.array([hue_low, sat_min, val_min], dtype=np.uint8),
                np.array([hue_high, 255, 255], dtype=np.uint8),
            )

        open_k = max(1, int(self.get_parameter('kernel_px').value))
        close_k = max(1, int(self.get_parameter('close_kernel_px').value))
        open_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (open_k, open_k)
        )
        close_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (close_k, close_k)
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
        return mask

    # ------------------------------------------------------------------
    def _classify_blobs(
        self,
        mask: np.ndarray,
        roi_y_offset: int,
        image_height: int,
    ) -> tuple[Optional[dict], Optional[dict]]:
        """Pick a guide blob (vertical, largest) and a stop blob (horizontal).

        Two-pass strategy:

        1. Scan the full mask for the best guide and any horizontal-ish
           stop that is *separate* from the guide blob.
        2. If a guide was found and `recover_stop_from_guide` is on,
           carve out a narrow strip along the guide's principal axis
           and re-scan the residual mask for a stop. This recovers the
           short perpendicular stop strip that, when it touches the
           guide tape, gets merged into the guide's connected
           component and would otherwise be silently consumed.

        Spatial gating: a blob is only considered for the guide / stop
        role if its centroid y (in full-image coords) lies at or below
        the configured `*_search_top_frac` of the image.
        """
        params = self._read_classify_params(image_height)

        # Pass 1: full mask.
        best_guide, best_stop = self._scan_blobs(
            mask, roi_y_offset, params
        )

        # Pass 2: residual scan with the guide's spine masked out.
        if (
            best_guide is not None
            and bool(self.get_parameter('recover_stop_from_guide').value)
        ):
            residual = self._mask_minus_guide_spine(mask, best_guide)
            _, recovered_stop = self._scan_blobs(
                residual, roi_y_offset, params
            )
            if recovered_stop is not None and (
                best_stop is None
                or recovered_stop['cy'] > best_stop['cy']
            ):
                best_stop = recovered_stop

        return best_guide, best_stop

    # ------------------------------------------------------------------
    def _read_classify_params(self, image_height: int) -> dict:
        guide_top_frac = max(
            0.0, min(1.0, float(self.get_parameter('guide_search_top_frac').value))
        )
        stop_top_frac = max(
            0.0, min(1.0, float(self.get_parameter('stop_search_top_frac').value))
        )
        return {
            'min_area': int(self.get_parameter('min_blob_area_px').value),
            'guide_tol': math.radians(
                float(self.get_parameter('guide_yaw_tolerance_deg').value)
            ),
            'stop_tol': math.radians(
                float(self.get_parameter('stop_yaw_tolerance_deg').value)
            ),
            'min_aspect': float(self.get_parameter('min_line_aspect').value),
            'guide_min_cy_full': guide_top_frac * image_height,
            'stop_min_cy_full': stop_top_frac * image_height,
        }

    # ------------------------------------------------------------------
    def _scan_blobs(
        self,
        mask: np.ndarray,
        roi_y_offset: int,
        params: dict,
    ) -> tuple[Optional[dict], Optional[dict]]:
        n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            mask, connectivity=8
        )

        best_guide: Optional[dict] = None
        best_guide_area = 0
        best_stop: Optional[dict] = None

        # label 0 is the background.
        for lbl in range(1, n_labels):
            area = int(stats[lbl, cv2.CC_STAT_AREA])
            if area < params['min_area']:
                continue

            ys, xs = np.where(labels == lbl)
            if xs.size < 5:
                continue
            pts = np.stack([xs, ys], axis=1).astype(np.float32)

            # PCA via cv2.fitLine -- returns (vx, vy, x0, y0) with
            # |(vx, vy)| = 1.
            vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
            vx, vy = float(vx), float(vy)
            x0, y0 = float(x0), float(y0)

            # Aspect from PCA spread along principal axis vs perpendicular.
            d_par = (xs - x0) * vx + (ys - y0) * vy
            d_perp = (xs - x0) * (-vy) + (ys - y0) * vx
            par_std = float(np.std(d_par))
            perp_std = float(np.std(d_perp)) + 1e-6
            aspect = par_std / perp_std
            if aspect < params['min_aspect']:
                continue

            yaw_from_vert = self._line_yaw_from_vertical(vx, vy)
            yaw_from_horz = self._line_yaw_from_horizontal(vx, vy)

            cy = float(centroids[lbl, 1])
            cx = float(centroids[lbl, 0])
            cy_full = cy + roi_y_offset

            if (
                abs(yaw_from_vert) <= params['guide_tol']
                and cy_full >= params['guide_min_cy_full']
                and area > best_guide_area
            ):
                best_guide = {
                    'line': (vx, vy, x0, y0),
                    'cx': cx,
                    'cy': cy,
                    'area': area,
                    'perp_std': perp_std,
                }
                best_guide_area = area

            if (
                abs(yaw_from_horz) <= params['stop_tol']
                and cy_full >= params['stop_min_cy_full']
            ):
                if best_stop is None or cy > best_stop['cy']:
                    best_stop = {
                        'line': (vx, vy, x0, y0),
                        'cx': cx,
                        'cy': cy,
                        'area': area,
                    }

        return best_guide, best_stop

    # ------------------------------------------------------------------
    def _mask_minus_guide_spine(
        self,
        mask: np.ndarray,
        guide: dict,
    ) -> np.ndarray:
        """Return ``mask`` with a strip along the guide's principal axis
        zeroed out, so a guide-attached cross arm can be re-detected."""
        vx, vy, x0, y0 = guide['line']
        perp_std = float(guide.get('perp_std', 1.0))
        spine_min = max(1, int(self.get_parameter('guide_spine_min_width_px').value))
        spine_half_width = max(spine_min, int(round(2.5 * perp_std)))

        h, w = mask.shape[:2]
        yy, xx = np.indices((h, w), dtype=np.float32)
        # Perpendicular distance from each pixel to the guide axis.
        d_perp = (xx - x0) * (-vy) + (yy - y0) * vx
        spine_mask = (np.abs(d_perp) <= spine_half_width).astype(np.uint8) * 255
        return cv2.bitwise_and(mask, cv2.bitwise_not(spine_mask))

    # ------------------------------------------------------------------
    @staticmethod
    def _line_x_at_y(vx: float, vy: float, x0: float, y0: float, y: float) -> float:
        """Solve the parametric line (x0 + t*vx, y0 + t*vy) for x at the given y."""
        if abs(vy) < 1e-6:
            return x0  # nearly horizontal; fall back to centroid x
        t = (y - y0) / vy
        return x0 + t * vx

    @staticmethod
    def _line_yaw_from_vertical(vx: float, vy: float) -> float:
        """Signed angle (rad) between the line direction and image-up.

        Image-up is (0, -1) in pixel coords (y points down). Output is in
        (-pi/2, +pi/2]: 0 = perfectly vertical, positive = tilted clockwise
        in the image (line goes from top-left to bottom-right).
        """
        # Choose the direction sign so vy <= 0 (i.e. the line points "up"
        # in the image) -- removes the 180-degree ambiguity in fitLine.
        if vy > 0:
            vx, vy = -vx, -vy
        return math.atan2(vx, -vy)

    @staticmethod
    def _line_yaw_from_horizontal(vx: float, vy: float) -> float:
        """Signed angle (rad) between the line direction and image-right.

        Output is in (-pi/2, +pi/2]: 0 = perfectly horizontal.
        """
        if vx < 0:
            vx, vy = -vx, -vy
        return math.atan2(vy, vx)

    # ------------------------------------------------------------------
    def _draw_overlay(
        self,
        frame: np.ndarray,
        y0: int,
        y1: int,
        guide: Optional[dict],
        stop: Optional[dict],
        state: LineState,
        desired_col: int,
        guide_top_frac: float = 0.0,
        stop_top_frac: float = 0.0,
        detect_left_frac: float = 0.0,
        detect_right_frac: float = 1.0,
        camera: str = '',
    ) -> np.ndarray:
        out = frame.copy()
        h, w = out.shape[:2]

        # ROI rectangle (thin grey box).
        cv2.rectangle(out, (0, y0), (w - 1, y1 - 1), (96, 96, 96), 1)
        # Desired-guide column (cyan vertical line).
        cv2.line(out, (desired_col, y0), (desired_col, y1 - 1), (255, 255, 0), 1)

        # Per-line search region cutoffs (only the area below each line
        # is eligible). Drawn dim so they don't dominate the view.
        guide_y = int(round(guide_top_frac * h))
        stop_y = int(round(stop_top_frac * h))
        if 0 < guide_y < h:
            cv2.line(out, (0, guide_y), (w - 1, guide_y), (0, 90, 0), 1)
        if 0 < stop_y < h:
            cv2.line(out, (0, stop_y), (w - 1, stop_y), (0, 0, 90), 1)

        # Horizontal detection band. Faint magenta vertical lines mark
        # the left/right cutoffs; pixels outside this band are masked
        # out before connected components.
        x_left = int(round(detect_left_frac * w))
        x_right = int(round(detect_right_frac * w))
        if 0 < x_left < w:
            cv2.line(out, (x_left, y0), (x_left, y1 - 1), (96, 0, 96), 1)
        if 0 < x_right < w:
            cv2.line(out, (x_right, y0), (x_right, y1 - 1), (96, 0, 96), 1)

        if guide is not None:
            vx, vy, lx, ly = guide['line']
            # Draw the guide line across the ROI in full-image coords.
            self._draw_line_in_roi(out, vx, vy, lx, ly, y0, y1, (0, 255, 0), 3)

        if stop is not None:
            vx, vy, lx, ly = stop['line']
            self._draw_line_in_roi(out, vx, vy, lx, ly, y0, y1, (0, 0, 255), 3)

        # HUD text.
        yaw_deg = math.degrees(state.guide_yaw_rad) if state.guide_detected else float('nan')
        lines = [
            f'cam={camera}' if camera else 'cam=?',
            f'guide={"YES" if state.guide_detected else "no "}'
            f' off_px={state.guide_offset_px:+.1f} yaw_deg={yaw_deg:+.1f}',
            f'stop ={"YES" if state.stop_detected else "no "}'
            f' dist_px={state.stop_distance_px:.1f}',
        ]
        for i, line in enumerate(lines):
            cv2.putText(
                out, line, (8, 22 + 22 * i),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA,
            )
            cv2.putText(
                out, line, (8, 22 + 22 * i),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA,
            )
        return out

    @staticmethod
    def _draw_line_in_roi(
        img: np.ndarray,
        vx: float,
        vy: float,
        x0_roi: float,
        y0_roi: float,
        roi_y0: int,
        roi_y1: int,
        colour: tuple[int, int, int],
        thickness: int,
    ) -> None:
        """Draw the parametric line spanning [roi_y0, roi_y1) into the full image."""
        if abs(vy) < 1e-6:
            # Horizontal line -- draw a thin segment at y0_roi shifted up.
            y_full = int(round(y0_roi + roi_y0))
            cv2.line(img, (0, y_full), (img.shape[1] - 1, y_full), colour, thickness)
            return

        y_top = roi_y0
        y_bot = roi_y1 - 1
        # Solve for x at y_top and y_bot in ROI-local then shift.
        t_top = (y_top - roi_y0 - y0_roi) / vy
        t_bot = (y_bot - roi_y0 - y0_roi) / vy
        x_top = int(round(x0_roi + t_top * vx))
        x_bot = int(round(x0_roi + t_bot * vx))
        cv2.line(img, (x_top, y_top), (x_bot, y_bot), colour, thickness)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LineDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
