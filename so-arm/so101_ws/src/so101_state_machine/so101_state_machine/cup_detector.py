#!/usr/bin/env python3
"""
Red cup detector.

Subscribes to an RGB-D camera, detects a red cup via HSV colour filtering,
back-projects the centroid to 3-D using the latest-cached depth image and
CameraInfo intrinsics, transforms into target_frame (default base_link), and
publishes the result as geometry_msgs/PoseStamped on /detected_cup_pose.

RGB drives the detection callback; depth is cached from a separate subscriber
(no ApproximateTimeSynchronizer — Isaac Sim timestamp skews > 200 ms break it).
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import cv2
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform with tf2 Buffer


# ─── QoS profiles ────────────────────────────────────────────────────────────

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# Isaac Sim's depth publisher uses RELIABLE QoS; a BEST_EFFORT subscriber can
# silently drop the large (~1.2 MB) depth frames under Cyclone. Use RELIABLE
# for depth to guarantee delivery.
DEPTH_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


# ─── HSV / morphology constants ──────────────────────────────────────────────

# Red wraps around the hue circle, so we need two inRange bands.
HSV_LOWER_RED_1 = np.array([0, 80, 40])
HSV_UPPER_RED_1 = np.array([12, 255, 255])
HSV_LOWER_RED_2 = np.array([160, 80, 40])
HSV_UPPER_RED_2 = np.array([180, 255, 255])

# Close kernel bridges cup handle into the body; open kernel removes noise.
CLOSE_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (31, 31))
OPEN_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

# 5x5 median depth sample around the centroid.
DEPTH_PATCH_RADIUS = 2

# Fallback intrinsics for Isaac Sim 640x480 camera until CameraInfo arrives.
FALLBACK_FX = FALLBACK_FY = 554.3
FALLBACK_CX = 320.0
FALLBACK_CY = 240.0

# Minimum depth in metres to count a pixel as valid (rejects zero/invalid samples).
MIN_VALID_DEPTH_M = 0.01

# Throttled-log period.
LOG_PERIOD_SEC = 2.0


class CupDetector(Node):
    def __init__(self):
        super().__init__("cup_detector")

        # ── parameters ───────────────────────────────────────────────
        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("output_frame", "camera_color_optical_frame")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("min_area_px", 150)

        rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
        depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self._output_frame = self.get_parameter("output_frame").get_parameter_value().string_value
        self._target_frame = self.get_parameter("target_frame").get_parameter_value().string_value
        self._min_area = self.get_parameter("min_area_px").get_parameter_value().integer_value

        self._bridge = CvBridge()
        self._pin_cam = PinholeCameraModel()
        self._have_info = False
        self._latest_depth: np.ndarray | None = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(CameraInfo, info_topic, self._on_camera_info, 1)
        self.create_subscription(Image, depth_topic, self._on_depth, DEPTH_QOS)
        self.create_subscription(Image, rgb_topic, self._on_rgb, SENSOR_QOS)

        self._pose_pub = self.create_publisher(PoseStamped, "/detected_cup_pose", 10)
        self._marker_pub = self.create_publisher(Marker, "/detected_cup_marker", 10)
        self._debug_pub = self.create_publisher(Image, "/cup_detector/debug_image", 1)

        self._frame_count = 0
        self._last_log_time = 0.0
        self.get_logger().info(
            f"CupDetector started  rgb={rgb_topic}  depth={depth_topic}  target={self._target_frame}"
        )

    # ─── Subscriber callbacks ────────────────────────────────────────
    def _on_camera_info(self, msg: CameraInfo):
        if self._have_info:
            return
        self._pin_cam.fromCameraInfo(msg)
        self._have_info = True
        self.get_logger().info(
            f"Camera intrinsics received: fx={self._pin_cam.fx():.1f} fy={self._pin_cam.fy():.1f}"
        )

    def _on_depth(self, msg: Image):
        try:
            self._latest_depth = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            ).astype(np.float32)
        except Exception as exc:
            self.get_logger().error(f"depth CvBridge: {exc}")

    def _on_rgb(self, rgb_msg: Image):
        self._frame_count += 1
        try:
            rgb = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"rgb CvBridge: {exc}")
            return

        if self._latest_depth is None:
            self._log_throttled(
                f"frames={self._frame_count}  waiting for first depth frame"
            )
            return

        mask, mask_pixels = self._build_red_mask(rgb)

        contour = self._largest_contour(mask)
        if contour is None:
            self._log_throttled(
                f"frames={self._frame_count}  mask_px={mask_pixels}  no cup contour"
            )
            self._publish_debug(rgb, mask, note=f"no cup (mask={mask_pixels}px)")
            return
        area = int(cv2.contourArea(contour))

        centroid = self._centroid(contour)
        if centroid is None:
            return
        u, v = centroid

        z = self._depth_at(u, v)
        if z is None:
            self._log_throttled(
                f"frames={self._frame_count}  cup@({u},{v}) but no valid depth"
            )
            self._publish_debug(rgb, mask, contour=contour, u=u, v=v,
                                note="no valid depth")
            return

        x, y = self._back_project(u, v, z)

        pose_out = self._transform_to_target(x, y, z)
        if pose_out is None:
            self._publish_debug(rgb, mask, contour=contour, u=u, v=v, z=z,
                                note=f"TF to {self._target_frame} failed")
            return

        self._pose_pub.publish(pose_out)
        self._publish_marker(pose_out)
        self._publish_debug(rgb, mask, contour=contour, u=u, v=v, z=z)

        p = pose_out.pose.position
        self._log_throttled(
            f"cup@({u},{v})  area={area}px  "
            f"optical=({x:.3f},{y:.3f},{z:.3f}) "
            f"{self._target_frame}=({p.x:.3f},{p.y:.3f},{p.z:.3f})"
        )

    # ─── Detection helpers ───────────────────────────────────────────
    @staticmethod
    def _build_red_mask(rgb: np.ndarray) -> tuple[np.ndarray, int]:
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, HSV_LOWER_RED_1, HSV_UPPER_RED_1),
            cv2.inRange(hsv, HSV_LOWER_RED_2, HSV_UPPER_RED_2),
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, CLOSE_KERNEL)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, OPEN_KERNEL)
        return mask, int(cv2.countNonZero(mask))

    def _largest_contour(self, mask: np.ndarray):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        return largest if cv2.contourArea(largest) >= self._min_area else None

    @staticmethod
    def _centroid(contour) -> tuple[int, int] | None:
        M = cv2.moments(contour)
        if M["m00"] < 1e-6:
            return None
        return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])

    def _depth_at(self, u: int, v: int) -> float | None:
        depth = self._latest_depth
        h, w = depth.shape[:2]
        if u >= w or v >= h:
            return None
        r = DEPTH_PATCH_RADIUS
        u0, u1 = max(0, u - r), min(w, u + r + 1)
        v0, v1 = max(0, v - r), min(h, v + r + 1)
        patch = depth[v0:v1, u0:u1]
        valid = patch[np.isfinite(patch) & (patch > MIN_VALID_DEPTH_M)]
        return None if valid.size == 0 else float(np.median(valid))

    def _back_project(self, u: int, v: int, z: float) -> tuple[float, float]:
        if self._have_info:
            ray = self._pin_cam.projectPixelTo3dRay((u, v))
            return (ray[0] / ray[2] * z, ray[1] / ray[2] * z)
        return (
            (u - FALLBACK_CX) * z / FALLBACK_FX,
            (v - FALLBACK_CY) * z / FALLBACK_FY,
        )

    def _transform_to_target(self, x: float, y: float, z: float) -> PoseStamped | None:
        pose_opt = PoseStamped()
        # time=0 → latest TF (avoids sim-time vs wall-time stamp mismatch).
        pose_opt.header.stamp = rclpy.time.Time().to_msg()
        pose_opt.header.frame_id = self._output_frame
        pose_opt.pose.position.x = x
        pose_opt.pose.position.y = y
        pose_opt.pose.position.z = z
        pose_opt.pose.orientation.w = 1.0
        try:
            return self._tf_buffer.transform(
                pose_opt, self._target_frame, timeout=Duration(seconds=0.5)
            )
        except Exception as exc:
            self._log_throttled(f"TF {self._output_frame} -> {self._target_frame} failed: {exc}")
            return None

    # ─── Publishers ──────────────────────────────────────────────────
    def _publish_marker(self, pose: PoseStamped):
        marker = Marker()
        marker.header = pose.header
        marker.ns = "cup"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.06
        marker.color.r = 1.0
        marker.color.a = 0.9
        self._marker_pub.publish(marker)

    def _publish_debug(self, rgb, mask, contour=None, u=None, v=None, z=None, note=""):
        debug = rgb.copy()
        mask_bgr = np.zeros_like(rgb)
        mask_bgr[..., 2] = mask
        debug = cv2.addWeighted(debug, 1.0, mask_bgr, 0.4, 0)
        if contour is not None:
            cv2.drawContours(debug, [contour], -1, (0, 255, 0), 2)
        if u is not None and v is not None:
            cv2.circle(debug, (u, v), 6, (0, 0, 255), -1)
            if z is not None:
                cv2.putText(debug, f"z={z:.3f}m", (u + 8, v),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        if note:
            cv2.putText(debug, note, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        self._debug_pub.publish(self._bridge.cv2_to_imgmsg(debug, encoding="bgr8"))

    # ─── Logging ─────────────────────────────────────────────────────
    def _log_throttled(self, msg: str):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_log_time >= LOG_PERIOD_SEC:
            self.get_logger().info(msg)
            self._last_log_time = now


# ─── Entry point ─────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = CupDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
