"""Read-only ROS 2 MediaPipe Pose tracking from one Kinect v2 RGB topic.

Output uses normalized 2D image coordinates; NEVER interpret it as base_link XYZ.
"""

import json
import os
import time
from typing import Optional

import cv2
import numpy as np
import mediapipe as mp
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Landmark IDs used in the predecessor's 9-joint Fusion prototype.
LANDMARK_IDS = (0, 11, 12, 13, 14, 15, 16, 23, 24)
LINES = ((11, 12), (11, 13), (13, 15), (12, 14), (14, 16),
         (11, 23), (12, 24), (23, 24))


def bgra_or_rgba_to_rgb(msg: Image, max_width: int) -> np.ndarray:
    """Decode a bgra8/rgba8 sensor_msgs/Image, honoring row stride."""
    if msg.encoding not in ('bgra8', 'rgba8'):
        raise ValueError(f"Expected bgra8 or rgba8; received {msg.encoding!r}")
    row_bytes = msg.width * 4
    if msg.step < row_bytes or len(msg.data) < msg.height * msg.step:
        raise ValueError("Image buffer smaller than height*step or invalid row stride")
    raw = np.frombuffer(msg.data, dtype=np.uint8, count=msg.height * msg.step)
    pixels = raw.reshape(msg.height, msg.step)[:, :row_bytes].reshape(msg.height, msg.width, 4)
    if 0 < max_width < msg.width:
        height = round(msg.height * max_width / msg.width)
        pixels = cv2.resize(pixels, (max_width, height), interpolation=cv2.INTER_AREA)
    mode = cv2.COLOR_BGRA2RGB if msg.encoding == 'bgra8' else cv2.COLOR_RGBA2RGB
    return np.ascontiguousarray(cv2.cvtColor(pixels, mode))


def header_ms(msg: Image) -> int:
    return int(msg.header.stamp.sec) * 1000 + int(msg.header.stamp.nanosec) // 1_000_000


class Pose2DNode(Node):
    def __init__(self) -> None:
        super().__init__('pose_tracker_2d')
        self.declare_parameter('image_topic', '/kinect_front/color/image_raw')
        self.declare_parameter('output_prefix', '/kinect_front/pose2d')
        self.declare_parameter('model_path', os.path.expanduser('~/ur3_hri_ws/models/pose_landmarker_lite.task'))
        self.declare_parameter('max_width', 640)
        self.declare_parameter('max_fps', 6.0)
        self.declare_parameter('min_visibility', 0.4)
        self.declare_parameter('publish_overlay', True)

        topic = str(self.get_parameter('image_topic').value)
        prefix = str(self.get_parameter('output_prefix').value).rstrip('/')
        model = os.path.expanduser(str(self.get_parameter('model_path').value))
        self.max_width = max(160, int(self.get_parameter('max_width').value))
        self.max_fps = max(0.2, float(self.get_parameter('max_fps').value))
        self.min_visibility = float(self.get_parameter('min_visibility').value)
        self.publish_overlay = bool(self.get_parameter('publish_overlay').value)
        if not os.path.isfile(model):
            raise FileNotFoundError(f'MediaPipe task model missing: {model}')

        options = mp.tasks.vision.PoseLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=model),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=0.5,
            min_pose_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.tracker = mp.tasks.vision.PoseLandmarker.create_from_options(options)
        self.json_pub = self.create_publisher(String, prefix + '/landmarks_json', 10)
        self.status_pub = self.create_publisher(String, prefix + '/status', 10)
        self.overlay_pub = self.create_publisher(Image, prefix + '/overlay', qos_profile_sensor_data)
        self.sub = self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)
        self.heartbeat = self.create_timer(1.0, self.on_heartbeat)
        self.last_arrival: Optional[float] = None
        self.last_processed: float = -1e9
        self.last_mp_ms: int = -1
        self.latest_detected: bool = False
        self.frames_seen: int = 0
        self.frames_processed: int = 0
        self.get_logger().info(f'Pose2D read-only input={topic}, output={prefix}, model={model}, max_fps={self.max_fps}')

    def on_image(self, msg: Image) -> None:
        now = time.monotonic()
        self.frames_seen += 1
        self.last_arrival = now
        if now - self.last_processed < 1.0 / self.max_fps:
            return
        self.last_processed = now
        try:
            rgb = bgra_or_rgba_to_rgb(msg, self.max_width)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            ms = header_ms(msg)
            if ms <= 0:
                ms = int(time.time() * 1000)
            ms = max(ms, self.last_mp_ms + 1)
            self.last_mp_ms = ms
            result = self.tracker.detect_for_video(mp_image, ms)
        except Exception as exc:
            self.get_logger().error(f'Pose processing failed: {type(exc).__name__}: {exc}')
            self.latest_detected = False
            return
        self.frames_processed += 1
        detected = bool(result.pose_landmarks)
        self.latest_detected = detected
        landmarks = result.pose_landmarks[0] if detected else []
        keypoints = []
        for idx in LANDMARK_IDS:
            if idx >= len(landmarks):
                continue
            item = landmarks[idx]
            keypoints.append({
                'id': idx, 'x_norm': float(item.x), 'y_norm': float(item.y),
                'visibility': float(item.visibility or 0.0),
                'presence': float(item.presence or 0.0),
            })
        payload = {
            'schema': 'research_pose2d_v1',
            'source_image_topic': str(self.get_parameter('image_topic').value),
            'stamp_sec': int(msg.header.stamp.sec),
            'stamp_nanosec': int(msg.header.stamp.nanosec),
            'frame_id': msg.header.frame_id,
            'image_width': int(msg.width), 'image_height': int(msg.height),
            'detected': detected, 'landmarks': keypoints,
            'warning': 'NORMALIZED_2D_ONLY_NOT_METERS_NOT_BASE_LINK',
        }
        self.json_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))
        if self.publish_overlay and self.overlay_pub.get_subscription_count() > 0:
            annotated = self.make_overlay(rgb, landmarks)
            overlay_msg = Image()
            overlay_msg.header = msg.header
            overlay_msg.height, overlay_msg.width = annotated.shape[:2]
            overlay_msg.encoding = 'bgr8'
            overlay_msg.is_bigendian = 0
            overlay_msg.step = int(overlay_msg.width * 3)
            overlay_msg.data = annotated.tobytes()
            self.overlay_pub.publish(overlay_msg)

    def make_overlay(self, rgb: np.ndarray, landmarks) -> np.ndarray:
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        h, w = bgr.shape[:2]
        visible = {}
        for idx in LANDMARK_IDS:
            if idx >= len(landmarks):
                continue
            p = landmarks[idx]
            if float(p.visibility or 0.0) < self.min_visibility:
                continue
            if not (0.0 <= p.x <= 1.0 and 0.0 <= p.y <= 1.0):
                continue
            visible[idx] = (int(p.x * (w - 1)), int(p.y * (h - 1)))
        for i, j in LINES:
            if i in visible and j in visible:
                cv2.line(bgr, visible[i], visible[j], (20, 220, 255), 2)
        for i, p in visible.items():
            cv2.circle(bgr, p, 5, (0, 255, 0), -1)
            cv2.putText(bgr, str(i), (p[0] + 4, p[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        cv2.putText(bgr, '2D ONLY - NOT CALIBRATED FOR UR3', (10, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return bgr

    def on_heartbeat(self) -> None:
        age = None if self.last_arrival is None else time.monotonic() - self.last_arrival
        if age is None or age > 1.0:
            state = 'STALE_NO_FRAMES'
        elif self.latest_detected:
            state = 'OBSERVING_PERSON_2D'
        else:
            state = 'NO_POSE_DETECTED_UNCERTAIN'
        self.status_pub.publish(String(data=json.dumps({
            'state': state,
            'seconds_since_image': None if age is None else round(age, 3),
            'frames_seen': self.frames_seen,
            'frames_processed': self.frames_processed,
            'not_safety_certified': True,
        })))

    def close(self) -> None:
        self.tracker.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = Pose2DNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
