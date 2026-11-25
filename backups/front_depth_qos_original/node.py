"""Read-only conversion: 2D keypoints + synchronized registered depth -> camera optical XYZ.

Output is NOT calibrated to UR3 base_link, and is NOT a safety function.
"""
import json
import math
import time
from collections import OrderedDict
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from .core import JOINT_IDS, reconstruct


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def depth_from_msg(msg: Image):
    if msg.encoding != '32FC1' or msg.is_bigendian or msg.width < 2 or msg.height < 2:
        raise ValueError('Expected 32FC1 little-endian depth, minimum 2x2')
    if msg.step < msg.width * 4 or len(msg.data) < msg.height * msg.step:
        raise ValueError('Invalid depth stride/length')
    raw = np.frombuffer(msg.data, dtype='<f4', count=(msg.step // 4) * msg.height)
    return raw.reshape(msg.height, msg.step // 4)[:, :msg.width].copy()


class Pose3DNode(Node):
    def __init__(self):
        super().__init__('pose_tracker_3d')
        self.declare_parameter('prefix', '/kinect_front')
        self.declare_parameter('min_visibility', 0.55)
        self.declare_parameter('min_presence', 0.50)
        p = str(self.get_parameter('prefix').value).rstrip('/')
        self.min_vis = float(self.get_parameter('min_visibility').value)
        self.min_pres = float(self.get_parameter('min_presence').value)
        self.expected_frame = p.lstrip('/') + '_depth_optical_frame'
        self.cache = OrderedDict()
        self.pending = OrderedDict()
        self.K: Optional[list] = None
        self.last_seen = None
        self.state = 'STARTING'
        self.output = self.create_publisher(PoseArray, p + '/human_skeleton_depth', 10)
        self.status = self.create_publisher(String, p + '/pose3d/status', 10)
        self.create_subscription(Image, p + '/depth_rect/image_raw', self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, p + '/depth_rect/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(String, p + '/pose2d/landmarks_json', self.on_pose, 10)
        self.create_timer(1.0, self.heartbeat)
        self.get_logger().info(f'Camera-optical-frame 3D only. Expects frame {self.expected_frame}')

    def report(self, state, **kwargs):
        self.state = state
        self.status.publish(String(data=json.dumps({'state': state, **kwargs,
                        'research_only_not_safety_certified': True})))

    def on_info(self, msg):
        if msg.header.frame_id != self.expected_frame or msg.width != 512 or msg.height != 424:
            self.report('INVALID_CAMERA_INFO_FRAME_OR_SIZE')
            return
        if not all(math.isfinite(float(x)) for x in msg.k) or msg.k[0] <= 0 or msg.k[4] <= 0:
            self.report('INVALID_INTRINSICS')
            return
        self.K = list(msg.k)

    def on_depth(self, msg):
        if msg.header.frame_id != self.expected_frame or msg.width != 512 or msg.height != 424:
            self.report('INVALID_DEPTH_FRAME_OR_SIZE')
            return
        try:
            data = depth_from_msg(msg)
        except ValueError as err:
            self.report('INVALID_DEPTH_IMAGE', error=str(err))
            return
        key = stamp_ns(msg.header.stamp)
        self.cache[key] = (data, msg.header.frame_id)
        self.cache.move_to_end(key)
        while len(self.cache) > 48:
            self.cache.popitem(last=False)
        if key in self.pending:
            payload = self.pending.pop(key)
            self.convert(key, payload)

    def on_pose(self, msg):
        try:
            payload = json.loads(msg.data)
            if payload.get('schema') != 'research_pose2d_v1':
                raise ValueError('Unexpected 2D schema')
            key = int(payload['stamp_sec']) * 1_000_000_000 + int(payload['stamp_nanosec'])
        except (ValueError, TypeError, KeyError, json.JSONDecodeError):
            self.report('INVALID_2D_PAYLOAD')
            return
        if payload.get('frame_id') != self.expected_frame or payload.get('image_width') != 512 or payload.get('image_height') != 424:
            self.report('MISMATCHED_2D_FRAME_OR_SIZE')
            return
        self.last_seen = time.monotonic()
        if key in self.cache:
            self.convert(key, payload)
        else:
            self.pending[key] = payload
            while len(self.pending) > 12:
                self.pending.popitem(last=False)

    def convert(self, key, payload):
        if self.K is None or key not in self.cache:
            self.report('WAITING_FOR_DEPTH_INTRINSICS_OR_MATCH')
            return
        depth_m, frame = self.cache[key]
        points = reconstruct(payload.get('landmarks', []) if payload.get('detected', False) else [],
                             depth_m, self.K, self.min_vis, self.min_pres)
        arr = PoseArray()
        arr.header.frame_id = frame
        arr.header.stamp.sec = key // 1_000_000_000
        arr.header.stamp.nanosec = key % 1_000_000_000
        for jid in JOINT_IDS:
            pose = Pose()
            if jid in points:
                pose.position.x, pose.position.y, pose.position.z = points[jid]
                pose.orientation.w = 1.0  # 'valid' convention of research fusion core
            else:
                pose.position.x = pose.position.y = pose.position.z = float('nan')
                pose.orientation.w = 0.0
            arr.poses.append(pose)
        self.output.publish(arr)
        self.report('OBSERVING_3D_CAMERA_FRAME' if points else 'NO_VALID_3D_JOINTS',
                    detected_2d=bool(payload.get('detected')), valid_joints=len(points),
                    frame_id=frame, camera_to_ur3_calibrated=False)

    def heartbeat(self):
        if self.last_seen is None or time.monotonic() - self.last_seen > 1.0:
            self.report('STALE_2D_INPUT')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Pose3DNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
