#!/usr/bin/env python3
"""Read-only CSV recorder for Kinect-front 9-joint camera-optical-frame skeletons.

No robot commands, no images stored, no camera calibration or safety claims.
Requires ROS 2 Jazzy + geometry_msgs + std_msgs (no mediapipe import needed).
"""
import argparse
import csv
import json
import math
import os
import time
from collections import OrderedDict
from datetime import datetime, timezone

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from std_msgs.msg import String

JOINTS = ((0, 'nose'), (11, 'left_shoulder'), (12, 'right_shoulder'),
          (13, 'left_elbow'), (14, 'right_elbow'),
          (15, 'left_wrist'), (16, 'right_wrist'),
          (23, 'left_hip'), (24, 'right_hip'))
HEADERS = ['stamp_sec', 'stamp_nanosec', 'frame_id', 'joint_id', 'joint_name',
           'valid_3d', 'x_m', 'y_m', 'z_m', 'mp_visibility', 'mp_presence',
           'x_norm', 'y_norm', 'detected_2d', 'matched_2d_metadata',
           'arrival_time_utc']


def key_of_stamp(stamp):
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


class Recorder(Node):
    def __init__(self, prefix, path):
        super().__init__('front_pose3d_xyz_recorder')
        self.prefix = prefix.rstrip('/')
        self.path = os.path.abspath(os.path.expanduser(path))
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        self.fd = open(self.path, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.fd)
        self.writer.writerow(HEADERS)
        self.pending_3d = OrderedDict()
        self.recent_2d = OrderedDict()
        self.frames = 0
        self.good = 0
        self.matched = 0
        self.bad_json = 0
        self.poses2d = 0
        self.total_joints = 0
        self.started = time.monotonic()
        self.last_log = self.started
        self.expected_frame = self.prefix.lstrip('/') + '_depth_optical_frame'
        self.frame_warning = 0
        self.create_subscription(PoseArray, self.prefix + '/human_skeleton_depth',
                                 self.on_3d, 10)
        self.create_subscription(String, self.prefix + '/pose2d/landmarks_json',
                                 self.on_2d, 10)
        self.get_logger().info('Read-only 9-joint XYZ recorder: ' + self.path)

    def on_2d(self, msg):
        try:
            payload = json.loads(msg.data)
            if payload.get('schema') != 'research_pose2d_v1':
                raise ValueError('wrong schema')
            key = int(payload['stamp_sec']) * 1000000000 + int(payload['stamp_nanosec'])
        except (ValueError, KeyError, TypeError, json.JSONDecodeError):
            self.bad_json += 1
            return
        self.poses2d += 1
        self.recent_2d[key] = (payload, time.monotonic())
        self.recent_2d.move_to_end(key)
        if key in self.pending_3d:
            pose_msg, arrival, _ = self.pending_3d.pop(key)
            self.write_frame(pose_msg, arrival, payload)
        self.flush_expired()

    def on_3d(self, msg):
        key = key_of_stamp(msg.header.stamp)
        arrival = datetime.now(timezone.utc).isoformat(timespec='milliseconds')
        if key in self.recent_2d:
            payload, _ = self.recent_2d[key]
            self.write_frame(msg, arrival, payload)
        else:
            self.pending_3d[key] = (msg, arrival, time.monotonic())
            self.pending_3d.move_to_end(key)
        self.flush_expired()

    def write_frame(self, msg, arrival, meta=None):
        sec, ns = int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)
        if msg.header.frame_id != self.expected_frame:
            self.frame_warning += 1
        self.frames += 1
        if meta is not None:
            self.matched += 1
            points = {int(p.get('id', -1)): p for p in meta.get('landmarks', [])}
            detected = int(bool(meta.get('detected', False)))
        else:
            points = {}
            detected = ''
        count = 0
        for idx, (jid, name) in enumerate(JOINTS):
            p = msg.poses[idx] if idx < len(msg.poses) else None
            is_valid = (p is not None and p.orientation.w >= 0.5 and
                        all(math.isfinite(v) for v in (p.position.x, p.position.y, p.position.z)) and
                        0.4 <= p.position.z <= 4.5)
            if is_valid:
                count += 1
                xyz = (f'{p.position.x:.7f}', f'{p.position.y:.7f}', f'{p.position.z:.7f}')
            else:
                xyz = ('', '', '')  # never imply invalid joints are at origin
            landmark = points.get(jid, {})
            self.writer.writerow((sec, ns, msg.header.frame_id, jid, name, int(is_valid),
                                  *xyz, landmark.get('visibility', ''),
                                  landmark.get('presence', ''), landmark.get('x_norm', ''),
                                  landmark.get('y_norm', ''), detected, int(meta is not None),
                                  arrival))
        self.total_joints += count
        if count > 0:
            self.good += 1
        if time.monotonic() - self.last_log >= 5:
            self.fd.flush()
            print(f'[XYZ] frames={self.frames}, latest={count}/9, '
                  f'2D metadata matched={self.matched}/{self.frames}', flush=True)
            self.last_log = time.monotonic()

    def flush_expired(self, force=False):
        now = time.monotonic()
        while self.pending_3d:
            key, (msg, arrival, start) = next(iter(self.pending_3d.items()))
            if not force and now - start < 2.0 and len(self.pending_3d) < 100:
                break
            self.pending_3d.pop(key)
            self.write_frame(msg, arrival, None)
        while self.recent_2d:
            key, (_, start) = next(iter(self.recent_2d.items()))
            if not force and now - start < 4.0 and len(self.recent_2d) < 150:
                break
            self.recent_2d.pop(key)

    def finish(self):
        self.flush_expired(force=True)
        self.fd.flush()
        self.fd.close()
        dt = max(time.monotonic() - self.started, 0.001)
        print('\n=== FRONT KINECT XYZ RECORDER ===')
        print(f'Runtime: {dt:.1f}s; Pose3D frames: {self.frames} (~{self.frames/dt:.2f}/s)')
        print(f'2D messages: {self.poses2d}; 3D with 2D metadata: {self.matched}/{self.frames}')
        print(f'Valid XYZ joints/frame: {self.total_joints/max(self.frames, 1):.2f}/9')
        print(f'Frames with >=1 joint: {self.good}/{self.frames}; wrong frame IDs: {self.frame_warning}')
        print(f'Bad 2D JSON: {self.bad_json}')
        print('NOTE: 2D visibility/presence are MediaPipe estimates, not metric XYZ accuracy.')
        print('NOTE: camera optical frame only; no UR3 base_link transform or safety validation.')
        print(f'CSV: {self.path}', flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seconds', type=float, default=30.0)
    ap.add_argument('--prefix', default='/kinect_front')
    ap.add_argument('--csv', default='~/ur3_hri_ws/data/pose3d/front_static_01_xyz.csv')
    args = ap.parse_args()
    if args.seconds <= 0:
        ap.error('--seconds must be positive')
    rclpy.init()
    node = Recorder(args.prefix, args.csv)
    try:
        while rclpy.ok() and time.monotonic() - node.started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.flush_expired()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
