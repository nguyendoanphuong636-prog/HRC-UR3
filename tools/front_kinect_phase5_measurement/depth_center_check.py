#!/usr/bin/env python3
"""Read-only depth ROI measurement: flat board centered on Kinect Front IR depth image.

Uses 32FC1 rectified depth in METERS and RELIABLE QoS (phase-4 trial).
Measuring the optical Z axis from a hand tape is approximate, not full calibration.
No raw images saved; does not command the UR3.
"""
import argparse
import math
import statistics as st
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                        HistoryPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import Image, CameraInfo


class TargetCheck(Node):
    def __init__(self, prefix, u, v, radius):
        super().__init__('front_depth_center_check')
        self.prefix = prefix.rstrip('/')
        self.u, self.v, self.radius = u, v, radius
        self.frames = 0
        self.frame_medians = []
        self.failed = 0
        self.intrinsics = None
        self.started = time.monotonic()
        depth_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                               reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Image, self.prefix + '/depth_rect/image_raw',
                                 self.depth_callback, depth_qos)
        self.create_subscription(CameraInfo, self.prefix + '/depth_rect/camera_info',
                                 self.info_callback, qos_profile_sensor_data)
        self.get_logger().info(f'Read-only depth ROI: center=({u},{v}), radius={radius} pixels')

    def info_callback(self, msg):
        if msg.k[0] > 0 and msg.k[4] > 0:
            self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def depth_callback(self, msg):
        self.frames += 1
        if (msg.encoding != '32FC1' or msg.is_bigendian or msg.step < msg.width * 4 or
                len(msg.data) < msg.height * msg.step or
                not (0 <= self.u < msg.width and 0 <= self.v < msg.height)):
            self.failed += 1
            return
        vals = []
        for v in range(max(0, self.v-self.radius), min(msg.height, self.v+self.radius+1)):
            for u in range(max(0, self.u-self.radius), min(msg.width, self.u+self.radius+1)):
                z = struct.unpack_from('<f', msg.data, int(v*msg.step + u*4))[0]
                if math.isfinite(z) and 0.4 <= z <= 4.5:
                    vals.append(z)
        total_roi = (min(msg.width, self.u+self.radius+1) - max(0, self.u-self.radius)) * \
                    (min(msg.height, self.v+self.radius+1) - max(0, self.v-self.radius))
        if len(vals) < max(6, int(total_roi * 0.5)):
            self.failed += 1
            return
        med = st.median(vals)
        ordered = sorted(vals)
        spread = ordered[int((len(ordered)-1)*.9)] - ordered[int((len(ordered)-1)*.1)]
        if spread > 0.08:
            self.failed += 1
            return
        self.frame_medians.append(med)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--prefix', default='/kinect_front')
    ap.add_argument('--seconds', type=float, default=12.0)
    ap.add_argument('--u', type=int, default=256, help='pixel x on depth 512x424')
    ap.add_argument('--v', type=int, default=212, help='pixel y on depth 512x424')
    ap.add_argument('--radius', type=int, default=5)
    ap.add_argument('--reference-m', type=float, default=None,
                    help='optional approximate physical Z distance in meters')
    args = ap.parse_args()
    if args.seconds <= 0 or args.radius < 2:
        ap.error('seconds must be positive and radius at least 2')
    rclpy.init()
    node = TargetCheck(args.prefix, args.u, args.v, args.radius)
    try:
        while rclpy.ok() and time.monotonic() - node.started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        vals = node.frame_medians
        print('\n=== FRONT KINECT RECTIFIED DEPTH CENTER CHECK ===')
        print(f'Images received={node.frames}; usable ROI samples={len(vals)}; rejected={node.failed}')
        if vals:
            med = st.median(vals)
            sd_mm = st.stdev(vals)*1000 if len(vals)>1 else 0.0
            mad_mm = st.median([abs(x-med) for x in vals])*1000
            print(f'ROI Z median={med:.4f} m; temporal std={sd_mm:.1f} mm; '
                  f'MAD={mad_mm:.1f} mm; min/max=({min(vals):.4f},{max(vals):.4f}) m')
            if node.intrinsics:
                fx, fy, cx, cy = node.intrinsics
                print(f'Camera K: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}')
                print(f'Approximate ROI XYZ=({(args.u-cx)*med/fx:.4f}, '
                      f'{(args.v-cy)*med/fy:.4f}, {med:.4f}) m')
            if args.reference_m is not None:
                print(f'Approximate measured-reference Z difference: '
                      f'{1000*(med-args.reference_m):+.1f} mm')
                print('WARNING: reference includes tape placement/orientation/IR optical-center uncertainty.')
        else:
            print('No usable measurements. Center a non-reflective flat board in depth view, '
                  'check topic/QoS, and keep ROI clear of edges.')
        print('Not a calibrated absolute accuracy test; no robot safety use.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
