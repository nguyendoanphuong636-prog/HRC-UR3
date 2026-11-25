"""ROS 2 read-only PoseArray fusion with strict timestamp and frame checks."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String
from .fusion_core import BODY_IDS, FULL_IDS, fuse, inputs_usable


class FusionNode(Node):
    def __init__(self):
        super().__init__('dual_kinect_fusion')
        for name, val in [
            ('front_topic', '/kinect_front/human_skeleton_base'),
            ('back_topic', '/kinect_back/human_skeleton_base'),
            ('output_topic', '/human_skeleton_base'),
            ('status_topic', '/human_skeleton_fusion_status'),
            ('target_frame', 'base_link'),
            ('fusion_rate_hz', 20.0),
            ('max_input_age_sec', 0.35),
            ('max_stamp_skew_sec', 0.10),
            ('max_merge_dist', 0.20),
            ('tracked_joint_ids', list(BODY_IDS)),
        ]:
            self.declare_parameter(name, val)
        p = lambda n: self.get_parameter(n).value
        self.target = str(p('target_frame'))
        self.ids = tuple(int(v) for v in p('tracked_joint_ids'))
        if len(self.ids) != len(set(self.ids)) or not set(self.ids).issubset(FULL_IDS):
            raise ValueError('tracked_joint_ids must be unique MediaPipe IDs from original schema')
        self.max_age = float(p('max_input_age_sec'))
        self.max_skew = float(p('max_stamp_skew_sec'))
        self.merge_dist = float(p('max_merge_dist'))
        rate = float(p('fusion_rate_hz'))
        if rate <= 0 or self.max_age <= 0 or self.max_skew < 0:
            raise ValueError('Invalid fusion timing parameters')
        self.latest = {'front': None, 'back': None}
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PoseArray, str(p('front_topic')), lambda m: self.receive('front', m), qos)
        self.create_subscription(PoseArray, str(p('back_topic')), lambda m: self.receive('back', m), qos)
        self.out = self.create_publisher(PoseArray, str(p('output_topic')), 10)
        self.status = self.create_publisher(String, str(p('status_topic')), 10)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info('Read-only fusion ready. Target frame: %s' % self.target)

    def receive(self, which, msg):
        self.latest[which] = msg

    @staticmethod
    def stamp_s(msg):
        if msg is None:
            return None
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def decode(self, msg):
        if msg is None or msg.header.frame_id != self.target:
            return {}, False
        # Preserve source schema: 9 body points or 51 body+hand points.
        if len(msg.poses) == len(BODY_IDS):
            order = BODY_IDS
        elif len(msg.poses) == len(FULL_IDS):
            order = FULL_IDS
        else:
            return {}, False
        output = {}
        for jid, pose in zip(order, msg.poses):
            pos = pose.position
            if pose.orientation.w <= 0:
                continue
            pt = (pos.x, pos.y, pos.z)
            if all(math.isfinite(x) for x in pt):
                output[jid] = pt
        return output, True

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        fmsg, bmsg = self.latest['front'], self.latest['back']
        f_ok, b_ok, skew = inputs_usable(self.stamp_s(fmsg), self.stamp_s(bmsg),
                                         now, self.max_age, self.max_skew)
        front, fvalid = self.decode(fmsg) if f_ok else ({}, False)
        back, bvalid = self.decode(bmsg) if b_ok else ({}, False)
        # Fail closed in the *output data*: if two camera stamps disagree, do not fuse them.
        # Neither output nor status is a certified protective stop.
        if not skew:
            front, back = {}, {}
        if not fvalid and f_ok:
            front = {}
        if not bvalid and b_ok:
            back = {}
        fused, mode = fuse(front, back, self.ids, self.merge_dist)
        out = PoseArray()
        out.header.frame_id = self.target
        out.header.stamp = self.get_clock().now().to_msg()
        for jid in self.ids:
            pt = Pose()
            pt.orientation.w = 1.0
            if jid in fused:
                pt.position.x, pt.position.y, pt.position.z = fused[jid]
            else:
                pt.position.x = float('nan')
                pt.position.y = float('nan')
                pt.position.z = float('nan')
            out.poses.append(pt)
        self.out.publish(out)
        info = String()
        status = 'STALE_OR_UNSYNCED' if not skew or not fused else 'OBSERVING'
        info.data = (f'status={status} mode={mode} front_joints={len(front)} '
                     f'back_joints={len(back)} fused={len(fused)} '
                     f'frame_ok_front={fvalid} frame_ok_back={bvalid} stamp_skew_ok={skew}')
        self.status.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
