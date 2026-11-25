"""Generate two synchronized *synthetic* skeleton sources for testing, not actual Kinect data."""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from .fusion_core import BODY_IDS


class Synthetic(Node):
    def __init__(self):
        super().__init__('synthetic_dual_kinect')
        self.front = self.create_publisher(PoseArray, '/kinect_front/human_skeleton_base', 10)
        self.back = self.create_publisher(PoseArray, '/kinect_back/human_skeleton_base', 10)
        self.t = 0
        self.create_timer(0.1, self.tick)

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        for side, pub in [('front', self.front), ('back', self.back)]:
            msg = PoseArray()
            msg.header.stamp = stamp
            msg.header.frame_id = 'base_link'
            for j, jid in enumerate(BODY_IDS):
                p = Pose()
                p.orientation.w = 1.0
                p.position.x = 0.7 + j * 0.01 + 0.02 * math.sin(self.t * 0.1)
                p.position.y = 0.1 + (0.005 if side == 'back' else 0.0)
                p.position.z = 1.0 + j * 0.03
                if side == 'front' and jid == 15:
                    p.position.x = float('nan')  # back should fill wrist
                msg.poses.append(p)
            pub.publish(msg)
        self.t += 1


def main(args=None):
    rclpy.init(args=args)
    node = Synthetic()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
