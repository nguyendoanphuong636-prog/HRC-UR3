import unittest
import numpy as np
from ur3_pose_tracking_3d.core import median_depth_at, unproject, reconstruct

class TestPose3D(unittest.TestCase):
    def test_unprojection(self):
        self.assertEqual(unproject(260, 212, 2.0, 365, 365, 260, 212), (0.0, 0.0, 2.0))

    def test_median(self):
        z = np.full((424, 512), 2.0, np.float32)
        z[212, 260] = np.nan
        self.assertAlmostEqual(median_depth_at(z,260,212), 2.0)

    def test_bad_edge(self):
        z = np.full((424,512), np.nan, np.float32)
        self.assertIsNone(median_depth_at(z, 1, 1))

    def test_joint(self):
        z = np.full((424,512), 2.0, np.float32)
        j = [{'id': 11, 'x_norm':260/511, 'y_norm':212/423, 'visibility':0.9,'presence':0.9}]
        pts = reconstruct(j, z, [365,0,260,0,365,212,0,0,1])
        self.assertIn(11,pts)
        self.assertAlmostEqual(pts[11][2],2.)

if __name__=='__main__': unittest.main()
