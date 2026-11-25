import math
from ur3_human_fusion.fusion_core import BODY_IDS, FULL_IDS, fuse, inputs_usable


def test_original_schema_and_missing_joint_fill():
    assert len(BODY_IDS) == 9 and len(FULL_IDS) == 51
    merged, mode = fuse({11: (0, 0, 1)}, {11: (0.1, 0, 1), 15: (0.5, 0, 1)})
    assert merged[11] == (0.05, 0.0, 1.0)
    assert merged[15] == (0.5, 0, 1)
    assert mode == 'FILL_FROM_BACK'


def test_divergence_prefers_front():
    merged, mode = fuse({11: (0, 0, 0)}, {11: (1, 0, 0)}, max_merge_dist=.2)
    assert merged[11] == (0, 0, 0)
    assert mode == 'BOTH'


def test_nan_not_propagated():
    merged, _ = fuse({11: (math.nan, 0, 0)}, {11: (1, 2, 3)})
    assert merged[11] == (1, 2, 3)


def test_stale_and_skew():
    assert inputs_usable(10.0, 10.05, 10.1, .35, .1) == (True, True, True)
    assert inputs_usable(10.0, 10.3, 10.31, .35, .1) == (True, True, False)
    assert inputs_usable(0.0, None, 11.0, .35, .1) == (False, False, True)
    assert inputs_usable(11.2, 11.2, 11.0, .35, .1) == (False, False, True)
