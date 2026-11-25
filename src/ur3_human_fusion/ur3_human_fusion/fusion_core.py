"""ROS-free fusion core, adapted from upstream SkeletonFuser.

Upstream: datn_ur3_robot_and_human-main/scripts/dual_kinect_fusion_controller.py
No TF estimation, skeleton identification, or safety-certified functionality.
"""
import math
from typing import Dict, Iterable, Optional, Tuple

Point = Tuple[float, float, float]
Skeleton = Dict[int, Point]
BODY_IDS = (0, 11, 12, 13, 14, 15, 16, 23, 24)
HAND_IDS = tuple(range(100, 121)) + tuple(range(200, 221))
FULL_IDS = BODY_IDS + HAND_IDS


def valid_point(p) -> bool:
    return len(p) == 3 and all(math.isfinite(float(v)) for v in p)


def fuse(front: Skeleton, back: Skeleton, tracked: Iterable[int] = BODY_IDS,
         max_merge_dist: float = 0.20):
    """Maintain upstream semantics: mean if close, prefer front if divergent.

    This is a baseline, not confidence-weighted fusion.
    """
    output = {}
    used_back_fill = False
    max_merge_dist = max(0.0, float(max_merge_dist))
    for jid in tracked:
        a, b = front.get(jid), back.get(jid)
        a = a if a is not None and valid_point(a) else None
        b = b if b is not None and valid_point(b) else None
        if a is not None and b is not None:
            d = math.dist(a, b)
            output[jid] = a if d > max_merge_dist else tuple((x + y) / 2 for x, y in zip(a, b))
        elif a is not None:
            output[jid] = a
        elif b is not None:
            output[jid] = b
            used_back_fill = True
    if front and back:
        mode = 'FILL_FROM_BACK' if used_back_fill else 'BOTH'
    elif front:
        mode = 'FRONT_ONLY'
    elif back:
        mode = 'BACK_ONLY'
    else:
        mode = 'NO_INPUT'
    return output, mode


def inputs_usable(front_stamp: Optional[float], back_stamp: Optional[float],
                  now: float, max_age: float, max_skew: float):
    """Return whether each input is fresh and whether both are synchronized.

    Reject zero/invalid/future timestamps; no fallback to receipt time.
    """
    def fresh(s):
        return s is not None and math.isfinite(s) and s > 0 and -0.05 <= now - s <= max_age
    f = fresh(front_stamp)
    b = fresh(back_stamp)
    skew_ok = not (f and b) or abs(front_stamp - back_stamp) <= max_skew
    return f, b, skew_ok
