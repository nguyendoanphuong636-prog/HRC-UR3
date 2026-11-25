"""Pure functions for depth-camera optical-frame pose reconstruction (research only)."""
from __future__ import annotations

import math
import numpy as np

JOINT_IDS = (0, 11, 12, 13, 14, 15, 16, 23, 24)


def median_depth_at(depth_m: np.ndarray, u: int, v: int, radius: int = 2,
                    min_m: float = 0.4, max_m: float = 4.5,
                    max_spread_m: float = 0.15) -> float | None:
    """Return robust depth or None for missing, discontinuous, or implausible depth.

    Discontinuity checks suppress mixing foreground/background at limb borders.
    This cannot guarantee a correct human surface: validate against ground truth.
    """
    if depth_m.ndim != 2:
        raise ValueError("depth_m must be a HxW array")
    h, w = depth_m.shape
    if not (0 <= u < w and 0 <= v < h):
        return None
    x0, x1 = max(0, u - radius), min(w, u + radius + 1)
    y0, y1 = max(0, v - radius), min(h, v + radius + 1)
    vals = depth_m[y0:y1, x0:x1].astype(np.float64, copy=False).ravel()
    vals = vals[np.isfinite(vals) & (vals >= min_m) & (vals <= max_m)]
    if vals.size < max(3, ((y1-y0)*(x1-x0)) // 3):
        return None
    med = float(np.median(vals))
    nearby = vals[np.abs(vals - med) <= 0.05]
    if nearby.size < max(3, ((y1-y0)*(x1-x0)) // 3):
        return None
    if float(np.quantile(vals, 0.85) - np.quantile(vals, 0.15)) > max_spread_m:
        return None
    return med


def unproject(u: float, v: float, depth_m: float,
              fx: float, fy: float, cx: float, cy: float) -> tuple[float, float, float]:
    if not all(map(math.isfinite, (u, v, depth_m, fx, fy, cx, cy))):
        raise ValueError("Non-finite unprojection input")
    if fx <= 0 or fy <= 0 or depth_m <= 0:
        raise ValueError("Invalid focal length or depth")
    return ((u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m)


def reconstruct(landmarks: list[dict], depth_m: np.ndarray, K,
                min_visibility: float = 0.55,
                min_presence: float = 0.5) -> dict[int, tuple[float, float, float]]:
    fx, fy, cx, cy = float(K[0]), float(K[4]), float(K[2]), float(K[5])
    h, w = depth_m.shape
    result = {}
    for joint in landmarks:
        jid = int(joint.get('id', -1))
        if jid not in JOINT_IDS:
            continue
        if float(joint.get('visibility', 0)) < min_visibility or float(joint.get('presence', 0)) < min_presence:
            continue
        ux, vy = float(joint.get('x_norm', float('nan'))), float(joint.get('y_norm', float('nan')))
        if not (math.isfinite(ux) and math.isfinite(vy) and 0 <= ux <= 1 and 0 <= vy <= 1):
            continue
        u, v = int(round(ux * (w-1))), int(round(vy * (h-1)))
        z = median_depth_at(depth_m, u, v)
        if z is None:
            continue
        result[jid] = unproject(u, v, z, fx, fy, cx, cy)
    return result
