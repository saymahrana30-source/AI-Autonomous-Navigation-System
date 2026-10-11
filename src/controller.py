"""Low-level controller: pure-pursuit path following for a unicycle robot."""
from __future__ import annotations
import math
import numpy as np

from .utils import wrap_angle


def densify(waypoints: np.ndarray, spacing: float = 0.25) -> np.ndarray:
    """Insert points along each segment so the follower has a smooth reference."""
    pts = [waypoints[0]]
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        n = max(1, int(np.linalg.norm(b - a) / spacing))
        for k in range(1, n + 1):
            pts.append(a + (b - a) * k / n)
    return np.array(pts)


class PurePursuit:
    def __init__(self, cfg):
        self.cfg = cfg

    def closest_index(self, dense, pos, start_idx, window=60) -> int:
        hi = min(len(dense), start_idx + window)
        d = np.linalg.norm(dense[start_idx:hi] - pos, axis=1)
        return start_idx + int(d.argmin())

    def command(self, pose, dense, idx, goal_dist) -> tuple:
        """Return (v, w) steering towards a look-ahead point on the path."""
        cfg = self.cfg
        x, y, th = pose
        j = idx
        while j < len(dense) - 1 and math.hypot(dense[j][0] - x, dense[j][1] - y) < cfg.lookahead:
            j += 1
        err = wrap_angle(math.atan2(dense[j][1] - y, dense[j][0] - x) - th)
        w = float(np.clip(cfg.k_heading * err, -cfg.w_max, cfg.w_max))
        v = cfg.v_max * max(0.0, math.cos(err)) ** 2          # turn in place if badly misaligned
        v *= min(1.0, goal_dist / 2.0)                          # ease into the goal
        return v, w
