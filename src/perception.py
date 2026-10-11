"""Perception: occupancy-grid mapping + OpenCV obstacle detection and tracking."""
from __future__ import annotations
from dataclasses import dataclass
import cv2
import numpy as np

from .utils import inflate


class OccupancyMap:
    """The robot's belief about the world, built from range scans (log-odds grid).

    Cells start from the prior map (known layout). Every scan then:
      * raises the log-odds of the cell where a ray ended (obstacle seen)
      * lowers the log-odds of cells the ray passed through (free space seen)
    so hidden boxes get *added* and moved obstacles get *removed*.
    """

    def __init__(self, prior: np.ndarray, cfg):
        self.cfg = cfg
        self.prior = prior.copy()
        self.L = np.where(prior, cfg.l_max, 0.0).astype(float)
        self.age = np.full(prior.shape, 999, dtype=np.int16)   # steps since a cell was "cleared"
        self._occ = self.L >= cfg.l_occ

    def update(self, pose, scan) -> None:
        cfg, (H, W) = self.cfg, self.L.shape
        x, y, th = pose
        ang = th + scan.rel_angles
        t = np.arange(cfg.ray_step, cfg.max_range + 1e-9, cfg.ray_step)

        # free cells: samples well before the measured range
        xs = x + np.outer(np.cos(ang), t)
        ys = y + np.outer(np.sin(ang), t)
        cols, rows = np.floor(xs).astype(int), np.floor(ys).astype(int)
        ok = (t[None, :] < scan.ranges[:, None] - 0.25) & (rows >= 0) & (rows < H) & (cols >= 0) & (cols < W)
        free_idx = np.unique(rows[ok] * W + cols[ok])

        # hit cells: just behind the measured range
        h = scan.hit
        hx = x + (scan.ranges[h] + 0.05) * np.cos(ang[h])
        hy = y + (scan.ranges[h] + 0.05) * np.sin(ang[h])
        hc, hr = np.floor(hx).astype(int), np.floor(hy).astype(int)
        v = (hr >= 0) & (hr < H) & (hc >= 0) & (hc < W)
        hit_idx = np.unique(hr[v] * W + hc[v])
        free_idx = np.setdiff1d(free_idx, hit_idx)

        was_occ = self.L >= cfg.l_occ
        flat = self.L.reshape(-1)
        flat[free_idx] += cfg.l_free
        flat[hit_idx] += cfg.l_hit
        np.clip(flat, cfg.l_min, cfg.l_max, out=flat)
        now_occ = self.L >= cfg.l_occ

        self.age = np.minimum(self.age + 1, 999)
        self.age[was_occ & ~now_occ] = 0         # something that was there has gone -> it moves
        self._occ = now_occ

    def occupied(self) -> np.ndarray:
        return self._occ

    def inflated(self, cells: int) -> np.ndarray:
        return inflate(self._occ, cells)


@dataclass
class Detection:
    id: int
    bbox: tuple        # (x, y, w, h) in world units
    centroid: tuple    # (x, y)
    area: int
    moving: bool
    distance: float


class ObstacleDetector:
    """Finds *unexpected* obstacles (not in the prior map) with OpenCV blob analysis,
    tracks them frame to frame, and labels each one static or moving."""

    def __init__(self, cfg):
        self.cfg = cfg
        self._tracks = []          # (id, (x, y))
        self._next_id = 0

    def detect(self, occ_map: OccupancyMap, pose) -> list:
        new_obs = (occ_map.occupied() & ~occ_map.prior).astype(np.uint8)
        n, labels, stats, cents = cv2.connectedComponentsWithStats(new_obs, connectivity=8)
        recent = (occ_map.age < 40).astype(np.uint8)
        recent = cv2.dilate(recent, np.ones((5, 5), np.uint8)).astype(bool)

        dets, new_tracks = [], []
        for i in range(1, n):
            x, y, w, h, area = stats[i]
            if area > self.cfg.max_cluster_area:
                continue
            cx, cy = float(cents[i][0]) + 0.5, float(cents[i][1]) + 0.5
            tid = self._match(cx, cy, new_tracks)
            moving = bool(recent[labels == i].any())
            dist = float(np.hypot(cx - pose[0], cy - pose[1]))
            dets.append(Detection(tid, (float(x), float(y), float(w), float(h)), (cx, cy),
                                  int(area), moving, dist))
            new_tracks.append((tid, (cx, cy)))
        self._tracks = new_tracks
        return dets

    def _match(self, cx, cy, taken) -> int:
        used = {t for t, _ in taken}
        best, best_d = None, 3.0
        for tid, (px, py) in self._tracks:
            d = np.hypot(cx - px, cy - py)
            if tid not in used and d < best_d:
                best, best_d = tid, d
        if best is None:
            best = self._next_id
            self._next_id += 1
        return best

    @property
    def unique_obstacles(self) -> int:
        return self._next_id
