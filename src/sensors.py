"""Simulated range sensor. A 360-degree LiDAR is many ultrasonic sensors in one."""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np


@dataclass
class Scan:
    ranges: np.ndarray       # measured distance per ray (cells)
    hit: np.ndarray          # True where the ray hit something before max_range
    rel_angles: np.ndarray   # ray angles relative to robot heading


class Lidar:
    def __init__(self, cfg):
        self.cfg = cfg
        self.rel_angles = np.linspace(-np.pi, np.pi, cfg.n_rays, endpoint=False)
        self.t = np.arange(cfg.ray_step, cfg.max_range + 1e-9, cfg.ray_step)

    def scan(self, pose, occ: np.ndarray, rng: np.random.Generator) -> Scan:
        """Cast rays through the ground-truth grid `occ` and return noisy ranges."""
        x, y, th = pose
        ang = th + self.rel_angles
        xs = x + np.outer(np.cos(ang), self.t)
        ys = y + np.outer(np.sin(ang), self.t)
        cols, rows = np.floor(xs).astype(int), np.floor(ys).astype(int)
        H, W = occ.shape
        inside = (rows >= 0) & (rows < H) & (cols >= 0) & (cols < W)
        hit_grid = np.ones(inside.shape, dtype=bool)          # outside map counts as a wall
        hit_grid[inside] = occ[rows[inside], cols[inside]]
        hit = hit_grid.any(axis=1)
        first = hit_grid.argmax(axis=1)
        ranges = np.where(hit, self.t[first], self.cfg.max_range)
        noise = np.clip(rng.normal(0, self.cfg.range_noise, len(ranges)), -0.05, 0.05)
        ranges = np.where(hit, np.clip(ranges + noise, 0.05, self.cfg.max_range), ranges)
        return Scan(ranges, hit, self.rel_angles)

    @staticmethod
    def sector_min(scan: Scan, center: float, half_width: float) -> float:
        """Smallest range inside a sector (robot frame) - used for the safety layer."""
        d = np.abs((scan.rel_angles - center + np.pi) % (2 * np.pi) - np.pi)
        return float(scan.ranges[d <= half_width].min())

    @staticmethod
    def sector_mean(scan: Scan, center: float, half_width: float) -> float:
        d = np.abs((scan.rel_angles - center + np.pi) % (2 * np.pi) - np.pi)
        return float(scan.ranges[d <= half_width].mean())
