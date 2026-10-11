"""The 'brain': perception -> decision making -> planning -> control, as a state machine."""
from __future__ import annotations
import math
from enum import Enum
import numpy as np

from .controller import PurePursuit, densify
from .perception import ObstacleDetector, OccupancyMap
from .planner import plan, smooth_path
from .sensors import Lidar
from .utils import cell_center, to_cell


class State(str, Enum):
    PLANNING = "PLANNING"
    NAVIGATING = "NAVIGATING"
    AVOIDING = "AVOIDING"
    WAITING = "WAITING"
    GOAL_REACHED = "GOAL_REACHED"
    FAILED = "FAILED"


class NavigationAgent:
    def __init__(self, start, goal, prior, cfg, use_prior=True):
        self.cfg = cfg
        self.goal = goal
        gx, gy = cell_center(goal)
        sx, sy = cell_center(start)
        self.goal_xy = np.array([gx, gy])
        self.pose = np.array([sx, sy, math.atan2(gy - sy, gx - sx)])   # x, y, heading

        self.lidar = Lidar(cfg)
        self.map = OccupancyMap(prior if use_prior else np.zeros_like(prior), cfg)
        if not use_prior:                      # unknown world: add the outer wall as common sense
            self.map.L[[0, -1], :] = cfg.l_max
            self.map.L[:, [0, -1]] = cfg.l_max
            self.map.prior = self.map.L >= cfg.l_occ
        self.detector = ObstacleDetector(cfg)
        self.controller = PurePursuit(cfg)

        self.state = State.PLANNING
        self.waypoints = np.empty((0, 2))
        self.dense = np.empty((0, 2))
        self.idx = 0
        self.detections = []
        self.scan = None
        self.front_min = cfg.max_range
        self.replans = 0
        self.plan_calls = 0
        self.plan_ms = 0.0
        self.events = []                       # (name, message) produced this step
        self._wait_steps = 0
        self._force_replan = False
        self._seen_detection = self._seen_replan = self._seen_avoid = False

    # ------------------------------------------------------------------ sense
    def perceive(self, scan) -> None:
        self.events = []
        self.scan = scan
        self.map.update(self.pose, scan)
        self.detections = self.detector.detect(self.map, self.pose)
        self.front_min = self.lidar.sector_min(scan, 0.0, math.radians(35))
        if self.detections and not self._seen_detection:
            self._seen_detection = True
            d = min(self.detections, key=lambda d: d.distance)
            self.events.append(("first_detection", f"unexpected obstacle detected {d.distance:.1f} m away"))

    # ------------------------------------------------------------------- plan
    def _plan(self, reason: str) -> bool:
        cfg = self.cfg
        start = to_cell(self.pose[0], self.pose[1])
        for margin in (cfg.inflation_cells, 0):          # fall back to no margin to escape tight spots
            blocked = self.map.inflated(margin)
            res = plan(blocked, start, self.goal, cfg.algorithm)
            self.plan_calls += 1
            self.plan_ms += res.runtime_ms
            if res.success:
                cells = smooth_path(blocked, res.path)
                pts = np.array([cell_center(c) for c in cells])
                pts[0] = self.pose[:2]
                self.waypoints, self.dense, self.idx = pts, densify(pts), 0
                self.last_cost = res.cost
                return True
        return False

    def _remaining_length(self) -> float:
        seg = np.diff(self.dense[self.idx:], axis=0)
        return float(np.linalg.norm(seg, axis=1).sum()) if len(seg) else 0.0

    def _path_blocked(self) -> bool:
        if len(self.dense) == 0:
            return True
        pts = self.dense[self.idx:]
        rows, cols = np.floor(pts[:, 1]).astype(int), np.floor(pts[:, 0]).astype(int)
        raw = self.map.occupied()[rows, cols]
        inf = self.map.inflated(self.cfg.inflation_cells)[rows, cols]
        far = np.linalg.norm(pts - self.pose[:2], axis=1) > 1.5   # ignore the zone we're already in
        return bool(raw.any() or (inf & far).any())

    # --------------------------------------------------------------------- act
    def act(self, step: int) -> tuple:
        """Decide (v, w) for this time step."""
        cfg = self.cfg
        goal_dist = float(np.linalg.norm(self.goal_xy - self.pose[:2]))
        if goal_dist < cfg.goal_tolerance:
            if self.state != State.GOAL_REACHED:
                self.events.append(("goal", "goal reached"))
            self.state = State.GOAL_REACHED
            return 0.0, 0.0

        # ---- decision making ----
        if self.state in (State.PLANNING, State.WAITING):
            reason_initial = self.state == State.PLANNING
            if self._plan("initial" if self.state == State.PLANNING else "retry"):
                if reason_initial:
                    j = min(6, len(self.dense) - 1)
                    self.pose[2] = math.atan2(self.dense[j][1] - self.pose[1], self.dense[j][0] - self.pose[0])
                self.state, self._wait_steps = State.NAVIGATING, 0
            else:
                self._wait_steps += 1
                self.state = State.WAITING
                if self._wait_steps > cfg.wait_timeout:
                    self.state = State.FAILED
                    self.events.append(("failed", "no path to goal"))
                return 0.0, 0.0
        elif self.state == State.FAILED:
            return 0.0, 0.0
        else:
            if self._force_replan or self._path_blocked():
                self._force_replan = False
                self.replans += 1
                if not self._seen_replan:
                    self._seen_replan = True
                    self.events.append(("first_replan", "path blocked -> re-planning"))
                if not self._plan("blocked"):
                    self.state = State.WAITING
                    return 0.0, 0.0
            elif step % cfg.replan_interval == 0:         # opportunistic shortcut check
                old = self._remaining_length()
                keep = (self.waypoints, self.dense, self.idx)
                if self._plan("refresh") and self.last_cost > old - cfg.replan_gain:
                    self.waypoints, self.dense, self.idx = keep

        # ---- control ----
        self.idx = self.controller.closest_index(self.dense, self.pose[:2], self.idx)
        v, w = self.controller.command(self.pose, self.dense, self.idx, goal_dist)

        # ---- reactive collision avoidance (safety layer, overrides the planner) ----
        if self.front_min < cfg.stop_dist:
            left = self.lidar.sector_mean(self.scan, math.radians(60), math.radians(30))
            right = self.lidar.sector_mean(self.scan, -math.radians(60), math.radians(30))
            self.state = State.AVOIDING
            self._force_replan = True
            if not self._seen_avoid:
                self._seen_avoid = True
                self.events.append(("first_avoid", f"emergency stop - obstacle {self.front_min:.2f} m ahead"))
            return 0.0, (0.8 if left >= right else -0.8) * cfg.w_max
        if self.state == State.AVOIDING:
            self.state = State.NAVIGATING
        if self.front_min < cfg.slow_dist:
            v *= float(np.clip((self.front_min - cfg.stop_dist) / (cfg.slow_dist - cfg.stop_dist), 0.25, 1.0))
        return v, w
