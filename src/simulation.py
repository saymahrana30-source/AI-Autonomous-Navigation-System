"""Simulation loop: world <-> sensor <-> agent <-> robot dynamics, plus metrics."""
from __future__ import annotations
import math
import numpy as np

from .agent import NavigationAgent, State
from .config import Config
from .world import World


class Simulation:
    def __init__(self, world: World, cfg: Config = Config(), use_prior: bool = True):
        self.world, self.cfg = world, cfg
        self.rng = np.random.default_rng(cfg.seed)
        self.agent = NavigationAgent(world.start, world.goal, world.prior_map(), cfg, use_prior)
        self.step_count = 0
        self.trail = [tuple(self.agent.pose[:2])]
        self.path_len = 0.0
        self.collisions = 0
        self._in_collision = False
        self.min_clearance = math.inf
        self.log = {"t": [], "v": [], "w": [], "front": [], "min_range": [], "state": []}
        self.events = {}                       # name -> step first seen
        self.new_events = []

    @property
    def done(self) -> bool:
        return self.agent.state in (State.GOAL_REACHED, State.FAILED) or self.step_count >= self.cfg.max_steps

    def step(self) -> None:
        cfg, world, agent = self.cfg, self.world, self.agent
        scan = agent.lidar.scan(agent.pose, world.occ, self.rng)
        agent.perceive(scan)
        v, w = agent.act(self.step_count)
        self.new_events = [(n, m) for n, m in agent.events]
        for n, _ in self.new_events:
            self.events.setdefault(n, self.step_count)

        # robot motion (unicycle model); a collision blocks translation
        x, y, th = agent.pose
        th_mid = th + 0.5 * w * cfg.dt
        nx, ny = x + v * math.cos(th_mid) * cfg.dt, y + v * math.sin(th_mid) * cfg.dt
        if world.collides(nx, ny, cfg.robot_radius):
            nx, ny = x, y
        else:
            self.path_len += math.hypot(nx - x, ny - y)
        agent.pose = np.array([nx, ny, (th + w * cfg.dt + math.pi) % (2 * math.pi) - math.pi])
        world.step(cfg.dt)

        hit_now = world.collides(agent.pose[0], agent.pose[1], cfg.robot_radius)
        if hit_now and not self._in_collision:
            self.collisions += 1
        self._in_collision = hit_now

        self.min_clearance = min(self.min_clearance, float(scan.ranges.min()) - cfg.robot_radius)
        self.trail.append((agent.pose[0], agent.pose[1]))
        for k, val in (("t", self.step_count * cfg.dt), ("v", v), ("w", w), ("front", agent.front_min),
                       ("min_range", float(scan.ranges.min())), ("state", agent.state.value)):
            self.log[k].append(val)
        self.step_count += 1

    def run(self, on_step=None) -> dict:
        while not self.done:
            self.step()
            if on_step:
                on_step(self)
        return self.metrics()

    def metrics(self) -> dict:
        a, w = self.agent, self.world
        straight = math.hypot(w.goal[0] - w.start[0], w.goal[1] - w.start[1])
        return {
            "scenario": w.name,
            "success": a.state == State.GOAL_REACHED,
            "final_state": a.state.value,
            "sim_time_s": round(self.step_count * self.cfg.dt, 1),
            "path_length": round(self.path_len, 1),
            "straight_line_distance": round(straight, 1),
            "path_efficiency": round(straight / self.path_len, 3) if self.path_len else 0.0,
            "replans": a.replans,
            "planner_calls": a.plan_calls,
            "avg_plan_time_ms": round(a.plan_ms / max(1, a.plan_calls), 2),
            "unexpected_obstacles_detected": a.detector.unique_obstacles,
            "collisions": self.collisions,
            "min_clearance": round(self.min_clearance, 2),
            "algorithm": self.cfg.algorithm,
        }
