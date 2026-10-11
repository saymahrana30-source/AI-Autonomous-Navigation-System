"""Virtual environment: static map, hidden obstacles and moving obstacles.

Coordinates: world (x, y) with x = column, y = row, 1 cell = 1 unit.
The robot is given only the *prior map* (static layout). Hidden and moving
obstacles are discovered at run time through the sensor - like the real world.
"""
from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np

from .utils import inflate
from .planner import plan


@dataclass
class DynamicObstacle:
    """A 1-cell obstacle (pedestrian / forklift) patrolling between waypoints."""
    waypoints: list
    speed: float = 1.0
    label: str = "mover"
    pos: np.ndarray = field(init=False)
    _target: int = field(default=1, init=False)

    def __post_init__(self):
        self.pos = np.array(self.waypoints[0], dtype=float)

    def update(self, dt: float) -> None:
        remaining = self.speed * dt
        while remaining > 1e-9:
            tgt = np.array(self.waypoints[self._target], dtype=float)
            delta = tgt - self.pos
            dist = float(np.linalg.norm(delta))
            if dist <= remaining:
                self.pos = tgt
                remaining -= dist
                self._target = (self._target + 1) % len(self.waypoints)
            else:
                self.pos = self.pos + delta / dist * remaining
                remaining = 0.0

    def cell(self) -> tuple:
        return int(round(self.pos[1] - 0.5)), int(round(self.pos[0] - 0.5))


class World:
    def __init__(self, name, static, hidden, dynamic, start, goal):
        self.name = name
        self.static = static            # known layout (walls, shelves, buildings)
        self.hidden = hidden            # unknown static obstacles (boxes, parked cars)
        self.dynamic = dynamic          # unknown moving obstacles
        self.start = start              # (row, col)
        self.goal = goal                # (row, col)
        self.height, self.width = static.shape
        self.refresh()

    # ---- ground truth (only the simulator and the sensor may look at this) ----
    def dynamic_mask(self) -> np.ndarray:
        m = np.zeros_like(self.static)
        for d in self.dynamic:
            r, c = d.cell()
            if 0 <= r < self.height and 0 <= c < self.width:
                m[r, c] = True
        return m

    def refresh(self) -> None:
        self.occ = self.static | self.hidden | self.dynamic_mask()

    def step(self, dt: float) -> None:
        for d in self.dynamic:
            d.update(dt)
        self.refresh()

    def prior_map(self) -> np.ndarray:
        """What the robot knows before it starts moving."""
        return self.static.copy()

    def collides(self, x: float, y: float, radius: float) -> bool:
        pts = [(x, y)] + [(x + radius * np.cos(a), y + radius * np.sin(a))
                          for a in np.linspace(0, 2 * np.pi, 8, endpoint=False)]
        for px, py in pts:
            r, c = int(np.floor(py)), int(np.floor(px))
            if not (0 <= r < self.height and 0 <= c < self.width) or self.occ[r, c]:
                return True
        return False


# --------------------------------------------------------------------------
# Scenario builders
# --------------------------------------------------------------------------
def _rect(g, r0, r1, c0, c1):
    g[r0:r1 + 1, c0:c1 + 1] = True


def _border(g):
    g[0, :] = g[-1, :] = True
    g[:, 0] = g[:, -1] = True


def build_warehouse() -> World:
    """50x30 warehouse: shelf rows, cross aisle, boxes left in aisles, a forklift."""
    static = np.zeros((30, 50), dtype=bool)
    _border(static)
    for c in (8, 16, 24, 32, 40):
        _rect(static, 5, 12, c, c + 1)
        _rect(static, 17, 24, c, c + 1)
    hidden = np.zeros_like(static)
    _rect(hidden, 14, 15, 12, 13)    # box in the cross aisle
    _rect(hidden, 8, 9, 28, 29)      # box in a vertical aisle
    _rect(hidden, 20, 21, 20, 21)    # box in a vertical aisle
    dynamic = [
        DynamicObstacle([(36.5, 6.5), (36.5, 26.5)], speed=1.0, label="forklift"),
        DynamicObstacle([(14.5, 15.5), (30.5, 15.5)], speed=0.8, label="worker"),
    ]
    return World("warehouse", static, hidden, dynamic, start=(26, 3), goal=(3, 46))


def build_city() -> World:
    """42x60 city blocks: buildings, a parked car, a road closure and moving traffic."""
    static = np.zeros((42, 60), dtype=bool)
    _border(static)
    for r0, r1 in ((5, 12), (17, 24), (29, 36)):
        for c0, c1 in ((3, 12), (17, 26), (31, 40), (45, 54)):
            _rect(static, r0, r1, c0, c1)
    hidden = np.zeros_like(static)
    _rect(hidden, 14, 15, 20, 21)    # parked car
    _rect(hidden, 25, 28, 36, 36)    # road closure
    dynamic = [
        DynamicObstacle([(42.5, 5.5), (42.5, 38.5)], speed=1.0, label="vehicle"),
        DynamicObstacle([(14.5, 38.5), (40.5, 38.5)], speed=0.9, label="pedestrian"),
        DynamicObstacle([(22.5, 1.5), (22.5, 4.5)], speed=2.0, label="cyclist"),   # darts across the road
    ]
    return World("city", static, hidden, dynamic, start=(2, 2), goal=(39, 57))


def build_random(seed: int = 0, rows: int = 40, cols: int = 60, density: float = 0.14,
                 hidden_fraction: float = 0.4) -> World:
    """Random clutter; resampled until start->goal is reachable (used by tests/benchmark)."""
    for attempt in range(200):
        rng = np.random.default_rng(seed * 1000 + attempt)
        static = np.zeros((rows, cols), dtype=bool)
        _border(static)
        hidden = np.zeros_like(static)
        target = int(rows * cols * density)
        placed = 0
        while placed < target:
            h, w = rng.integers(1, 5, size=2)
            r, c = rng.integers(2, rows - 6), rng.integers(2, cols - 6)
            layer = hidden if rng.random() < hidden_fraction else static
            layer[r:r + h, c:c + w] = True
            placed += int(h * w)
        start, goal = (rows - 3, 2), (2, cols - 3)
        for rr, cc in (start, goal):
            static[rr - 2:rr + 3, cc - 2:cc + 3] = False
            hidden[rr - 2:rr + 3, cc - 2:cc + 3] = False
        _border(static)
        if plan(inflate(static | hidden, 1), start, goal).success:
            return World(f"random-{seed}", static, hidden, [], start, goal)
    raise RuntimeError("could not build a solvable random map")


SCENARIOS = {"warehouse": build_warehouse, "city": build_city}


def build_scenario(name: str, seed: int = 0) -> World:
    if name == "random":
        return build_random(seed)
    if name not in SCENARIOS:
        raise ValueError(f"Unknown scenario '{name}'. Choose: {list(SCENARIOS) + ['random']}")
    return SCENARIOS[name]()
