"""Global path planning on an occupancy grid: A*, Dijkstra, and path smoothing."""
from __future__ import annotations
import heapq
import math
import time
from dataclasses import dataclass
import numpy as np

SQRT2 = math.sqrt(2.0)
MOVES = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
         (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]


@dataclass
class PlanResult:
    success: bool
    path: list            # list of (row, col)
    cost: float
    expanded: int         # nodes expanded - lower is "more efficient"
    runtime_ms: float
    algorithm: str


def octile(a, b) -> float:
    """Admissible heuristic for 8-connected grids."""
    dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dr + dc) + (SQRT2 - 2.0) * min(dr, dc)


def plan(blocked: np.ndarray, start: tuple, goal: tuple, algorithm: str = "astar") -> PlanResult:
    """Shortest path on an 8-connected grid. blocked[r, c] == True means obstacle.

    The start cell is always allowed (the robot may sit inside an inflated zone).
    Diagonal moves are not allowed to cut corners.
    """
    t0 = time.perf_counter()
    H, W = blocked.shape
    blk = blocked.tolist()                       # python lists are faster to index
    use_h = algorithm == "astar"

    def fail(expanded=0):
        return PlanResult(False, [], math.inf, expanded, (time.perf_counter() - t0) * 1e3, algorithm)

    if not (0 <= goal[0] < H and 0 <= goal[1] < W) or blk[goal[0]][goal[1]]:
        return fail()

    g = {start: 0.0}
    parent = {}
    heap = [(octile(start, goal) if use_h else 0.0, 0, start)]
    closed = set()
    tie = 0
    expanded = 0
    while heap:
        _, _, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        closed.add(cur)
        expanded += 1
        if cur == goal:
            path = [cur]
            while cur in parent:
                cur = parent[cur]
                path.append(cur)
            path.reverse()
            return PlanResult(True, path, g[goal], expanded,
                              (time.perf_counter() - t0) * 1e3, algorithm)
        r, c = cur
        for dr, dc, step in MOVES:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < H and 0 <= nc < W) or blk[nr][nc]:
                continue
            if dr != 0 and dc != 0 and (blk[r + dr][c] or blk[r][c + dc]):
                continue                         # no corner cutting
            nb = (nr, nc)
            ng = g[cur] + step
            if ng < g.get(nb, math.inf):
                g[nb] = ng
                parent[nb] = cur
                tie += 1
                heapq.heappush(heap, (ng + (octile(nb, goal) if use_h else 0.0), tie, nb))
    return fail(expanded)


def line_of_sight(blocked: np.ndarray, a: tuple, b: tuple) -> bool:
    """True if the straight segment between cell centres a and b is obstacle free."""
    ax, ay, bx, by = a[1] + 0.5, a[0] + 0.5, b[1] + 0.5, b[0] + 0.5
    n = max(2, int(math.hypot(bx - ax, by - ay) / 0.1))
    t = np.linspace(0.0, 1.0, n + 1)
    cols = np.floor(ax + (bx - ax) * t).astype(int)
    rows = np.floor(ay + (by - ay) * t).astype(int)
    keep = ~((rows == a[0]) & (cols == a[1]))   # ignore the cell we start in
    H, W = blocked.shape
    rows, cols = rows[keep], cols[keep]
    if ((rows < 0) | (rows >= H) | (cols < 0) | (cols >= W)).any():
        return False
    return not blocked[rows, cols].any()


def smooth_path(blocked: np.ndarray, path: list, max_skip: int = 40) -> list:
    """Remove zig-zags: keep only waypoints needed to stay collision-free."""
    if len(path) <= 2:
        return list(path)
    out, i, n = [path[0]], 0, len(path)
    while i < n - 1:
        j = min(n - 1, i + max_skip)
        while j > i + 1 and not line_of_sight(blocked, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out
