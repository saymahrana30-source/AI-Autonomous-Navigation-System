"""Central configuration. Change values here to tune the whole system."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Config:
    # ---- simulation ----
    dt: float = 0.1              # seconds per simulation step
    max_steps: int = 3000        # safety limit (300 s of simulated time)
    seed: int = 7

    # ---- robot (unicycle / differential-drive model) ----
    robot_radius: float = 0.35   # cells (1 cell = 1 metre in the story)
    v_max: float = 1.6           # max linear speed  (cells/s)
    w_max: float = 2.8           # max angular speed (rad/s)

    # ---- simulated range sensor (2D LiDAR; ultrasonic = 1 ray) ----
    n_rays: int = 90
    max_range: float = 8.0
    ray_step: float = 0.1
    range_noise: float = 0.02    # gaussian sigma, clipped to +-0.05

    # ---- occupancy-grid mapping (log-odds) ----
    l_hit: float = 1.5
    l_free: float = -1.0
    l_min: float = -1.5
    l_max: float = 2.5
    l_occ: float = 1.5           # cell is "occupied" when log-odds >= this

    # ---- planning ----
    inflation_cells: int = 1     # safety margin around obstacles
    algorithm: str = "astar"     # "astar" or "dijkstra"
    replan_interval: int = 25    # steps between "is there a shorter path?" checks
    replan_gain: float = 1.5     # adopt a new path only if it is this much shorter

    # ---- control + safety ----
    lookahead: float = 2.0
    k_heading: float = 3.0
    stop_dist: float = 0.9       # emergency stop distance (front cone)
    slow_dist: float = 2.2       # start slowing down
    goal_tolerance: float = 0.6
    wait_timeout: int = 200      # steps to wait when no path exists before failing

    # ---- perception ----
    max_cluster_area: int = 50   # bigger blobs are treated as structure, not obstacles
