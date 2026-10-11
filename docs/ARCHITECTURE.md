# Architecture

```
                 ┌──────────────────────── SIMULATED WORLD (world.py) ───────────────────────┐
                 │  static map (known)  +  hidden boxes (unknown)  +  moving obstacles (unknown) │
                 └───────────────┬───────────────────────────────────────────▲─────────────────┘
                                 │ ground truth (sensor only)                │ robot pose update
                                 ▼                                           │
 ┌───────────────┐   ranges   ┌──────────────────┐   belief map   ┌──────────┴─────────┐
 │ SENSOR        │ ─────────► │ PERCEPTION       │ ─────────────► │ DECISION / PLANNING │
 │ sensors.py    │            │ perception.py    │   detections   │ agent.py + planner.py│
 │ 90-ray LiDAR  │            │ occupancy grid + │                │ state machine,       │
 │ (+noise)      │            │ OpenCV blob      │                │ A*, smoothing,       │
 └───────────────┘            │ detection+tracking│               │ re-plan triggers     │
        ▲                     └──────────────────┘                └──────────┬──────────┘
        │                                                                      │ waypoints
        │                         ┌──────────────────┐   (v, w)    ┌──────────▼──────────┐
        └──────── pose ────────── │ ROBOT DYNAMICS   │ ◄────────── │ CONTROL + SAFETY    │
                                  │ simulation.py    │             │ controller.py       │
                                  │ unicycle model   │             │ pure pursuit +      │
                                  └──────────────────┘             │ emergency stop      │
                                                                   └─────────────────────┘
        Visualization (visualizer.py) reads everything and renders the dashboard / GIF / plots.
```

## Data flow per time step (dt = 0.1 s)
1. **Sense** - the LiDAR casts 90 rays through the *ground-truth* grid and returns noisy ranges. The robot never sees the ground truth directly.
2. **Perceive** - ranges update a log-odds occupancy grid (obstacle seen → up, free space seen → down). Cells that were occupied and then cleared are marked "recently changed" (= something moved).
3. **Detect** - OpenCV `connectedComponentsWithStats` finds blobs that are *not* in the prior map. Blobs are tracked across frames and labelled STATIC / MOVING.
4. **Decide** - state machine: `PLANNING → NAVIGATING ⇄ AVOIDING / WAITING → GOAL_REACHED | FAILED`.
   Re-plan when the remaining path hits an occupied cell or the safety margin; every 25 steps check for a shorter path.
5. **Plan** - A* (8-connected, octile heuristic, no corner cutting) on the *inflated* belief map, then line-of-sight smoothing.
6. **Control** - pure pursuit turns waypoints into `(v, w)`; speed drops near obstacles.
7. **Safety layer** - if anything is closer than 0.9 m in the ±35° front cone: stop and turn toward the freer side, request a re-plan. This overrides the planner.
8. **Act** - unicycle kinematics move the robot; collisions are counted by the simulator.

## Module map
| File | Responsibility |
|---|---|
| `src/config.py` | every tunable number in one place |
| `src/world.py` | scenarios (warehouse, city, random), moving obstacles, collision check |
| `src/sensors.py` | simulated LiDAR / ultrasonic ranging |
| `src/perception.py` | occupancy-grid mapping, OpenCV obstacle detection + tracking |
| `src/planner.py` | A*, Dijkstra, path smoothing |
| `src/controller.py` | pure-pursuit path follower |
| `src/agent.py` | decision-making state machine, re-planning, safety layer |
| `src/simulation.py` | main loop, robot dynamics, metrics |
| `src/visualizer.py` | dashboard, GIF, trajectory + telemetry plots |
| `main.py` / `benchmark.py` | CLI entry points |
| `tests/` | 14 automated tests |

## Honest limitations
- 2D grid world; no camera images (so no YOLO / lane detection yet - see roadmap in README).
- A moving obstacle that leaves sensor range can leave a stale "ghost" in the map until the robot sees that cell again.
- Obstacles are labelled moving/static from map changes, not classified by type.
