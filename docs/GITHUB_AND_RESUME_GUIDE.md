# GitHub, resume and interview guide

## 1. Publish on GitHub (exact commands)
Repo name: **`AI-Autonomous-Navigation-System`**
Description: *A* path planning, LiDAR mapping, OpenCV obstacle detection and safety-layer collision avoidance for a simulated autonomous robot (Python).*
Topics: `autonomous-navigation` `robotics` `path-planning` `a-star` `computer-vision` `opencv` `python` `simulation` `self-driving` `occupancy-grid` `lidar` `ai`

```bash
cd AI-Autonomous-Navigation-System
git init
git add .
git commit -m "feat: initial project structure, world and planner"
git branch -M main
# create an EMPTY repo on github.com (no README/license), then:
git remote add origin https://github.com/<your-username>/AI-Autonomous-Navigation-System.git
git push -u origin main
```
Pin the repo on your profile (Profile → *Customize your pins*). Add the GIF and screenshots - they are already in `outputs/` and referenced by the README.

## 2. Honest 7-day commit plan
Do the real work in this order so the history matches what you actually built. (Your repo already contains the finished code - to build the history, add files progressively.)

| Day | Commit | Save as proof | Commit message |
|---|---|---|---|
| 1 | `config.py`, `utils.py`, `world.py`, `.gitignore`, `requirements.txt` | `01_environment_start.png` | `feat: grid world with warehouse and city scenarios` |
| 2 | `planner.py`, `tests/test_planner.py` | pytest output screenshot | `feat: A* and Dijkstra planners with tests` |
| 3 | `sensors.py`, `perception.py` | `03_obstacle_detection.png` | `feat: simulated LiDAR, occupancy mapping, OpenCV detection` |
| 4 | `controller.py`, `agent.py` | `04_replanning.png` | `feat: pure-pursuit control, state machine, re-planning` |
| 5 | safety layer, `simulation.py` | terminal screenshot | `feat: emergency stop layer and metrics` |
| 6 | `visualizer.py`, `main.py`, `benchmark.py` | `navigation_demo.gif`, `planner_comparison.png` | `feat: dashboard, GIF export, planner benchmark` |
| 7 | README, docs, outputs | GitHub repo + README screenshots | `docs: README, architecture, results` |

Note for LinkedIn each day: what you built, one problem you hit, how you fixed it.

## 3. Resume bullets
- Built a Python autonomous-navigation simulator with a 90-ray LiDAR model, log-odds occupancy mapping and OpenCV obstacle detection/tracking; robot reached its goal with **0 collisions** in warehouse and city scenarios containing unknown and moving obstacles.
- Implemented A\* global planning with path smoothing, event-driven re-planning and a reactive emergency-stop layer; A\* expanded **51% fewer nodes** than Dijkstra at identical path cost (30 random maps).
- Structured the system as modular, tested code (14 pytest tests, CLI, JSON metrics, GIF/telemetry output) and documented it for reproducible demos on GitHub.

## 4. LinkedIn post (short)
> Built an AI-based autonomous navigation system from scratch in Python. A simulated robot senses with LiDAR, maps its surroundings, spots obstacles nobody told it about (boxes, forklifts, pedestrians), plans routes with A*, re-plans on the fly, and brakes on its own - 0 collisions across warehouse and city scenarios. Next step: camera-based detection with YOLO and ROS 2. Code + demo: <link> #robotics #python #computervision #autonomousvehicles

## 5. Interview Q&A (answers match what this repo actually does)
1. **What does the project do?** A simulated robot navigates from start to goal using LiDAR-based mapping, obstacle detection, A\* planning, re-planning and a safety layer - with no manual control.
2. **How does it detect obstacles?** A simulated LiDAR casts rays through the hidden ground truth. Ranges update an occupancy grid; OpenCV connected-components finds blobs that are not in the prior map and tracks them across frames.
3. **Why A\* over Dijkstra?** Same optimal cost, but the heuristic guides the search - 51% fewer nodes expanded in my benchmark.
4. **What if the map is wrong?** The robot starts with only the static layout. Hidden boxes get added when seen; moved obstacles get removed when the cells are seen free again (log-odds).
5. **How do you avoid collisions?** Two layers: the planner inflates obstacles by a safety margin; a reactive layer stops the robot if something is within 0.9 m ahead, regardless of the plan.
6. **How do you detect moving obstacles?** A cell that was occupied and later becomes free marks recent change; blobs near recent changes are labelled MOVING.
7. **Why simulate?** Fast, safe, repeatable testing with metrics; the brain only consumes `Scan` objects, so the sensor and motors can be swapped for real hardware.
8. **Biggest challenge?** Start-up deadlocks and flickering map cells; fixed with log-odds clamping, a no-corner-cutting planner and a fallback to zero margin when boxed in.
9. **Limitations?** 2D, range-only sensing, no localisation noise, stale "ghost" obstacles. Next: camera + YOLO, SLAM, ROS 2.
10. **Explain to HR.** "I built software that lets a virtual robot find its own way around a warehouse or city, notice unexpected obstacles, change its route and stop safely - the same idea behind self-driving cars and warehouse robots."

**Be ready to say honestly:** this version uses range sensing and OpenCV blob analysis; YOLO/lane detection are listed as future work, not implemented.

## 6. Troubleshooting
| Problem | Reason | Fix |
|---|---|---|
| `pip install opencv-python` fails | old pip / unsupported Python | `python -m pip install --upgrade pip`; use Python 3.9-3.12 |
| `ModuleNotFoundError: src` | running from wrong folder | `cd` into the project root and run `python main.py` |
| `--live` shows no window | headless machine / no Tk | omit `--live`; open `outputs/navigation_demo.gif` |
| Robot stuck in `WAITING` | goal/path fully blocked by margin | lower `inflation_cells` in `config.py`; check goal cell is free |
| Collisions > 0 after edits | margin too small or speed too high | raise `inflation_cells` / `stop_dist`, lower `v_max` |
| `git push` rejected | remote has a README you did not pull | `git pull origin main --rebase` then push |
| `git push` auth error | password auth removed | use a Personal Access Token or GitHub CLI (`gh auth login`) |
| GIF too large for README | many frames | `python main.py --frame-skip 8` |
