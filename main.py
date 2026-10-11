"""AI-Based Autonomous Navigation System - run the simulation.

Examples
    python main.py                          # warehouse demo, saves GIF + screenshots to outputs/
    python main.py --scenario city
    python main.py --scenario random --seed 3
    python main.py --live                   # open a live window instead of headless
    python main.py --algorithm dijkstra     # compare planners
    python main.py --no-prior               # robot starts with NO map (pure exploration)
"""
import argparse
import dataclasses
import json
import os
import sys

import matplotlib


def parse_args():
    p = argparse.ArgumentParser(description="AI-Based Autonomous Navigation System (2D simulation)")
    p.add_argument("--scenario", default="warehouse", choices=["warehouse", "city", "random"])
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--algorithm", default="astar", choices=["astar", "dijkstra"])
    p.add_argument("--no-prior", action="store_true", help="do not give the robot the static map")
    p.add_argument("--live", action="store_true", help="show a live window (needs a display)")
    p.add_argument("--no-gif", action="store_true", help="skip GIF creation (faster)")
    p.add_argument("--frame-skip", type=int, default=5, help="render every N-th step into the GIF")
    p.add_argument("--out", default="outputs", help="output folder")
    return p.parse_args()


def main():
    args = parse_args()
    if not args.live:
        matplotlib.use("Agg")                     # headless: works on servers / WSL / Colab
    from src.config import Config
    from src.simulation import Simulation
    from src.visualizer import Dashboard, plot_telemetry, plot_trajectory, save_gif
    from src.world import build_scenario

    cfg = dataclasses.replace(Config(), seed=args.seed, algorithm=args.algorithm)
    world = build_scenario(args.scenario, args.seed)
    sim = Simulation(world, cfg, use_prior=not args.no_prior)
    shots = os.path.join(args.out, "screenshots")
    os.makedirs(shots, exist_ok=True)

    print("=" * 64)
    print(" AI-Based Autonomous Navigation System")
    print(f" scenario={world.name}  grid={world.width}x{world.height}  planner={cfg.algorithm.upper()}"
          f"  prior_map={'no' if args.no_prior else 'yes'}")
    print(f" start={world.start}  goal={world.goal}  moving obstacles={len(world.dynamic)}")
    print("=" * 64)

    dash = Dashboard(sim, live=args.live)
    frames = []
    shot_names = {"first_detection": "03_obstacle_detection.png", "first_replan": "04_replanning.png",
                  "first_avoid": "05_emergency_avoidance.png"}

    def on_step(s):
        for name, msg in s.new_events:
            print(f"[t={s.step_count * cfg.dt:6.1f}s] {name:<16} {msg}")
        capture = s.step_count % args.frame_skip == 0 or s.done
        if capture or args.live or any(n in shot_names for n, _ in s.new_events):
            dash.draw()
            if not args.no_gif and capture:
                frames.append(dash.frame())
            for n, _ in s.new_events:
                if n in shot_names:
                    dash.save(os.path.join(shots, shot_names[n]))

    dash.draw()
    dash.save(os.path.join(shots, "01_environment_start.png"))
    metrics = sim.run(on_step)
    dash.draw()
    dash.save(os.path.join(shots, "06_final_run.png"))

    plot_trajectory(sim, os.path.join(shots, "02_trajectory.png"))
    plot_telemetry(sim, os.path.join(shots, "07_telemetry.png"))
    if not args.no_gif and frames:
        frames += [frames[-1]] * 12               # hold the last frame
        save_gif(frames, os.path.join(args.out, "navigation_demo.gif"))
    with open(os.path.join(args.out, "metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2)

    print("-" * 64)
    print(" RESULT:", "SUCCESS - goal reached" if metrics["success"] else f"FAILED ({metrics['final_state']})")
    for k in ("sim_time_s", "path_length", "straight_line_distance", "path_efficiency", "replans",
              "planner_calls", "avg_plan_time_ms", "unexpected_obstacles_detected", "collisions", "min_clearance"):
        print(f"   {k:<32}{metrics[k]}")
    print(f" outputs saved in: {os.path.abspath(args.out)}/")
    print("=" * 64)
    return 0 if metrics["success"] else 1


if __name__ == "__main__":
    sys.exit(main())
