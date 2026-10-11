"""Compare A* and Dijkstra on random maps (nodes expanded, runtime, path cost).

    python benchmark.py            # 30 maps
    python benchmark.py --maps 50
"""
import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from src.planner import plan
from src.utils import inflate
from src.world import build_random


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--maps", type=int, default=30)
    ap.add_argument("--out", default="outputs")
    args = ap.parse_args()
    rows = []
    for seed in range(args.maps):
        w = build_random(seed)
        blocked = inflate(w.static | w.hidden, 1)
        a = plan(blocked, w.start, w.goal, "astar")
        d = plan(blocked, w.start, w.goal, "dijkstra")
        rows.append((a.expanded, d.expanded, a.runtime_ms, d.runtime_ms, a.cost, d.cost))
    r = np.array(rows)
    print(f"{'':<10}{'nodes expanded':>16}{'runtime (ms)':>16}{'path cost':>12}")
    print(f"{'A*':<10}{r[:, 0].mean():>16.0f}{r[:, 2].mean():>16.1f}{r[:, 4].mean():>12.1f}")
    print(f"{'Dijkstra':<10}{r[:, 1].mean():>16.0f}{r[:, 3].mean():>16.1f}{r[:, 5].mean():>12.1f}")
    print(f"A* expands {100 * (1 - r[:, 0].sum() / r[:, 1].sum()):.0f}% fewer nodes; "
          f"path costs identical: {bool(np.allclose(r[:, 4], r[:, 5]))}")

    os.makedirs(args.out, exist_ok=True)
    fig, axs = plt.subplots(1, 2, figsize=(10, 4))
    for ax, (i, title, unit) in zip(axs, [(0, "Nodes expanded", "nodes"), (2, "Planning time", "ms")]):
        ax.bar(["A*", "Dijkstra"], [r[:, i].mean(), r[:, i + 1].mean()], color=["#2e7d32", "#90a4ae"])
        ax.set_title(f"{title} (mean of {args.maps} random maps)"); ax.set_ylabel(unit)
    fig.tight_layout(); fig.savefig(os.path.join(args.out, "planner_comparison.png"), dpi=130)
    print("saved outputs/planner_comparison.png")


if __name__ == "__main__":
    main()
