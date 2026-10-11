"""Dashboard rendering (Matplotlib): ground-truth world + the robot's own belief."""
from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon, Rectangle
from PIL import Image

C_FREE, C_WALL = (0.97, 0.97, 0.97), (0.22, 0.25, 0.30)
C_HIDDEN, C_MOVER = (0.96, 0.62, 0.20), (0.85, 0.15, 0.15)
C_INFL, C_NEW = (1.0, 0.88, 0.84), (0.93, 0.40, 0.10)
STATE_COLORS = {"NAVIGATING": "#2e7d32", "AVOIDING": "#c62828", "WAITING": "#ef6c00",
                "PLANNING": "#1565c0", "GOAL_REACHED": "#6a1b9a", "FAILED": "#000000"}


class Dashboard:
    def __init__(self, sim, live: bool = False, dpi: int = 72):
        self.sim, self.live = sim, live
        H, W = sim.world.static.shape
        self.fig, (self.ax_w, self.ax_b) = plt.subplots(1, 2, figsize=(14, 5.7), dpi=dpi)
        self.fig.subplots_adjust(left=0.02, right=0.99, top=0.82, bottom=0.07, wspace=0.05)
        ext = (0, W, H, 0)
        self.im_w = self.ax_w.imshow(self._truth_rgb(), extent=ext, interpolation="nearest")
        self.im_b = self.ax_b.imshow(self._belief_rgb(), extent=ext, interpolation="nearest")
        self.ax_w.set_title("Simulated world (ground truth)", fontsize=11)
        self.ax_b.set_title("Robot's belief: occupancy map + detections", fontsize=11)
        sr, sc = sim.world.start
        gr, gc = sim.world.goal
        for ax in (self.ax_w, self.ax_b):
            ax.set_xticks([]); ax.set_yticks([])
            ax.plot(sc + .5, sr + .5, "s", color="#1565c0", ms=7, zorder=5)
            ax.plot(gc + .5, gr + .5, "*", color="#2e7d32", ms=15, zorder=5)
        (self.trail_w,) = self.ax_w.plot([], [], "-", color="#1565c0", lw=1.6, zorder=4)
        (self.path_b,) = self.ax_b.plot([], [], "-", color="#2e7d32", lw=2, zorder=4)
        (self.trail_b,) = self.ax_b.plot([], [], "-", color="#1565c0", lw=1.2, alpha=.8, zorder=4)
        self.rays = LineCollection([], colors="#4fc3f7", linewidths=0.5, alpha=0.55, zorder=3)
        self.ax_w.add_collection(self.rays)
        self.robot_w = Polygon(np.zeros((3, 2)), closed=True, fc="#1565c0", ec="white", zorder=6)
        self.robot_b = Polygon(np.zeros((3, 2)), closed=True, fc="#1565c0", ec="white", zorder=6)
        self.ax_w.add_patch(self.robot_w); self.ax_b.add_patch(self.robot_b)
        self.det_artists = []
        self.title = self.fig.text(0.5, 0.95, "", ha="center", va="center", fontsize=13, fontweight="bold")
        self.hud = self.fig.text(0.5, 0.895, "", ha="center", va="center", fontsize=10, family="monospace")
        self.fig.text(0.5, 0.025, "blue square = start   green star = goal   orange = unexpected obstacle   "
                      "red box = moving obstacle   pink = safety margin   green line = planned path",
                      ha="center", fontsize=9, color="#444")
        if live:
            plt.ion(); plt.show(block=False)

    # ---------------------------------------------------------------- images
    def _truth_rgb(self):
        w = self.sim.world
        img = np.ones((*w.static.shape, 3)) * C_FREE
        img[w.static] = C_WALL
        img[w.hidden] = C_HIDDEN
        img[w.dynamic_mask()] = C_MOVER
        return img

    def _belief_rgb(self):
        m, cfg = self.sim.agent.map, self.sim.cfg
        occ = m.occupied()
        img = np.ones((*occ.shape, 3)) * C_FREE
        img[m.inflated(cfg.inflation_cells) & ~occ] = C_INFL
        img[occ] = C_WALL
        img[occ & ~m.prior] = C_NEW
        return img

    @staticmethod
    def _tri(x, y, th, s=0.9):
        pts = np.array([[1.0, 0.0], [-0.7, 0.6], [-0.7, -0.6]]) * s
        rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        return pts @ rot.T + [x, y]

    # ------------------------------------------------------------------ draw
    def draw(self):
        sim, a = self.sim, self.sim.agent
        x, y, th = a.pose
        self.im_w.set_data(self._truth_rgb())
        self.im_b.set_data(self._belief_rgb())
        tr = np.array(sim.trail)
        self.trail_w.set_data(tr[:, 0], tr[:, 1]); self.trail_b.set_data(tr[:, 0], tr[:, 1])
        if len(a.waypoints):
            self.path_b.set_data(a.waypoints[:, 0], a.waypoints[:, 1])
        for p in (self.robot_w, self.robot_b):
            p.set_xy(self._tri(x, y, th))
        if a.scan is not None:
            ang = th + a.scan.rel_angles
            ends = np.stack([x + a.scan.ranges * np.cos(ang), y + a.scan.ranges * np.sin(ang)], 1)
            self.rays.set_segments([[(x, y), tuple(e)] for e in ends])
        for art in self.det_artists:
            art.remove()
        self.det_artists = []
        for d in a.detections:
            bx, by, bw, bh = d.bbox
            col = "#d50000" if d.moving else "#ff6d00"
            self.det_artists.append(self.ax_b.add_patch(
                Rectangle((bx - .5, by - .5), bw + 1, bh + 1, fill=False, ec=col, lw=1.8, zorder=7)))
            self.det_artists.append(self.ax_b.text(bx - .5, by - .9, f"{'MOVING' if d.moving else 'STATIC'} #{d.id}",
                                                    color=col, fontsize=7, fontweight="bold", zorder=8))
        self.title.set_text(f"AI-Based Autonomous Navigation System  |  {sim.world.name}  |  {a.state.value}")
        self.title.set_color(STATE_COLORS.get(a.state.value, "k"))
        self.hud.set_text(f"t={sim.step_count * sim.cfg.dt:5.1f}s  dist={sim.path_len:5.1f}m  replans={a.replans:3d}  "
                          f"front={a.front_min:4.1f}m  detected={a.detector.unique_obstacles:2d}  "
                          f"collisions={sim.collisions}  planner={sim.cfg.algorithm.upper()}")
        self.fig.canvas.draw()
        if self.live:
            plt.pause(0.001)

    def frame(self) -> np.ndarray:
        return np.asarray(self.fig.canvas.buffer_rgba())[..., :3].copy()

    def save(self, path: str) -> None:
        self.fig.savefig(path, dpi=110)

    def close(self):
        plt.close(self.fig)


def save_gif(frames: list, path: str, fps: int = 12, colors: int = 80) -> None:
    imgs = [Image.fromarray(f).convert("P", palette=Image.ADAPTIVE, colors=colors) for f in frames]
    imgs[0].save(path, save_all=True, append_images=imgs[1:], duration=int(1000 / fps), loop=0, optimize=True)


def plot_trajectory(sim, path: str) -> None:
    w = sim.world
    img = np.ones((*w.static.shape, 3)) * C_FREE
    img[w.static] = C_WALL; img[w.hidden] = C_HIDDEN
    fig, ax = plt.subplots(figsize=(10, 6.5))
    H, W = w.static.shape
    ax.imshow(img, extent=(0, W, H, 0), interpolation="nearest")
    tr = np.array(sim.trail)
    ax.plot(tr[:, 0], tr[:, 1], "-", color="#1565c0", lw=2, label="robot trajectory")
    ax.plot(w.start[1] + .5, w.start[0] + .5, "s", color="#1565c0", ms=9, label="start")
    ax.plot(w.goal[1] + .5, w.goal[0] + .5, "*", color="#2e7d32", ms=16, label="goal")
    ax.set_title(f"Trajectory - {w.name} (dark = known layout, orange = obstacles found by sensor)")
    ax.legend(loc="upper right", fontsize=8); ax.set_xticks([]); ax.set_yticks([])
    fig.tight_layout(); fig.savefig(path, dpi=130); plt.close(fig)


def plot_telemetry(sim, path: str) -> None:
    lg = sim.log
    t = np.array(lg["t"])
    fig, axs = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    axs[0].plot(t, lg["v"], color="#1565c0"); axs[0].set_ylabel("speed (m/s)")
    axs[1].plot(t, lg["min_range"], color="#ef6c00", label="closest obstacle")
    axs[1].plot(t, lg["front"], color="#2e7d32", alpha=.8, label="front cone")
    axs[1].axhline(sim.cfg.stop_dist, color="r", ls="--", lw=1, label="emergency stop distance")
    axs[1].set_ylabel("range (m)"); axs[1].legend(fontsize=8, ncol=3)
    names = list(STATE_COLORS)
    axs[2].step(t, [names.index(s) for s in lg["state"]], where="post", color="#6a1b9a")
    axs[2].set_yticks(range(len(names))); axs[2].set_yticklabels(names, fontsize=8)
    axs[2].set_xlabel("simulated time (s)"); axs[2].set_ylabel("agent state")
    fig.suptitle(f"Navigation telemetry - {sim.world.name}"); fig.tight_layout()
    fig.savefig(path, dpi=130); plt.close(fig)
