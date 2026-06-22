#!/usr/bin/env python3
"""Render kinematics figures into ``assets/`` (headless, pure-Python).

Run:  .venv/bin/python scripts/make_figures.py

Produces:
  * assets/workspace.png          — reachable end-effector cloud (FK of random q)
  * assets/ik_cartesian_path.png  — a commanded Cartesian line solved by IK
  * assets/trajectory_profile.png — quintic position/velocity/acceleration vs time
"""

import os
import sys

import numpy as np

# Make ``robot_arm`` importable when run from anywhere.
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams.update({
    "figure.dpi": 130, "savefig.dpi": 130, "savefig.bbox": "tight",
    "figure.facecolor": "white", "axes.facecolor": "white",
    "axes.edgecolor": "#334155", "axes.linewidth": 0.8,
    "axes.grid": True, "grid.color": "#e2e8f0", "grid.linewidth": 0.7,
    "axes.spines.top": False, "axes.spines.right": False,
    "font.size": 11, "axes.titlesize": 13, "axes.titleweight": "bold",
    "axes.labelsize": 11, "legend.frameon": False, "lines.linewidth": 2.0,
})
PALETTE = ["#2563eb", "#dc2626", "#059669", "#d97706", "#7c3aed", "#0891b2"]

from robot_arm import (  # noqa: E402
    Box,
    cartesian_line,
    default_kinematics,
    path_length,
    quintic_polynomial,
    rrt_connect,
    shortcut_path,
)

ASSETS = os.path.join(ROOT, "assets")
os.makedirs(ASSETS, exist_ok=True)


# ---------------------------------------------------------------------------
# Figure 1 — reachable workspace
# ---------------------------------------------------------------------------


def figure_workspace(n_samples: int = 6000, seed: int = 0) -> str:
    rk = default_kinematics()
    rng = np.random.default_rng(seed)

    pts = np.empty((n_samples, 3))
    for i in range(n_samples):
        q = rk.random_valid_q(rng)
        pts[i] = rk.forward_kinematics(q)[:3, 3]

    fig, axes = plt.subplots(1, 2, figsize=(11, 5))

    # X-Z side view.
    ax = axes[0]
    ax.scatter(pts[:, 0], pts[:, 2], s=3, c=PALETTE[0], alpha=0.25, edgecolors="none")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Z (m)")
    ax.set_title("Reachable workspace — side view (X-Z)")
    ax.set_aspect("equal", adjustable="box")

    # X-Y top view.
    ax = axes[1]
    ax.scatter(pts[:, 0], pts[:, 1], s=3, c=PALETTE[4], alpha=0.25, edgecolors="none")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Reachable workspace — top view (X-Y)")
    ax.set_aspect("equal", adjustable="box")

    fig.suptitle(
        f"6-DOF arm end-effector workspace ({n_samples} random configs)",
        fontsize=14, fontweight="bold",
    )
    out = os.path.join(ASSETS, "workspace.png")
    fig.savefig(out)
    plt.close(fig)
    return out


# ---------------------------------------------------------------------------
# Figure 2 — Cartesian straight line solved by IK
# ---------------------------------------------------------------------------


def figure_ik_cartesian_path(n: int = 60) -> str:
    rk = default_kinematics()

    # Folded seed well inside the workspace, then a straight Cartesian move.
    q_seed = np.array([0.3, 0.8, -1.2, 0.2, 0.6, -0.3])
    pose_start = rk.forward_kinematics(q_seed)
    pose_goal = pose_start.copy()
    pose_goal[:3, 3] += np.array([0.06, -0.04, 0.04])

    q_path, waypoints, err = cartesian_line(
        pose_start, pose_goal, n=n, q_seed=q_seed
    )

    # Actual achieved EE path (FK of the IK solutions).
    ee = np.array([rk.forward_kinematics(q)[:3, 3] for q in q_path])
    cmd = waypoints[:, :3, 3]
    s = np.linspace(0.0, 1.0, n)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6))

    # (a) Commanded vs achieved path in X-Z.
    ax = axes[0]
    ax.plot(cmd[:, 0], cmd[:, 2], "--", color="#94a3b8", label="commanded")
    ax.plot(ee[:, 0], ee[:, 2], color=PALETTE[1], label="achieved (FK of IK)")
    ax.scatter([cmd[0, 0]], [cmd[0, 2]], color=PALETTE[2], zorder=5, label="start")
    ax.scatter([cmd[-1, 0]], [cmd[-1, 2]], color=PALETTE[3], zorder=5, label="goal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Z (m)")
    ax.set_title("End-effector path (X-Z)")
    ax.legend(loc="best", fontsize=9)
    ax.set_aspect("equal", adjustable="datalim")

    # (b) Joint angles along the path.
    ax = axes[1]
    for j in range(rk.n):
        ax.plot(s, q_path[:, j], color=PALETTE[j % len(PALETTE)],
                label=rk.chain.actuated_joint_names[j])
    ax.set_xlabel("path parameter s")
    ax.set_ylabel("joint angle (rad)")
    ax.set_title("Joint angles along the path")
    ax.legend(loc="best", ncol=2, fontsize=8)

    # (c) Cartesian tracking error.
    ax = axes[2]
    ax.plot(s, err * 1e3, color=PALETTE[5])
    ax.set_xlabel("path parameter s")
    ax.set_ylabel("tracking error (mm)")
    ax.set_title("IK Cartesian tracking error")

    fig.suptitle(
        "Cartesian straight-line motion solved by damped-least-squares IK",
        fontsize=14, fontweight="bold",
    )
    out = os.path.join(ASSETS, "ik_cartesian_path.png")
    fig.savefig(out)
    plt.close(fig)
    return out


# ---------------------------------------------------------------------------
# Figure 3 — quintic trajectory profile
# ---------------------------------------------------------------------------


def figure_trajectory_profile() -> str:
    q0, qf, T = -0.5, 1.2, 2.5
    t, q, qd, qdd = quintic_polynomial(q0, qf, T=T, n=400)

    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)

    axes[0].plot(t, q, color=PALETTE[0])
    axes[0].axhline(q0, ls=":", color="#94a3b8")
    axes[0].axhline(qf, ls=":", color="#94a3b8")
    axes[0].set_ylabel("position (rad)")
    axes[0].set_title("Quintic point-to-point trajectory (joint 2)")

    axes[1].plot(t, qd, color=PALETTE[2])
    axes[1].set_ylabel("velocity (rad/s)")

    axes[2].plot(t, qdd, color=PALETTE[1])
    axes[2].set_ylabel("acceleration (rad/s²)")
    axes[2].set_xlabel("time (s)")

    fig.suptitle(
        "Quintic profile — zero velocity & acceleration at both ends",
        fontsize=14, fontweight="bold",
    )
    out = os.path.join(ASSETS, "trajectory_profile.png")
    fig.savefig(out)
    plt.close(fig)
    return out


# ---------------------------------------------------------------------------
# Figure 4 — RRT-Connect motion planning around an obstacle
# ---------------------------------------------------------------------------


def _ee_curve(rk, path, per_edge: int = 10) -> np.ndarray:
    """End-effector positions (FK) sampled densely along a joint-space path."""
    pts = []
    for a, b in zip(path[:-1], path[1:]):
        for t in np.linspace(0.0, 1.0, per_edge, endpoint=False):
            pts.append(rk.forward_kinematics(a + t * (b - a))[:3, 3])
    pts.append(rk.forward_kinematics(path[-1])[:3, 3])
    return np.asarray(pts)


def _draw_box(ax, box: Box, a0: int, a1: int, label=None) -> None:
    """Draw a Box obstacle's (a0, a1)-plane projection as a filled rectangle."""
    lo, hi = box.lower, box.upper
    x0, x1 = lo[a0], hi[a0]
    y0, y1 = lo[a1], hi[a1]
    ax.add_patch(
        plt.Rectangle(
            (x0, y0), x1 - x0, y1 - y0,
            facecolor=PALETTE[1], edgecolor=PALETTE[1], alpha=0.18,
            linewidth=1.5, label=label, zorder=1,
        )
    )


def figure_motion_planning(seed: int = 7) -> str:
    rk = default_kinematics()
    limits = rk.chain.joint_limits()

    # Start tilts the arm to +y, goal to -y; a thin wall in the y=0 plane at the
    # outer reach blocks the straight joint interpolation, so the planner must
    # route around it (this is the exact scenario the tests exercise).
    q_start = np.array([0.9, 0.95, -0.6, 0.0, 0.7, 0.0])
    q_goal = np.array([-0.9, 0.95, -0.6, 0.0, 0.7, 0.0])
    wall = Box(center=[0.30, 0.0, 0.46], size=[0.34, 0.06, 0.50])

    raw, info = rrt_connect(
        q_start, q_goal, [wall], limits, max_iters=5000, step=0.1, seed=seed,
    )
    if raw is None:  # pragma: no cover - the scenario is solvable
        raise RuntimeError(f"planner found no path: {info}")
    short = shortcut_path(raw, [wall], iters=200, seed=seed)

    ee_raw = _ee_curve(rk, raw)
    ee_short = _ee_curve(rk, short)
    p_start = rk.forward_kinematics(q_start)[:3, 3]
    p_goal = rk.forward_kinematics(q_goal)[:3, 3]

    fig, axes = plt.subplots(1, 2, figsize=(12, 5.2))

    for ax, (a0, a1, xl, yl, title) in zip(
        axes,
        [(0, 2, "X (m)", "Z (m)", "End-effector path — side view (X-Z)"),
         (0, 1, "X (m)", "Y (m)", "End-effector path — top view (X-Y)")],
    ):
        _draw_box(ax, wall, a0, a1, label="obstacle")
        ax.plot(ee_raw[:, a0], ee_raw[:, a1], color="#94a3b8",
                linewidth=1.6, label=f"raw RRT ({path_length(raw):.1f} rad)")
        ax.plot(ee_short[:, a0], ee_short[:, a1], color=PALETTE[0],
                label=f"shortcutted ({path_length(short):.1f} rad)")
        ax.scatter([p_start[a0]], [p_start[a1]], color=PALETTE[2],
                   zorder=5, s=60, label="start")
        ax.scatter([p_goal[a0]], [p_goal[a1]], color=PALETTE[3],
                   zorder=5, s=60, label="goal")
        ax.set_xlabel(xl)
        ax.set_ylabel(yl)
        ax.set_title(title)
        ax.set_aspect("equal", adjustable="datalim")

    axes[0].legend(loc="best", fontsize=8)

    fig.suptitle(
        "RRT-Connect: collision-free joint-space path around an obstacle "
        "(raw vs shortcutted)",
        fontsize=14, fontweight="bold",
    )
    out = os.path.join(ASSETS, "motion_planning.png")
    fig.savefig(out)
    plt.close(fig)
    return out


def main() -> None:
    outputs = [
        figure_workspace(),
        figure_ik_cartesian_path(),
        figure_trajectory_profile(),
        figure_motion_planning(),
    ]
    for path in outputs:
        size = os.path.getsize(path)
        print(f"wrote {path} ({size} bytes)")


if __name__ == "__main__":
    main()
