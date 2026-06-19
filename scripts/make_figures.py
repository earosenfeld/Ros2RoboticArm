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
    cartesian_line,
    default_kinematics,
    quintic_polynomial,
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


def main() -> None:
    outputs = [
        figure_workspace(),
        figure_ik_cartesian_path(),
        figure_trajectory_profile(),
    ]
    for path in outputs:
        size = os.path.getsize(path)
        print(f"wrote {path} ({size} bytes)")


if __name__ == "__main__":
    main()
