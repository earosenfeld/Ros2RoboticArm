"""Trajectory-generation tests: quintic, trapezoidal, and Cartesian line."""

import numpy as np

from robot_arm import (
    cartesian_line,
    default_kinematics,
    quintic_polynomial,
    trapezoidal_profile,
)


# ---------------------------------------------------------------------------
# Quintic polynomial
# ---------------------------------------------------------------------------


def test_quintic_endpoints_exact_scalar():
    t, q, qd, qdd = quintic_polynomial(0.0, 1.5, T=2.0, n=200)
    assert np.isclose(q[0], 0.0, atol=1e-12)
    assert np.isclose(q[-1], 1.5, atol=1e-12)
    assert np.isclose(t[0], 0.0) and np.isclose(t[-1], 2.0)


def test_quintic_zero_velocity_and_acceleration_at_ends():
    t, q, qd, qdd = quintic_polynomial(-1.0, 2.0, T=3.0, n=300)
    assert np.isclose(qd[0], 0.0, atol=1e-9)
    assert np.isclose(qd[-1], 0.0, atol=1e-9)
    assert np.isclose(qdd[0], 0.0, atol=1e-9)
    assert np.isclose(qdd[-1], 0.0, atol=1e-9)


def test_quintic_vectorized_over_joints():
    q0 = np.array([0.0, 1.0, -2.0])
    qf = np.array([1.0, -1.0, 0.5])
    t, q, qd, qdd = quintic_polynomial(q0, qf, T=2.0, n=150)
    assert q.shape == (150, 3)
    assert np.allclose(q[0], q0, atol=1e-12)
    assert np.allclose(q[-1], qf, atol=1e-12)
    assert np.allclose(qd[0], 0.0, atol=1e-9)
    assert np.allclose(qd[-1], 0.0, atol=1e-9)
    assert np.allclose(qdd[0], 0.0, atol=1e-9)
    assert np.allclose(qdd[-1], 0.0, atol=1e-9)


def test_quintic_velocity_matches_position_derivative():
    t, q, qd, qdd = quintic_polynomial(0.0, 1.0, T=2.0, n=2000)
    qd_num = np.gradient(q, t)
    # interior points only (np.gradient is less accurate at the edges)
    assert np.max(np.abs(qd_num[5:-5] - qd[5:-5])) < 1e-3


# ---------------------------------------------------------------------------
# Trapezoidal profile
# ---------------------------------------------------------------------------


def test_trapezoidal_reaches_target_scalar():
    t, q, qd, qdd = trapezoidal_profile(0.0, 2.0, vmax=1.0, amax=2.0, dt=0.001)
    assert np.isclose(q[-1], 2.0, atol=1e-6)
    assert np.isclose(q[0], 0.0, atol=1e-9)


def test_trapezoidal_respects_vmax_and_amax():
    vmax, amax = 1.2, 3.0
    t, q, qd, qdd = trapezoidal_profile(-1.0, 3.0, vmax=vmax, amax=amax, dt=0.001)
    assert np.max(np.abs(qd)) <= vmax + 1e-6
    assert np.max(np.abs(qdd)) <= amax + 1e-6
    assert np.isclose(q[-1], 3.0, atol=1e-6)


def test_trapezoidal_negative_direction():
    t, q, qd, qdd = trapezoidal_profile(1.0, -2.0, vmax=1.0, amax=2.0, dt=0.001)
    assert np.isclose(q[-1], -2.0, atol=1e-6)
    assert np.max(np.abs(qd)) <= 1.0 + 1e-6


def test_trapezoidal_triangular_when_short():
    """A short move that never reaches vmax stays a triangle but still arrives."""
    vmax, amax = 5.0, 2.0  # high vmax -> triangular profile
    t, q, qd, qdd = trapezoidal_profile(0.0, 0.1, vmax=vmax, amax=amax, dt=0.001)
    assert np.isclose(q[-1], 0.1, atol=1e-6)
    assert np.max(np.abs(qd)) <= vmax + 1e-6
    assert np.max(np.abs(qdd)) <= amax + 1e-6


def test_trapezoidal_multijoint_synchronized():
    q0 = np.array([0.0, 0.0])
    qf = np.array([2.0, 0.5])  # joint 0 travels farther
    vmax = np.array([1.0, 1.0])
    amax = np.array([2.0, 2.0])
    t, q, qd, qdd = trapezoidal_profile(q0, qf, vmax, amax, dt=0.001)
    assert q.shape[1] == 2
    assert np.allclose(q[-1], qf, atol=1e-6)
    assert np.allclose(q[0], q0, atol=1e-9)
    # per-joint limits respected after synchronization
    assert np.all(np.abs(qd) <= vmax + 1e-6)
    assert np.all(np.abs(qdd) <= amax + 1e-6)


# ---------------------------------------------------------------------------
# Cartesian line via IK
# ---------------------------------------------------------------------------


def test_cartesian_line_tracks_straight_path():
    rk = default_kinematics()
    # Folded seed well inside the workspace (reach ~0.68 m vs ~0.75 m max).
    q_seed = np.array([0.3, 0.8, -1.2, 0.2, 0.6, -0.3])
    pose_start = rk.forward_kinematics(q_seed)

    # Short reachable move: +5 cm x, -3 cm y, +4 cm z, same orientation.
    pose_goal = pose_start.copy()
    pose_goal[:3, 3] += np.array([0.05, -0.03, 0.04])

    q_path, waypoints, err = cartesian_line(
        pose_start, pose_goal, n=25, q_seed=q_seed
    )
    assert q_path.shape == (25, 6)
    assert waypoints.shape == (25, 4, 4)
    # End effector tracks the commanded line closely at every waypoint.
    assert np.max(err) < 1e-3
    # Endpoints reached.
    assert np.allclose(
        rk.forward_kinematics(q_path[0])[:3, 3], pose_start[:3, 3], atol=1e-3
    )
    assert np.allclose(
        rk.forward_kinematics(q_path[-1])[:3, 3], pose_goal[:3, 3], atol=1e-3
    )


def test_cartesian_line_waypoints_are_collinear():
    rk = default_kinematics()
    q_seed = np.array([0.0, 0.6, -0.9, 0.0, 0.7, 0.0])
    pose_start = rk.forward_kinematics(q_seed)
    pose_goal = pose_start.copy()
    pose_goal[:3, 3] += np.array([0.05, 0.03, -0.02])

    _q_path, waypoints, _err = cartesian_line(
        pose_start, pose_goal, n=10, q_seed=q_seed
    )
    pts = waypoints[:, :3, 3]
    line = pts[-1] - pts[0]
    line /= np.linalg.norm(line)
    # Every interior waypoint lies on the start->goal line.
    for p in pts:
        d = p - pts[0]
        perp = d - np.dot(d, line) * line
        assert np.linalg.norm(perp) < 1e-9
