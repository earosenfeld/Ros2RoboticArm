"""Joint- and Cartesian-space trajectory generation.

Pure-Python (numpy only). NO ROS / rclpy imports.

* :func:`quintic_polynomial`  — smooth point-to-point with zero end velocity
  and acceleration (vectorized over joints).
* :func:`trapezoidal_profile` — trapezoidal velocity profile honoring vmax/amax.
* :func:`cartesian_line`      — straight-line Cartesian motion solved via IK at
  interpolated waypoints, with an end-effector tracking-error report.
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

from .kinematics import (
    RobotKinematics,
    axis_angle_to_matrix,
    default_kinematics,
    homogeneous,
    rotation_matrix_to_axis_angle,
)


# ---------------------------------------------------------------------------
# Quintic (5th-order) polynomial — zero vel & accel at both ends
# ---------------------------------------------------------------------------


def quintic_polynomial(
    q0, qf, T: float, n: int = 100
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """5th-order polynomial joint trajectory from ``q0`` to ``qf`` over ``T`` s.

    Boundary conditions: ``q(0)=q0``, ``q(T)=qf`` and zero velocity *and*
    acceleration at both endpoints. With these six constraints the unique
    solution is

        q(t) = q0 + (qf - q0) * (10 s^3 - 15 s^4 + 6 s^5),   s = t / T

    Returns ``(t, q, qd, qdd)`` where ``t`` is ``(n,)`` and the others are
    ``(n, dof)`` (or ``(n,)`` if scalar inputs were given).
    """
    q0 = np.atleast_1d(np.asarray(q0, dtype=float))
    qf = np.atleast_1d(np.asarray(qf, dtype=float))
    scalar = q0.shape == (1,) and np.isscalar(T)  # noqa: F841 (kept for clarity)
    if T <= 0:
        raise ValueError("Trajectory duration T must be positive.")

    t = np.linspace(0.0, T, n)
    s = t / T  # normalized time in [0, 1]

    # Normalized position/vel/accel shape functions.
    f = 10 * s**3 - 15 * s**4 + 6 * s**5
    fd = (30 * s**2 - 60 * s**3 + 30 * s**4) / T
    fdd = (60 * s - 180 * s**2 + 120 * s**3) / (T**2)

    delta = (qf - q0)  # (dof,)
    q = q0[None, :] + f[:, None] * delta[None, :]
    qd = fd[:, None] * delta[None, :]
    qdd = fdd[:, None] * delta[None, :]

    if q0.shape == (1,):
        return t, q[:, 0], qd[:, 0], qdd[:, 0]
    return t, q, qd, qdd


# ---------------------------------------------------------------------------
# Trapezoidal velocity profile
# ---------------------------------------------------------------------------


def _trapezoidal_scalar(
    q0: float, qf: float, vmax: float, amax: float, dt: float
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Single-joint trapezoidal (or triangular) profile. Returns t,q,qd,qdd."""
    D = qf - q0
    dist = abs(D)
    direction = np.sign(D) if D != 0 else 1.0

    vmax = abs(vmax)
    amax = abs(amax)

    if dist < 1e-12 or vmax < 1e-12 or amax < 1e-12:
        t = np.array([0.0])
        return t, np.array([qf]), np.array([0.0]), np.array([0.0])

    # Distance covered ramping up to vmax (and symmetric ramp down).
    d_acc = vmax**2 / (2.0 * amax)

    if 2.0 * d_acc <= dist:
        # Trapezoid: accelerate to vmax, cruise, decelerate.
        t_acc = vmax / amax
        d_cruise = dist - 2.0 * d_acc
        t_cruise = d_cruise / vmax
        v_peak = vmax
    else:
        # Triangle: never reach vmax. Peak speed where the two ramps meet.
        t_acc = np.sqrt(dist / amax)
        t_cruise = 0.0
        v_peak = amax * t_acc

    T_total = 2.0 * t_acc + t_cruise
    n = max(int(np.ceil(T_total / dt)) + 1, 2)
    t = np.linspace(0.0, T_total, n)

    q = np.empty(n)
    qd = np.empty(n)
    qdd = np.empty(n)

    t1 = t_acc
    t2 = t_acc + t_cruise
    d_acc_actual = 0.5 * amax * t_acc**2  # distance at end of accel phase

    for i, ti in enumerate(t):
        if ti <= t1:  # accelerate
            a = amax
            v = amax * ti
            p = 0.5 * amax * ti**2
        elif ti <= t2:  # cruise
            a = 0.0
            v = v_peak
            p = d_acc_actual + v_peak * (ti - t1)
        else:  # decelerate
            td = ti - t2
            a = -amax
            v = v_peak - amax * td
            p = d_acc_actual + v_peak * t_cruise + v_peak * td - 0.5 * amax * td**2
        qdd[i] = direction * a
        qd[i] = direction * v
        q[i] = q0 + direction * p

    # Pin the final sample exactly on target (kill float drift).
    q[-1] = qf
    qd[-1] = 0.0
    qdd[-1] = 0.0
    return t, q, qd, qdd


def trapezoidal_profile(
    q0, qf, vmax, amax, dt: float = 0.01
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Trapezoidal-velocity joint trajectory honoring ``vmax`` and ``amax``.

    For multi-joint inputs every joint is time-scaled to the slowest joint's
    duration so they start and finish together (synchronized), while still
    respecting each joint's ``|qd| <= vmax`` and ``|qdd| <= amax``.

    Returns ``(t, q, qd, qdd)`` with ``q`` shaped ``(n,)`` for scalar input or
    ``(n, dof)`` otherwise.
    """
    q0 = np.atleast_1d(np.asarray(q0, dtype=float))
    qf = np.atleast_1d(np.asarray(qf, dtype=float))
    dof = q0.shape[0]

    vmax_arr = np.broadcast_to(np.atleast_1d(np.asarray(vmax, float)), (dof,))
    amax_arr = np.broadcast_to(np.atleast_1d(np.asarray(amax, float)), (dof,))

    # Single joint: return its native profile directly.
    if dof == 1:
        return _trapezoidal_scalar(
            q0[0], qf[0], vmax_arr[0], amax_arr[0], dt
        )

    # Multi-joint: build each joint's profile, then synchronize on a common clock.
    profiles = [
        _trapezoidal_scalar(q0[j], qf[j], vmax_arr[j], amax_arr[j], dt)
        for j in range(dof)
    ]
    T_total = max(p[0][-1] for p in profiles)

    if T_total <= 0:
        t = np.array([0.0])
        return t, qf[None, :], np.zeros((1, dof)), np.zeros((1, dof))

    n = max(int(np.ceil(T_total / dt)) + 1, 2)
    t = np.linspace(0.0, T_total, n)

    q = np.empty((n, dof))
    qd = np.empty((n, dof))
    qdd = np.empty((n, dof))
    for j, (tj, qj, qdj, qddj) in enumerate(profiles):
        # Time-scale joint j so it spans the common horizon (scale <= 1 keeps
        # |qd| and |qdd| within their per-joint limits).
        scale = tj[-1] / T_total if tj[-1] > 0 else 0.0
        t_local = t * scale
        q[:, j] = np.interp(t_local, tj, qj)
        qd[:, j] = np.interp(t_local, tj, qdj) * scale
        qdd[:, j] = np.interp(t_local, tj, qddj) * (scale**2)
    return t, q, qd, qdd


# ---------------------------------------------------------------------------
# Cartesian straight-line motion via IK
# ---------------------------------------------------------------------------


def _slerp_rotation(R0: np.ndarray, R1: np.ndarray, s: float) -> np.ndarray:
    """Interpolate between two rotations along the shortest geodesic (SLERP).

    Implemented in axis-angle form: take the relative rotation R0^T R1, scale its
    angle by ``s``, and re-apply.
    """
    R_rel = R0.T @ R1
    axis, angle = rotation_matrix_to_axis_angle(R_rel)
    return R0 @ axis_angle_to_matrix(axis, angle * s)


def cartesian_line(
    pose_start: np.ndarray,
    pose_goal: np.ndarray,
    n: int,
    q_seed,
    kin: Optional[RobotKinematics] = None,
    tol: float = 1e-5,
    num_restarts: int = 0,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Joint path tracing a straight Cartesian line from ``pose_start`` to ``pose_goal``.

    Position is linearly interpolated; orientation is SLERP-interpolated. Each
    waypoint is solved by damped-least-squares IK warm-started from the previous
    solution. ``num_restarts`` defaults to 0 so the solver stays on a single IK
    branch — random restarts would teleport between elbow/wrist configurations
    and break the path's joint-space continuity.

    Returns
    -------
    q_path : (n, dof) joint angles at each waypoint.
    waypoints : (n, 4, 4) the commanded Cartesian poses.
    tracking_error : (n,) Euclidean position error ||p_fk - p_cmd|| at each point.
    """
    kin = kin if kin is not None else default_kinematics()

    p0 = pose_start[:3, 3]
    p1 = pose_goal[:3, 3]
    R0 = pose_start[:3, :3]
    R1 = pose_goal[:3, :3]

    q = np.asarray(q_seed, dtype=float).reshape(-1).copy()

    q_path = np.empty((n, kin.n))
    waypoints = np.empty((n, 4, 4))
    tracking_error = np.empty(n)

    for i in range(n):
        s = i / (n - 1) if n > 1 else 0.0
        p = (1.0 - s) * p0 + s * p1
        R = _slerp_rotation(R0, R1, s)
        T_cmd = homogeneous(R=R, p=p)
        waypoints[i] = T_cmd

        q, _success, _iters = kin.inverse_kinematics(
            T_cmd, q_init=q, tol=tol, max_iters=300, num_restarts=num_restarts
        )
        q_path[i] = q

        T_fk = kin.forward_kinematics(q)
        tracking_error[i] = float(np.linalg.norm(T_fk[:3, 3] - p))

    return q_path, waypoints, tracking_error
