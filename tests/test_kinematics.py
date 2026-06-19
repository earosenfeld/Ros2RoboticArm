"""Forward kinematics, Jacobian, and inverse-kinematics correctness tests."""

import numpy as np
import pytest

from robot_arm import (
    RobotKinematics,
    default_kinematics,
    forward_kinematics,
    geometric_jacobian,
)
from robot_arm.kinematics import (
    axis_angle_to_matrix,
    rotation_error,
    rotation_matrix_to_axis_angle,
    rpy_to_matrix,
)


# ---------------------------------------------------------------------------
# Transform helpers
# ---------------------------------------------------------------------------


def test_rpy_identity():
    assert np.allclose(rpy_to_matrix([0, 0, 0]), np.eye(3))


def test_rpy_known_yaw():
    R = rpy_to_matrix([0, 0, np.pi / 2])
    assert np.allclose(R @ np.array([1, 0, 0]), [0, 1, 0], atol=1e-12)


def test_axis_angle_roundtrip():
    rng = np.random.default_rng(3)
    for _ in range(20):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        angle = rng.uniform(0.01, np.pi - 0.01)
        R = axis_angle_to_matrix(axis, angle)
        ax2, ang2 = rotation_matrix_to_axis_angle(R)
        # axis*angle should match up to sign convention
        assert np.allclose(ax2 * ang2, axis * angle, atol=1e-9) or np.allclose(
            ax2 * ang2, -axis * -angle, atol=1e-9
        )


def test_axis_angle_at_pi():
    """Recover the axis for a 180 deg rotation (the sin(theta)->0 singularity)."""
    for axis in (np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])):
        R = axis_angle_to_matrix(axis, np.pi)
        ax2, ang2 = rotation_matrix_to_axis_angle(R)
        assert abs(ang2 - np.pi) < 1e-6
        assert np.allclose(np.abs(ax2), np.abs(axis), atol=1e-6)


# ---------------------------------------------------------------------------
# Forward kinematics
# ---------------------------------------------------------------------------


def test_fk_at_zero_equals_origin_sum():
    """FK at q=0 must equal the summed URDF origin translation to end_effector.

    Origins along the default chain (all rpy=0): joint1 0.05, joint2 0.15,
    joint3 0.15, joint4 0.15, joint5 0.10, joint6 0.075, gripper_joint 0.05,
    end_effector_joint 0.025 -> 0.75 along +z. With all joints at zero the
    orientation is identity.
    """
    rk = default_kinematics()
    T = rk.forward_kinematics(np.zeros(rk.n))
    expected_z = 0.05 + 0.15 + 0.15 + 0.15 + 0.10 + 0.075 + 0.05 + 0.025
    assert np.allclose(T[:3, 3], [0.0, 0.0, expected_z], atol=1e-9)
    assert np.allclose(T[:3, :3], np.eye(3), atol=1e-9)
    assert abs(expected_z - 0.75) < 1e-12


def test_fk_module_wrapper_matches_class():
    rk = default_kinematics()
    q = np.array([0.1, -0.2, 0.3, -0.4, 0.5, -0.6])
    assert np.allclose(forward_kinematics(q), rk.forward_kinematics(q))


def test_link_frames_contains_all_links():
    rk = default_kinematics()
    frames = rk.link_frames(np.zeros(rk.n))
    assert rk.chain.base_link in frames
    assert rk.chain.tip_link in frames
    for j in rk.chain.joints:
        assert j.child in frames
    # base link is identity
    assert np.allclose(frames[rk.chain.base_link], np.eye(4))


def test_fk_joint1_pure_rotation_about_z():
    """Rotating joint1 by 90 deg keeps the tip on the z-axis (axis passes through it)."""
    rk = default_kinematics()
    q = np.zeros(rk.n)
    q[0] = np.pi / 2
    T = rk.forward_kinematics(q)
    # Tip stays at (0, 0, 0.75); only orientation rotates about z.
    assert np.allclose(T[:3, 3], [0, 0, 0.75], atol=1e-9)
    assert np.allclose(T[:3, :3], rpy_to_matrix([0, 0, np.pi / 2]), atol=1e-9)


# ---------------------------------------------------------------------------
# Jacobian vs finite difference
# ---------------------------------------------------------------------------


def _finite_difference_jacobian(rk, q, eps=1e-6):
    J = np.zeros((6, rk.n))
    T0 = rk.forward_kinematics(q)
    for i in range(rk.n):
        dq = np.zeros(rk.n)
        dq[i] = eps
        Tp = rk.forward_kinematics(q + dq)
        J[:3, i] = (Tp[:3, 3] - T0[:3, 3]) / eps
        R_rel = Tp[:3, :3] @ T0[:3, :3].T
        ax, ang = rotation_matrix_to_axis_angle(R_rel)
        J[3:, i] = ax * ang / eps
    return J


def test_geometric_jacobian_matches_finite_difference():
    rk = default_kinematics()
    rng = np.random.default_rng(7)
    for _ in range(25):
        q = rk.random_valid_q(rng)
        J = rk.geometric_jacobian(q)
        Jfd = _finite_difference_jacobian(rk, q)
        assert np.max(np.abs(J - Jfd)) < 1e-5


def test_jacobian_module_wrapper_matches_class():
    rk = default_kinematics()
    q = np.array([0.2, 0.3, -0.1, 0.4, -0.5, 0.6])
    assert np.allclose(geometric_jacobian(q), rk.geometric_jacobian(q))


def test_jacobian_shape():
    rk = default_kinematics()
    assert rk.geometric_jacobian(np.zeros(rk.n)).shape == (6, 6)


# ---------------------------------------------------------------------------
# Inverse kinematics
# ---------------------------------------------------------------------------

IK_SEED = 12345


def _reachable_targets(rk, n, seed):
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        q = rk.random_valid_q(rng)
        out.append((q, rk.forward_kinematics(q)))
    return out


def test_ik_roundtrip_random_reachable_targets():
    """FK . IK round trip: recover a config whose FK matches the target."""
    rk = default_kinematics()
    targets = _reachable_targets(rk, 50, IK_SEED)
    n_ok = 0
    for _q_true, T_target in targets:
        q_sol, success, _iters = rk.inverse_kinematics(T_target, tol=1e-4)
        T_sol = rk.forward_kinematics(q_sol)
        pos_err = np.linalg.norm(T_sol[:3, 3] - T_target[:3, 3])
        ori_err = np.linalg.norm(
            rotation_error(T_target[:3, :3], T_sol[:3, :3])
        )
        assert pos_err < 1e-3, f"position error too large: {pos_err}"
        assert ori_err < 1e-3, f"orientation error too large: {ori_err}"
        n_ok += 1
    assert n_ok == 50


def test_ik_respects_joint_limits():
    rk = default_kinematics()
    targets = _reachable_targets(rk, 50, IK_SEED + 1)
    for _q_true, T_target in targets:
        q_sol, _success, _iters = rk.inverse_kinematics(T_target, tol=1e-4)
        assert np.all(q_sol >= rk.lower - 1e-9)
        assert np.all(q_sol <= rk.upper + 1e-9)


def test_ik_reports_success_flag():
    rk = default_kinematics()
    _q_true, T_target = _reachable_targets(rk, 1, IK_SEED + 2)[0]
    q_sol, success, iters = rk.inverse_kinematics(T_target, tol=1e-4)
    assert success is True
    assert iters >= 1
    assert np.linalg.norm(rk.pose_error(T_target, q_sol)) < 1e-3


def test_manipulability_drops_at_singularity():
    """At the wrist singularity (joint5=0, joints 4&6 collinear) manipulability ~ 0."""
    rk = default_kinematics()
    q_sing = np.zeros(rk.n)  # j5 = 0 -> wrist axes 4 and 6 aligned
    q_reg = np.array([0.3, 0.5, -0.7, 0.2, 0.9, -0.4])
    assert rk.manipulability(q_sing) < rk.manipulability(q_reg)
    assert rk.manipulability(q_sing) < 1e-6


def test_clamp_to_limits():
    rk = default_kinematics()
    q = rk.upper + 1.0
    clamped = rk.clamp_to_limits(q)
    assert np.all(clamped <= rk.upper + 1e-12)
    assert np.allclose(clamped, rk.upper)
