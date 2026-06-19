"""Forward / inverse kinematics and the geometric Jacobian for a serial chain.

Pure-Python (numpy only). NO ROS / rclpy imports.

Conventions
-----------
* All poses are 4x4 homogeneous transforms ``T = [[R, p], [0, 1]]`` expressing a
  frame in the chain's base frame (``base_link``).
* ``q`` is the actuated-joint vector, ordered as
  ``KinematicChain.actuated_joints`` (the 6 revolute joints for the default arm;
  the gripper prismatic comes last if included in the chain).
* For each joint the local transform is ``T_origin(xyz, rpy) @ T_motion(axis, q)``
  where a revolute joint rotates ``q`` about ``axis``, a prismatic joint
  translates ``q`` along ``axis``, and a fixed joint contributes only its origin.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import numpy as np

from .urdf_chain import KinematicChain, load_default_chain

# ---------------------------------------------------------------------------
# Basic rotation / transform helpers
# ---------------------------------------------------------------------------


def rotation_x(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rotation_y(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rotation_z(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def rpy_to_matrix(rpy) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw -> 3x3 rotation matrix.

    URDF uses the fixed-axis (extrinsic) X-Y-Z convention, which equals the
    intrinsic Z-Y-X product ``Rz(yaw) @ Ry(pitch) @ Rx(roll)``.
    """
    roll, pitch, yaw = (float(rpy[0]), float(rpy[1]), float(rpy[2]))
    return rotation_z(yaw) @ rotation_y(pitch) @ rotation_x(roll)


def axis_angle_to_matrix(axis, angle: float) -> np.ndarray:
    """Rodrigues' rotation: rotate ``angle`` rad about (unit) ``axis``."""
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.eye(3)
    kx, ky, kz = axis / n
    K = np.array([[0, -kz, ky], [kz, 0, -kx], [-ky, kx, 0]], dtype=float)
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)


def homogeneous(R=None, p=None) -> np.ndarray:
    """Assemble a 4x4 transform from a 3x3 rotation and a 3-vector translation."""
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if p is not None:
        T[:3, 3] = np.asarray(p, dtype=float).reshape(3)
    return T


def translation(p) -> np.ndarray:
    return homogeneous(p=p)


def transform_inverse(T: np.ndarray) -> np.ndarray:
    """Inverse of a homogeneous transform (R^T, -R^T p)."""
    R = T[:3, :3]
    p = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ p
    return Ti


def rotation_matrix_to_axis_angle(R: np.ndarray) -> Tuple[np.ndarray, float]:
    """3x3 rotation -> (unit axis, angle in [0, pi]).

    Numerically robust around the 0 and pi singularities. Returns the x-axis
    with angle 0 when ``R`` is (near) identity.
    """
    R = np.asarray(R, dtype=float)
    cos_theta = (np.trace(R) - 1.0) / 2.0
    cos_theta = float(np.clip(cos_theta, -1.0, 1.0))
    theta = np.arccos(cos_theta)

    if theta < 1e-8:
        return np.array([1.0, 0.0, 0.0]), 0.0

    if np.pi - theta < 1e-6:
        # Near 180 deg: sin(theta) ~ 0, recover axis from the diagonal of R.
        # axis_i = sqrt((R_ii + 1) / 2), with signs fixed from the off-diagonals.
        diag = np.clip((np.diag(R) + 1.0) / 2.0, 0.0, None)
        axis = np.sqrt(diag)
        # Largest component is most reliable; fix remaining signs relative to it.
        k = int(np.argmax(axis))
        if k == 0:
            axis[1] = np.copysign(axis[1], R[0, 1])
            axis[2] = np.copysign(axis[2], R[0, 2])
        elif k == 1:
            axis[0] = np.copysign(axis[0], R[0, 1])
            axis[2] = np.copysign(axis[2], R[1, 2])
        else:
            axis[0] = np.copysign(axis[0], R[0, 2])
            axis[1] = np.copysign(axis[1], R[1, 2])
        axis = axis / np.linalg.norm(axis)
        return axis, theta

    axis = np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]], dtype=float
    ) / (2.0 * np.sin(theta))
    return axis, theta


def rotation_error(R_target: np.ndarray, R_current: np.ndarray) -> np.ndarray:
    """Orientation error as a rotation vector: axis * angle of R_target R_current^T.

    This is the angular part of the IK error; its norm is the geodesic rotation
    angle between the two orientations.
    """
    R_err = R_target @ R_current.T
    axis, angle = rotation_matrix_to_axis_angle(R_err)
    return axis * angle


# ---------------------------------------------------------------------------
# Per-joint local transform
# ---------------------------------------------------------------------------


def joint_transform(joint, q: float) -> np.ndarray:
    """Local transform across ``joint`` at value ``q``: T_origin @ T_motion."""
    T_origin = homogeneous(R=rpy_to_matrix(joint.origin_rpy), p=joint.origin_xyz)

    if getattr(joint, "locked", False):
        return T_origin  # held at zero: origin transform only

    if joint.type in ("revolute", "continuous"):
        T_motion = homogeneous(R=axis_angle_to_matrix(joint.axis, q))
    elif joint.type == "prismatic":
        T_motion = translation(np.asarray(joint.axis, dtype=float) * q)
    else:  # fixed
        T_motion = np.eye(4)

    return T_origin @ T_motion


# ---------------------------------------------------------------------------
# Forward kinematics
# ---------------------------------------------------------------------------


class RobotKinematics:
    """Forward/inverse kinematics + Jacobian bound to a :class:`KinematicChain`."""

    def __init__(self, chain: Optional[KinematicChain] = None):
        self.chain = chain if chain is not None else load_default_chain()
        self.actuated = self.chain.actuated_joints
        self.n = len(self.actuated)
        self.lower, self.upper = self.chain.joint_limits()

    # -- forward kinematics ------------------------------------------------

    def _q_for_joint_index(self, q: np.ndarray) -> Dict[int, int]:
        """Map index-in-``chain.joints`` -> index-in-``q`` for actuated joints."""
        mapping = {}
        qi = 0
        for idx, j in enumerate(self.chain.joints):
            if j.is_actuated:
                mapping[idx] = qi
                qi += 1
        return mapping

    def link_frames(self, q) -> Dict[str, np.ndarray]:
        """Return {link_name: 4x4 base-frame pose} for every link on the chain.

        Includes the base link (identity) and the pose of each child link after
        applying every joint up to and including it.
        """
        q = np.asarray(q, dtype=float).reshape(-1)
        frames: Dict[str, np.ndarray] = {self.chain.base_link: np.eye(4)}
        T = np.eye(4)
        qi = 0
        for j in self.chain.joints:
            if j.is_actuated:
                val = q[qi] if qi < len(q) else 0.0
                qi += 1
            else:
                val = 0.0
            T = T @ joint_transform(j, val)
            frames[j.child] = T.copy()
        return frames

    def forward_kinematics(self, q) -> np.ndarray:
        """Return the 4x4 base->tip end-effector pose for joint vector ``q``."""
        return self.link_frames(q)[self.chain.tip_link]

    # -- geometric Jacobian ------------------------------------------------

    def geometric_jacobian(self, q) -> np.ndarray:
        """6xN geometric Jacobian at ``q`` (linear rows on top, angular below).

        For revolute joint *i* with base-frame axis ``z_i`` at position ``p_i``::

            Jv_i = z_i x (p_ee - p_i),   Jw_i = z_i

        For a prismatic joint::

            Jv_i = z_i,                  Jw_i = 0
        """
        q = np.asarray(q, dtype=float).reshape(-1)
        frames = self.link_frames(q)
        p_ee = frames[self.chain.tip_link][:3, 3]

        J = np.zeros((6, self.n))
        col = 0
        for j in self.chain.joints:
            # Only actuated (non-locked) revolute/prismatic joints get a column;
            # fixed and locked joints contribute none.
            if not j.is_actuated:
                continue

            # The joint frame is expressed in its parent link's frame, so apply
            # the parent pose and the joint origin to the local axis/position.
            T_parent = frames[j.parent]
            R_joint = T_parent[:3, :3] @ rpy_to_matrix(j.origin_rpy)
            axis_world = R_joint @ np.asarray(j.axis, dtype=float)
            # Origin of the joint frame (point the axis passes through), base frame.
            p_joint = (
                T_parent[:3, :3] @ np.asarray(j.origin_xyz, dtype=float)
                + T_parent[:3, 3]
            )

            if j.type in ("revolute", "continuous"):
                J[:3, col] = np.cross(axis_world, p_ee - p_joint)
                J[3:, col] = axis_world
            else:  # prismatic
                J[:3, col] = axis_world
                J[3:, col] = 0.0
            col += 1
        return J

    def manipulability(self, q) -> float:
        """Yoshikawa manipulability ``sqrt(det(J J^T))`` (0 at a singularity)."""
        J = self.geometric_jacobian(q)
        JJt = J @ J.T
        det = np.linalg.det(JJt)
        return float(np.sqrt(max(det, 0.0)))

    # -- limit handling ----------------------------------------------------

    def clamp_to_limits(self, q) -> np.ndarray:
        """Clamp each joint of ``q`` to its URDF [lower, upper] limit."""
        q = np.asarray(q, dtype=float).reshape(-1)
        return np.minimum(np.maximum(q, self.lower), self.upper)

    def within_limits(self, q, tol: float = 1e-9) -> bool:
        q = np.asarray(q, dtype=float).reshape(-1)
        return bool(
            np.all(q >= self.lower - tol) and np.all(q <= self.upper + tol)
        )

    def random_valid_q(self, rng: Optional[np.random.Generator] = None) -> np.ndarray:
        """Uniform-random joint vector inside the limits (for tests / sampling)."""
        rng = rng if rng is not None else np.random.default_rng()
        return rng.uniform(self.lower, self.upper)

    # -- inverse kinematics ------------------------------------------------

    def pose_error(self, target_pose: np.ndarray, q) -> np.ndarray:
        """6-vector spatial error [p_target - p_cur ; axis_angle(R_t R_c^T)]."""
        T_cur = self.forward_kinematics(q)
        p_err = target_pose[:3, 3] - T_cur[:3, 3]
        w_err = rotation_error(target_pose[:3, :3], T_cur[:3, :3])
        return np.concatenate([p_err, w_err])

    def _ik_solve_single(
        self,
        target_pose: np.ndarray,
        q_init: np.ndarray,
        max_iters: int,
        tol: float,
        base_lambda: float,
        position_only: bool,
    ) -> Tuple[np.ndarray, bool, int]:
        """One damped-least-squares descent from a single seed ``q_init``.

        Iterates ``dq = J^T (J J^T + lambda^2 I)^{-1} e`` (the Levenberg-Marquardt
        / DLS update), applies it, clamps to joint limits, and stops when
        ``||e|| < tol``. The damping ``lambda`` is raised near singularities
        (low manipulability) for numerical stability at the cost of speed.
        """
        q = self.clamp_to_limits(np.asarray(q_init, dtype=float).reshape(-1).copy())

        rows = slice(0, 3) if position_only else slice(0, 6)
        m = 3 if position_only else 6

        iterations = 0
        for it in range(1, max_iters + 1):
            iterations = it
            e = self.pose_error(target_pose, q)[rows]
            if np.linalg.norm(e) < tol:
                return q, True, it

            J = self.geometric_jacobian(q)[rows, :]

            # Raise damping when manipulability collapses (J near rank-deficient).
            JJt = J @ J.T
            w = np.sqrt(max(np.linalg.det(JJt), 0.0))
            lam = base_lambda
            w0 = 1e-3
            if w < w0:
                lam = base_lambda * (1.0 + (w0 - w) / w0)

            dq = J.T @ np.linalg.solve(JJt + (lam ** 2) * np.eye(m), e)

            # Step-size guard: cap very large updates near singularities.
            max_step = 0.5
            nrm = np.linalg.norm(dq)
            if nrm > max_step:
                dq = dq * (max_step / nrm)

            q = self.clamp_to_limits(q + dq)

        e_final = self.pose_error(target_pose, q)[rows]
        success = bool(np.linalg.norm(e_final) < tol)
        return q, success, iterations

    def inverse_kinematics(
        self,
        target_pose: np.ndarray,
        q_init=None,
        max_iters: int = 200,
        tol: float = 1e-4,
        base_lambda: float = 0.05,
        position_only: bool = False,
        num_restarts: int = 20,
        seed: Optional[int] = 0,
    ) -> Tuple[np.ndarray, bool, int]:
        """Damped least-squares inverse kinematics with random restarts.

        First descends from ``q_init`` (or the joint mid-range). DLS is a local
        method, so if that seed lands in a local minimum we retry from up to
        ``num_restarts`` random valid configurations and keep the first solution
        that meets ``tol`` (or, failing that, the closest one). This is the
        standard remedy for the wrist/elbow singularities of a 6R arm.

        Parameters
        ----------
        target_pose : (4,4) homogeneous goal pose in the base frame.
        q_init      : initial guess (defaults to the joint mid-range).
        position_only : if True, solve only the 3 positional DOF.
        num_restarts : extra random seeds to try if the first descent fails.
        seed         : RNG seed for the restarts (set None for nondeterminism).

        Returns
        -------
        (q, success, iterations) — ``iterations`` is the total across attempts.
        """
        rows = slice(0, 3) if position_only else slice(0, 6)
        rng = np.random.default_rng(seed)

        if q_init is None:
            first_seed = 0.5 * (self.lower + self.upper)
        else:
            first_seed = np.asarray(q_init, dtype=float).reshape(-1).copy()

        best_q = self.clamp_to_limits(first_seed)
        best_err = np.inf
        total_iters = 0

        seeds = [first_seed]
        for _ in range(max(num_restarts, 0)):
            seeds.append(self.random_valid_q(rng))

        for s in seeds:
            q, success, iters = self._ik_solve_single(
                target_pose, s, max_iters, tol, base_lambda, position_only
            )
            total_iters += iters
            err = float(np.linalg.norm(self.pose_error(target_pose, q)[rows]))
            if err < best_err:
                best_err = err
                best_q = q
            if success:
                return best_q, True, total_iters

        return best_q, bool(best_err < tol), total_iters


# ---------------------------------------------------------------------------
# Module-level convenience wrappers (use the default URDF chain)
# ---------------------------------------------------------------------------

_DEFAULT: Optional[RobotKinematics] = None


def default_kinematics() -> RobotKinematics:
    """Lazily-built :class:`RobotKinematics` for the repo's default URDF."""
    global _DEFAULT
    if _DEFAULT is None:
        _DEFAULT = RobotKinematics()
    return _DEFAULT


def forward_kinematics(q, chain: Optional[KinematicChain] = None) -> np.ndarray:
    rk = RobotKinematics(chain) if chain is not None else default_kinematics()
    return rk.forward_kinematics(q)


def link_frames(q, chain: Optional[KinematicChain] = None) -> Dict[str, np.ndarray]:
    rk = RobotKinematics(chain) if chain is not None else default_kinematics()
    return rk.link_frames(q)


def geometric_jacobian(q, chain: Optional[KinematicChain] = None) -> np.ndarray:
    rk = RobotKinematics(chain) if chain is not None else default_kinematics()
    return rk.geometric_jacobian(q)


def inverse_kinematics(
    target_pose, q_init=None, max_iters: int = 200, tol: float = 1e-4, **kw
):
    rk = default_kinematics()
    return rk.inverse_kinematics(
        target_pose, q_init=q_init, max_iters=max_iters, tol=tol, **kw
    )


def manipulability(q, chain: Optional[KinematicChain] = None) -> float:
    rk = RobotKinematics(chain) if chain is not None else default_kinematics()
    return rk.manipulability(q)
