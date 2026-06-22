"""Collision-aware RRT-Connect motion planning in joint space.

Pure-Python (numpy only). NO ROS / rclpy imports — this planner runs and is
unit-tested with numpy/pytest alone, like the rest of ``robot_arm/``.

Pieces
------
* :class:`Sphere` / :class:`Box`         — workspace obstacle primitives.
* :func:`link_sample_points`             — Cartesian sample points along the
  arm (capsule centerlines between consecutive link frames, plus interpolated
  points so a thin obstacle between two joints is still caught).
* :func:`in_collision`                   — True if any link sample lies inside
  any obstacle inflated by the link radius + a safety margin.
* :func:`rrt_connect`                    — bidirectional RRT-Connect: grow two
  trees (from start and goal), greedily ``connect`` one toward the other's new
  node, swap, repeat until they meet. Edges are collision-checked at a fine
  resolution; joint limits are respected by construction.
* :func:`shortcut_path`                  — random-pair straight-line shortcutting
  that never increases joint-space path length and stays collision-free.
* :func:`plan_to_config` / :func:`plan_to_pose` — convenience entry points
  (the latter runs IK to turn a Cartesian goal into a joint goal first).

Conventions
-----------
* ``q`` is the actuated-joint vector (the 6 revolute joints for the default
  arm), ordered as :attr:`RobotKinematics.actuated`.
* ``obstacles`` is a list of :class:`Sphere` / :class:`Box`.
* ``joint_limits`` is ``(lower, upper)`` arrays aligned with ``q`` (e.g.
  ``RobotKinematics().chain.joint_limits()``).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple, Union

import numpy as np

from .kinematics import RobotKinematics, default_kinematics

# ---------------------------------------------------------------------------
# Obstacle primitives
# ---------------------------------------------------------------------------


@dataclass
class Sphere:
    """A spherical obstacle in the base frame."""

    center: np.ndarray
    radius: float

    def __post_init__(self) -> None:
        self.center = np.asarray(self.center, dtype=float).reshape(3)
        self.radius = float(self.radius)

    def distance(self, points: np.ndarray) -> np.ndarray:
        """Signed-ish distance from each point to the surface (<=0 inside)."""
        pts = np.atleast_2d(np.asarray(points, dtype=float))
        return np.linalg.norm(pts - self.center, axis=1) - self.radius


@dataclass
class Box:
    """An axis-aligned box obstacle, given by its center and full ``size``."""

    center: np.ndarray
    size: np.ndarray  # full extents (width, depth, height)

    def __post_init__(self) -> None:
        self.center = np.asarray(self.center, dtype=float).reshape(3)
        self.size = np.asarray(self.size, dtype=float).reshape(3)
        self._half = self.size / 2.0

    @property
    def lower(self) -> np.ndarray:
        return self.center - self._half

    @property
    def upper(self) -> np.ndarray:
        return self.center + self._half

    def distance(self, points: np.ndarray) -> np.ndarray:
        """Exact distance from each point to the box (0 inside).

        Standard AABB distance: distance to the clamped (nearest-surface) point.
        Points strictly inside return 0, which combined with the inflation
        radius in :func:`in_collision` still flags them as colliding.
        """
        pts = np.atleast_2d(np.asarray(points, dtype=float))
        delta = np.maximum(np.maximum(self.lower - pts, pts - self.upper), 0.0)
        return np.linalg.norm(delta, axis=1)


Obstacle = Union[Sphere, Box]


# ---------------------------------------------------------------------------
# Arm collision model
# ---------------------------------------------------------------------------

# Effective radius of the arm links (the URDF collision boxes are ~0.1 m on a
# side, so ~0.06 m is a sensible capsule radius), and a default safety margin.
LINK_RADIUS = 0.06
SAFETY_MARGIN = 0.02


def link_sample_points(
    q: np.ndarray,
    rk: Optional[RobotKinematics] = None,
    per_segment: int = 4,
) -> np.ndarray:
    """Cartesian points approximating the arm's body at configuration ``q``.

    Takes the origin of every link frame (via :meth:`RobotKinematics.link_frames`)
    and additionally interpolates ``per_segment`` points along each capsule
    between consecutive link origins, so a thin obstacle lodged between two
    joints is still detected. Returns an ``(M, 3)`` array of base-frame points.
    """
    rk = rk if rk is not None else default_kinematics()
    frames = rk.link_frames(q)

    # Origins in chain order: base, then each joint child.
    ordered_links: List[str] = [rk.chain.base_link]
    for j in rk.chain.joints:
        ordered_links.append(j.child)

    origins = np.array([frames[name][:3, 3] for name in ordered_links])

    pts: List[np.ndarray] = [origins]
    if per_segment > 0 and len(origins) >= 2:
        # Interpolate strictly-interior points (endpoints already in origins).
        ts = np.linspace(0.0, 1.0, per_segment + 2)[1:-1]
        a = origins[:-1]
        b = origins[1:]
        for t in ts:
            pts.append(a + t * (b - a))
    return np.vstack(pts)


def in_collision(
    q: np.ndarray,
    obstacles: Sequence[Obstacle],
    rk: Optional[RobotKinematics] = None,
    link_radius: float = LINK_RADIUS,
    margin: float = SAFETY_MARGIN,
    per_segment: int = 4,
) -> bool:
    """True if the arm at ``q`` intersects any obstacle (with a safety margin).

    Each arm link is approximated by sample points (see
    :func:`link_sample_points`); a point collides with an obstacle when its
    distance to that obstacle is below ``link_radius + margin``. Self-collision
    is not modeled for this arm (the boxes never overlap within the joint
    limits in practice); the focus is environment collisions.
    """
    if not obstacles:
        return False
    pts = link_sample_points(q, rk=rk, per_segment=per_segment)
    thresh = link_radius + margin
    for obs in obstacles:
        if np.any(obs.distance(pts) <= thresh):
            return True
    return False


def _edge_collision_free(
    q_a: np.ndarray,
    q_b: np.ndarray,
    obstacles: Sequence[Obstacle],
    rk: RobotKinematics,
    resolution: float,
    link_radius: float,
    margin: float,
    per_segment: int,
) -> bool:
    """Check the straight joint-space segment ``q_a -> q_b`` at fine resolution.

    Subdivides the edge so consecutive checked configs differ by at most
    ``resolution`` (max-norm in joint space), then tests every interior sample
    (endpoints are assumed already validated by the caller).
    """
    diff = q_b - q_a
    dist = float(np.max(np.abs(diff)))
    n_steps = max(int(np.ceil(dist / resolution)), 1)
    # Check interior points; endpoints are validated by the tree/extend logic.
    for i in range(1, n_steps):
        q = q_a + (i / n_steps) * diff
        if in_collision(q, obstacles, rk=rk, link_radius=link_radius,
                        margin=margin, per_segment=per_segment):
            return False
    return True


# ---------------------------------------------------------------------------
# RRT-Connect
# ---------------------------------------------------------------------------


@dataclass
class _Tree:
    """A search tree: nodes plus a parent index per node."""

    nodes: List[np.ndarray] = field(default_factory=list)
    parents: List[int] = field(default_factory=list)

    def add(self, q: np.ndarray, parent: int) -> int:
        self.nodes.append(np.asarray(q, dtype=float).copy())
        self.parents.append(parent)
        return len(self.nodes) - 1

    def nearest(self, q: np.ndarray) -> int:
        """Index of the node nearest ``q`` (Euclidean in joint space)."""
        arr = np.asarray(self.nodes)
        d = np.linalg.norm(arr - q, axis=1)
        return int(np.argmin(d))

    def path_to_root(self, idx: int) -> List[np.ndarray]:
        """Node configs from ``idx`` back to the root (root last)."""
        out: List[np.ndarray] = []
        while idx != -1:
            out.append(self.nodes[idx])
            idx = self.parents[idx]
        return out


# Status codes for a single greedy extension toward a target.
_TRAPPED = 0   # immediate edge blocked, no progress
_ADVANCED = 1  # moved one or more steps but did not reach the target
_REACHED = 2   # arrived at (within step of) the target


def _extend(
    tree: _Tree,
    q_target: np.ndarray,
    obstacles: Sequence[Obstacle],
    rk: RobotKinematics,
    step: float,
    resolution: float,
    link_radius: float,
    margin: float,
    per_segment: int,
) -> Tuple[int, int]:
    """Grow ``tree`` one ``step`` from its nearest node toward ``q_target``.

    Returns ``(status, new_index)``. This is the single-step RRT ``EXTEND``.
    """
    near_idx = tree.nearest(q_target)
    q_near = tree.nodes[near_idx]
    diff = q_target - q_near
    dist = float(np.linalg.norm(diff))

    if dist <= step:
        q_new = q_target
        reached = True
    else:
        q_new = q_near + (step / dist) * diff
        reached = False

    # The new node and the edge to it must both be collision-free.
    if in_collision(q_new, obstacles, rk=rk, link_radius=link_radius,
                    margin=margin, per_segment=per_segment):
        return _TRAPPED, near_idx
    if not _edge_collision_free(q_near, q_new, obstacles, rk, resolution,
                                link_radius, margin, per_segment):
        return _TRAPPED, near_idx

    new_idx = tree.add(q_new, near_idx)
    return (_REACHED if reached else _ADVANCED), new_idx


def _connect(
    tree: _Tree,
    q_target: np.ndarray,
    obstacles: Sequence[Obstacle],
    rk: RobotKinematics,
    step: float,
    resolution: float,
    link_radius: float,
    margin: float,
    per_segment: int,
) -> Tuple[int, int]:
    """Repeatedly EXTEND ``tree`` toward ``q_target`` until blocked or reached.

    This is the greedy RRT-Connect ``CONNECT`` operator. Returns the final
    ``(status, last_index)``.
    """
    status = _ADVANCED
    last_idx = tree.nearest(q_target)
    # Bound the inner loop so an unreachable target can't spin forever.
    max_connect_steps = 10_000
    for _ in range(max_connect_steps):
        status, last_idx = _extend(
            tree, q_target, obstacles, rk, step, resolution,
            link_radius, margin, per_segment,
        )
        if status != _ADVANCED:
            break
    return status, last_idx


def rrt_connect(
    q_start: np.ndarray,
    q_goal: np.ndarray,
    obstacles: Sequence[Obstacle],
    joint_limits: Tuple[np.ndarray, np.ndarray],
    max_iters: int = 5000,
    step: float = 0.1,
    resolution: float = 0.02,
    rk: Optional[RobotKinematics] = None,
    link_radius: float = LINK_RADIUS,
    margin: float = SAFETY_MARGIN,
    per_segment: int = 4,
    seed: Optional[int] = 0,
) -> Tuple[Optional[List[np.ndarray]], Dict]:
    """Bidirectional RRT-Connect in joint space.

    Grows a tree from ``q_start`` and another from ``q_goal``. Each iteration:
    sample a random valid config, EXTEND the active tree one ``step`` toward it,
    then greedily CONNECT the other tree toward that new node. If the trees meet
    the two branches are stitched into a single start->goal path. Trees are
    swapped every iteration so growth alternates.

    Parameters
    ----------
    q_start, q_goal : start / goal joint vectors (must be collision-free and
        within ``joint_limits`` for a path to exist).
    obstacles       : list of :class:`Sphere` / :class:`Box`.
    joint_limits    : ``(lower, upper)`` arrays aligned with ``q``.
    max_iters       : maximum sample iterations before giving up.
    step            : max joint-space EXTEND distance (Euclidean) per step.
    resolution      : edge collision-check spacing (max-norm, joint space).
    seed            : RNG seed (set ``None`` for nondeterminism).

    Returns
    -------
    ``(path, info)``. ``path`` is a list of joint vectors from ``q_start`` to
    ``q_goal`` (inclusive) if one was found, else ``None``. ``info`` carries
    ``iterations``, ``tree_sizes``, ``reason`` and the configured ``seed``.
    """
    rk = rk if rk is not None else default_kinematics()
    lower, upper = joint_limits
    lower = np.asarray(lower, dtype=float).reshape(-1)
    upper = np.asarray(upper, dtype=float).reshape(-1)
    rng = np.random.default_rng(seed)

    q_start = np.asarray(q_start, dtype=float).reshape(-1).copy()
    q_goal = np.asarray(q_goal, dtype=float).reshape(-1).copy()

    info: Dict = {
        "iterations": 0,
        "tree_sizes": (1, 1),
        "reason": "",
        "seed": seed,
    }

    def _within(q: np.ndarray) -> bool:
        return bool(np.all(q >= lower - 1e-9) and np.all(q <= upper + 1e-9))

    # Endpoints must be valid for the problem to be well-posed.
    if not _within(q_start) or not _within(q_goal):
        info["reason"] = "endpoint outside joint limits"
        return None, info
    if in_collision(q_start, obstacles, rk=rk, link_radius=link_radius,
                    margin=margin, per_segment=per_segment):
        info["reason"] = "start in collision"
        return None, info
    if in_collision(q_goal, obstacles, rk=rk, link_radius=link_radius,
                    margin=margin, per_segment=per_segment):
        info["reason"] = "goal in collision"
        return None, info

    tree_a = _Tree()
    tree_a.add(q_start, -1)
    tree_b = _Tree()
    tree_b.add(q_goal, -1)
    # Track which tree is rooted at the start so we can orient the final path.
    a_is_start = True

    for it in range(1, max_iters + 1):
        info["iterations"] = it

        q_rand = rng.uniform(lower, upper)

        # EXTEND the active tree (tree_a) one step toward the sample.
        status, new_idx = _extend(
            tree_a, q_rand, obstacles, rk, step, resolution,
            link_radius, margin, per_segment,
        )
        if status != _TRAPPED:
            q_new = tree_a.nodes[new_idx]
            # CONNECT the other tree greedily toward the freshly added node.
            cstatus, b_idx = _connect(
                tree_b, q_new, obstacles, rk, step, resolution,
                link_radius, margin, per_segment,
            )
            if cstatus == _REACHED:
                # Trees meet: branch_a (root->q_new) + branch_b (q_new->root).
                branch_a = tree_a.path_to_root(new_idx)  # q_new ... root_a
                branch_b = tree_b.path_to_root(b_idx)     # q_meet ... root_b
                # Orient both so the result runs q_start -> q_goal.
                if a_is_start:
                    start_side = list(reversed(branch_a))  # root_a(start) -> q_new
                    goal_side = branch_b                   # q_meet -> root_b(goal)
                else:
                    start_side = list(reversed(branch_b))  # root_b(start) -> q_meet
                    goal_side = branch_a                   # q_new -> root_a(goal)
                # q_new and q_meet coincide (CONNECT reached it); drop the dup.
                path = start_side + goal_side[1:]
                info["tree_sizes"] = (len(tree_a.nodes), len(tree_b.nodes))
                info["reason"] = "connected"
                return path, info

        # Swap trees so growth alternates between start and goal.
        tree_a, tree_b = tree_b, tree_a
        a_is_start = not a_is_start

    info["tree_sizes"] = (len(tree_a.nodes), len(tree_b.nodes))
    info["reason"] = "max_iters reached"
    return None, info


# ---------------------------------------------------------------------------
# Path post-processing
# ---------------------------------------------------------------------------


def path_length(path: Sequence[np.ndarray]) -> float:
    """Total joint-space (Euclidean) length of a polyline path."""
    if path is None or len(path) < 2:
        return 0.0
    arr = np.asarray(path, dtype=float)
    return float(np.sum(np.linalg.norm(np.diff(arr, axis=0), axis=1)))


def shortcut_path(
    path: Sequence[np.ndarray],
    obstacles: Sequence[Obstacle],
    iters: int = 200,
    rk: Optional[RobotKinematics] = None,
    resolution: float = 0.02,
    link_radius: float = LINK_RADIUS,
    margin: float = SAFETY_MARGIN,
    per_segment: int = 4,
    seed: Optional[int] = 0,
) -> List[np.ndarray]:
    """Greedily shorten ``path`` by replacing detours with straight segments.

    Repeatedly picks a random pair of waypoints; if the straight joint-space
    segment between them is collision-free, the intermediate waypoints are
    removed. This never increases joint-space path length (each accepted edit
    replaces a polyline sub-path with the straight chord, which by the triangle
    inequality is no longer) and the result stays collision-free.
    """
    rk = rk if rk is not None else default_kinematics()
    if path is None or len(path) <= 2:
        return [np.asarray(p, dtype=float).copy() for p in (path or [])]

    rng = np.random.default_rng(seed)
    pts: List[np.ndarray] = [np.asarray(p, dtype=float).copy() for p in path]

    for _ in range(iters):
        if len(pts) <= 2:
            break
        i = int(rng.integers(0, len(pts) - 1))
        j = int(rng.integers(i + 1, len(pts)))
        if j - i < 2:
            continue  # adjacent: nothing to remove
        # The chord endpoints are existing (valid) waypoints; checking the
        # interior of the straight segment suffices.
        if _edge_collision_free(pts[i], pts[j], obstacles, rk, resolution,
                                link_radius, margin, per_segment):
            pts = pts[: i + 1] + pts[j:]

    return pts


# ---------------------------------------------------------------------------
# High-level entry points
# ---------------------------------------------------------------------------


def plan_to_config(
    q_start: np.ndarray,
    q_goal: np.ndarray,
    obstacles: Sequence[Obstacle],
    rk: Optional[RobotKinematics] = None,
    shortcut: bool = True,
    max_iters: int = 5000,
    step: float = 0.1,
    seed: Optional[int] = 0,
    **kwargs,
) -> Tuple[Optional[List[np.ndarray]], Dict]:
    """Plan a collision-free joint path from ``q_start`` to ``q_goal``.

    Thin wrapper around :func:`rrt_connect` that pulls joint limits from the
    kinematics chain and (optionally) runs :func:`shortcut_path` on success.
    """
    rk = rk if rk is not None else default_kinematics()
    limits = rk.chain.joint_limits()
    path, info = rrt_connect(
        q_start, q_goal, obstacles, limits,
        max_iters=max_iters, step=step, rk=rk, seed=seed, **kwargs,
    )
    if path is not None and shortcut:
        path = shortcut_path(path, obstacles, rk=rk, seed=seed)
        info["shortcut"] = True
    return path, info


def plan_to_pose(
    q_start: np.ndarray,
    target_pose: np.ndarray,
    obstacles: Sequence[Obstacle],
    rk: Optional[RobotKinematics] = None,
    ik_seed: Optional[int] = 0,
    shortcut: bool = True,
    max_iters: int = 5000,
    step: float = 0.1,
    seed: Optional[int] = 0,
    **kwargs,
) -> Tuple[Optional[List[np.ndarray]], Dict]:
    """IK the Cartesian ``target_pose`` to a joint goal, then plan to it.

    Returns ``(None, info)`` with ``reason='ik_failed'`` if IK does not
    converge or the IK solution itself is in collision.
    """
    rk = rk if rk is not None else default_kinematics()
    q_goal, ok, _iters = rk.inverse_kinematics(
        target_pose, q_init=q_start, seed=ik_seed
    )
    if not ok:
        return None, {"reason": "ik_failed", "iterations": 0}
    if in_collision(q_goal, obstacles, rk=rk):
        return None, {"reason": "ik goal in collision", "iterations": 0}
    return plan_to_config(
        q_start, q_goal, obstacles, rk=rk, shortcut=shortcut,
        max_iters=max_iters, step=step, seed=seed, **kwargs,
    )
