"""RRT-Connect collision-aware motion-planning tests.

These run alongside the kinematics suite with numpy/pytest only (the planner,
like the rest of ``robot_arm/``, has no ROS dependency).

Scenario
--------
The arm reaches out front; the start config tilts the base to +y and the goal
to -y. A thin wall sits in the ``y = 0`` plane at the outer reach: the straight
joint-space interpolation sweeps the arm through it, so a successful plan must
route *around* it. Both endpoints are collision-free by construction.
"""

import numpy as np
import pytest

from robot_arm import (
    Box,
    Sphere,
    default_kinematics,
    in_collision,
    path_length,
    plan_to_config,
    rrt_connect,
    shortcut_path,
)


# ---------------------------------------------------------------------------
# Shared fixtures / helpers
# ---------------------------------------------------------------------------

# Start tilts the arm to +y, goal to -y (same shape, base joint mirrored).
Q_START = np.array([0.9, 0.95, -0.6, 0.0, 0.7, 0.0])
Q_GOAL = np.array([-0.9, 0.95, -0.6, 0.0, 0.7, 0.0])

# Thin wall in the y=0 plane at the outer x reach: blocks the mid-swing of the
# straight interpolation but clears the arm at both endpoints.
BLOCKING_WALL = Box(center=[0.30, 0.0, 0.46], size=[0.34, 0.06, 0.50])


@pytest.fixture(scope="module")
def rk():
    return default_kinematics()


@pytest.fixture(scope="module")
def limits(rk):
    return rk.chain.joint_limits()


def _densely_collision_free(path, obstacles, spacing=0.01):
    """True if every config on a fine resampling of ``path`` is collision-free.

    Resamples each edge so consecutive checked configs differ by at most
    ``spacing`` rad (joint-space Euclidean) — finer than the planner's own
    edge-check resolution, so this genuinely re-validates the result.
    """
    for a, b in zip(path[:-1], path[1:]):
        seg = float(np.linalg.norm(b - a))
        n = max(int(np.ceil(seg / spacing)), 2)
        for t in np.linspace(0.0, 1.0, n):
            if in_collision(a + t * (b - a), obstacles):
                return False
    return True


# ---------------------------------------------------------------------------
# Collision model
# ---------------------------------------------------------------------------


def test_sphere_distance_inside_and_outside():
    s = Sphere(center=[0.0, 0.0, 0.0], radius=0.5)
    d = s.distance(np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]))
    assert d[0] < 0  # center is inside
    assert np.isclose(d[1], 0.5)  # 1.0 away from a 0.5-radius surface


def test_box_distance_inside_is_zero_outside_positive():
    b = Box(center=[0.0, 0.0, 0.0], size=[1.0, 1.0, 1.0])
    d = b.distance(np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]))
    assert d[0] == 0.0  # inside
    assert np.isclose(d[1], 0.5)  # 0.5 outside the +x face


def test_endpoints_are_collision_free(rk):
    """The chosen start/goal must be free for the scenario to be well-posed."""
    assert not in_collision(Q_START, [BLOCKING_WALL], rk=rk)
    assert not in_collision(Q_GOAL, [BLOCKING_WALL], rk=rk)


def test_straight_interpolation_is_actually_blocked(rk):
    """Sanity: the naive straight joint path hits the wall (so planning matters)."""
    hit = any(
        in_collision(Q_START + t * (Q_GOAL - Q_START), [BLOCKING_WALL], rk=rk)
        for t in np.linspace(0.0, 1.0, 80)
    )
    assert hit


def test_no_collision_when_no_obstacles(rk):
    assert in_collision(Q_START, [], rk=rk) is False


# ---------------------------------------------------------------------------
# RRT-Connect: finds a valid path around an obstacle
# ---------------------------------------------------------------------------


def test_rrt_connect_finds_path_around_obstacle(rk, limits):
    path, info = rrt_connect(
        Q_START, Q_GOAL, [BLOCKING_WALL], limits,
        max_iters=5000, step=0.1, seed=1,
    )
    assert path is not None, f"planner failed: {info}"
    assert info["reason"] == "connected"

    # Endpoints equal start/goal exactly.
    assert np.allclose(path[0], Q_START)
    assert np.allclose(path[-1], Q_GOAL)

    # Every config respects the joint limits.
    lower, upper = limits
    for q in path:
        assert np.all(q >= lower - 1e-9)
        assert np.all(q <= upper + 1e-9)

    # The path is collision-free when densely resampled (finer than the
    # planner's own edge resolution).
    assert _densely_collision_free(path, [BLOCKING_WALL])


def test_rrt_connect_succeeds_across_several_seeds(rk, limits):
    """Robustness: independent seeds all return a valid, collision-free path."""
    for seed in range(6):
        path, info = rrt_connect(
            Q_START, Q_GOAL, [BLOCKING_WALL], limits,
            max_iters=5000, step=0.1, seed=seed,
        )
        assert path is not None, f"seed {seed} failed: {info}"
        assert np.allclose(path[0], Q_START)
        assert np.allclose(path[-1], Q_GOAL)
        assert _densely_collision_free(path, [BLOCKING_WALL])


# ---------------------------------------------------------------------------
# RRT-Connect: no false success when the goal is unreachable
# ---------------------------------------------------------------------------


def test_rrt_connect_returns_none_when_goal_enclosed(rk, limits):
    """A sphere enclosing the goal makes every valid path impossible -> None.

    Any path must terminate at the goal config; if that config is in collision,
    the planner must report failure rather than fabricate a path.
    """
    ee_goal = rk.forward_kinematics(Q_GOAL)[:3, 3]
    cage = Sphere(center=ee_goal, radius=0.30)
    assert in_collision(Q_GOAL, [cage], rk=rk)  # goal really is enclosed

    path, info = rrt_connect(
        Q_START, Q_GOAL, [cage], limits, max_iters=3000, seed=1,
    )
    assert path is None
    assert info["reason"] == "goal in collision"


def test_rrt_connect_no_false_success_on_tiny_budget(rk, limits):
    """With a 1-iteration budget the planner returns None, never a bad path."""
    path, info = rrt_connect(
        Q_START, Q_GOAL, [BLOCKING_WALL], limits, max_iters=1, seed=0,
    )
    # Either it cleanly ran out of iterations (the expected case), or, if it
    # somehow connected, the returned path must still be genuinely valid.
    if path is None:
        assert info["reason"] == "max_iters reached"
    else:  # pragma: no cover - guards against a false success
        assert _densely_collision_free(path, [BLOCKING_WALL])


# ---------------------------------------------------------------------------
# Shortcutting
# ---------------------------------------------------------------------------


def test_shortcut_path_is_shorter_and_still_collision_free(rk, limits):
    path, info = rrt_connect(
        Q_START, Q_GOAL, [BLOCKING_WALL], limits,
        max_iters=5000, step=0.1, seed=2,
    )
    assert path is not None, f"planner failed: {info}"

    short = shortcut_path(path, [BLOCKING_WALL], iters=200, seed=2)

    # Endpoints preserved.
    assert np.allclose(short[0], Q_START)
    assert np.allclose(short[-1], Q_GOAL)

    # Not longer in joint space (triangle inequality guarantees this).
    assert path_length(short) <= path_length(path) + 1e-9

    # Still collision-free at a fine resampling.
    assert _densely_collision_free(short, [BLOCKING_WALL])


def test_shortcut_path_handles_trivial_paths():
    """A 1- or 2-point path is returned unchanged (nothing to shorten)."""
    two = [Q_START.copy(), Q_GOAL.copy()]
    out = shortcut_path(two, [BLOCKING_WALL], iters=50, seed=0)
    assert len(out) == 2
    assert np.allclose(out[0], Q_START) and np.allclose(out[-1], Q_GOAL)


# ---------------------------------------------------------------------------
# Determinism
# ---------------------------------------------------------------------------


def test_rrt_connect_deterministic_with_fixed_seed(rk, limits):
    p1, _ = rrt_connect(Q_START, Q_GOAL, [BLOCKING_WALL], limits, seed=3)
    p2, _ = rrt_connect(Q_START, Q_GOAL, [BLOCKING_WALL], limits, seed=3)
    assert p1 is not None and p2 is not None
    assert len(p1) == len(p2)
    for a, b in zip(p1, p2):
        assert np.allclose(a, b)


def test_shortcut_deterministic_with_fixed_seed(rk, limits):
    path, _ = rrt_connect(Q_START, Q_GOAL, [BLOCKING_WALL], limits, seed=4)
    assert path is not None
    s1 = shortcut_path(path, [BLOCKING_WALL], seed=7)
    s2 = shortcut_path(path, [BLOCKING_WALL], seed=7)
    assert len(s1) == len(s2)
    for a, b in zip(s1, s2):
        assert np.allclose(a, b)


# ---------------------------------------------------------------------------
# High-level wrapper
# ---------------------------------------------------------------------------


def test_plan_to_config_wrapper_shortcuts(rk):
    path, info = plan_to_config(
        Q_START, Q_GOAL, [BLOCKING_WALL], seed=5,
    )
    assert path is not None
    assert info.get("shortcut") is True
    assert np.allclose(path[0], Q_START)
    assert np.allclose(path[-1], Q_GOAL)
    assert _densely_collision_free(path, [BLOCKING_WALL])
