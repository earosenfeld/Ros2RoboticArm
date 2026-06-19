"""URDF parsing / kinematic-chain structure tests."""

import numpy as np

from robot_arm import load_default_chain, load_full_chain, parse_urdf
from robot_arm.urdf_chain import DEFAULT_URDF_PATH


EXPECTED_REVOLUTE = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def test_parses_six_revolute_joints():
    chain = load_default_chain()
    assert [j.name for j in chain.revolute_joints] == EXPECTED_REVOLUTE
    assert len(chain.revolute_joints) == 6


def test_default_chain_is_six_dof_arm():
    """Default chain reaches the end effector with the gripper locked -> 6 DOF."""
    chain = load_default_chain()
    assert chain.num_actuated == 6
    assert chain.actuated_joint_names == EXPECTED_REVOLUTE
    assert chain.tip_link == "end_effector"


def test_gripper_is_prismatic_and_separate():
    chain = load_default_chain()
    # The gripper joint is present in the full chain but not actuated by default.
    prismatics_full = load_full_chain().prismatic_joints
    assert len(prismatics_full) == 1
    assert prismatics_full[0].name == "gripper_joint"
    assert prismatics_full[0].type == "prismatic"
    # Locked out of the default arm chain's actuated set.
    assert "gripper_joint" not in chain.actuated_joint_names


def test_full_chain_has_seven_actuated():
    chain = load_full_chain()
    assert chain.num_actuated == 7
    assert chain.actuated_joint_names == EXPECTED_REVOLUTE + ["gripper_joint"]


def test_joint_axes_and_origins():
    chain = load_default_chain()
    by_name = {j.name: j for j in chain.joints}
    # Axes from the URDF: z, y, y, z, y, z.
    expected_axes = {
        "joint1": [0, 0, 1],
        "joint2": [0, 1, 0],
        "joint3": [0, 1, 0],
        "joint4": [0, 0, 1],
        "joint5": [0, 1, 0],
        "joint6": [0, 0, 1],
    }
    for name, ax in expected_axes.items():
        assert np.allclose(by_name[name].axis, ax)
    # First joint origin.
    assert np.allclose(by_name["joint1"].origin_xyz, [0, 0, 0.05])


def test_joint_limits_match_urdf():
    chain = load_default_chain()
    lo, hi = chain.joint_limits()
    assert lo.shape == (6,) and hi.shape == (6,)
    assert np.allclose(lo, -3.14)
    assert np.allclose(hi, 3.14)


def test_explicit_parse_matches_default():
    chain = parse_urdf(
        DEFAULT_URDF_PATH, base_link="base_link", tip_link="end_effector",
        lock_joints=["gripper_joint"],
    )
    assert chain.actuated_joint_names == EXPECTED_REVOLUTE
