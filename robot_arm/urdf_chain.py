"""Parse a URDF into an ordered kinematic chain.

Pure-Python (xml.etree + dataclasses + numpy). NO ROS / rclpy imports — this
module must import and run standalone under plain CPython so the kinematics can
be unit-tested without a ROS installation.
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

# Default URDF shipped with the repo (…/urdf/robot_arm.urdf).
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_URDF_PATH = os.path.normpath(
    os.path.join(_THIS_DIR, "..", "urdf", "robot_arm.urdf")
)


@dataclass
class Joint:
    """A single URDF joint as a node in the kinematic chain."""

    name: str
    type: str  # 'revolute' | 'prismatic' | 'fixed' | 'continuous'
    parent: str
    child: str
    origin_xyz: np.ndarray  # (3,) translation of the joint frame in the parent
    origin_rpy: np.ndarray  # (3,) fixed roll/pitch/yaw of the joint frame
    axis: np.ndarray  # (3,) unit motion axis (joint frame)
    lower: Optional[float] = None  # position limit (rad for revolute, m for prismatic)
    upper: Optional[float] = None
    locked: bool = False  # if True: present in chain (origin only) but not actuated

    @property
    def is_actuated(self) -> bool:
        return (not self.locked) and self.type in (
            "revolute",
            "prismatic",
            "continuous",
        )


@dataclass
class Link:
    """A URDF link with its inertial parameters (mass kept for completeness)."""

    name: str
    mass: float = 0.0


@dataclass
class KinematicChain:
    """Ordered base->tip chain plus a name->joint lookup.

    ``joints`` is the full ordered list from ``base_link`` to ``tip`` (including
    fixed joints). ``actuated_joints`` is the subset of revolute/prismatic
    joints, in order — this is what forward kinematics maps ``q`` onto.
    """

    base_link: str
    tip_link: str
    joints: List[Joint]
    links: Dict[str, Link] = field(default_factory=dict)

    @property
    def actuated_joints(self) -> List[Joint]:
        return [j for j in self.joints if j.is_actuated]

    @property
    def actuated_joint_names(self) -> List[str]:
        return [j.name for j in self.actuated_joints]

    @property
    def revolute_joints(self) -> List[Joint]:
        return [j for j in self.joints if j.type in ("revolute", "continuous")]

    @property
    def prismatic_joints(self) -> List[Joint]:
        return [j for j in self.joints if j.type == "prismatic"]

    @property
    def num_actuated(self) -> int:
        return len(self.actuated_joints)

    def joint_limits(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return (lower, upper) arrays aligned with ``actuated_joints``.

        Missing limits (e.g. continuous joints) default to +/- pi.
        """
        lo, hi = [], []
        for j in self.actuated_joints:
            lo.append(j.lower if j.lower is not None else -np.pi)
            hi.append(j.upper if j.upper is not None else np.pi)
        return np.asarray(lo, dtype=float), np.asarray(hi, dtype=float)


def _parse_vec3(text: Optional[str], default=(0.0, 0.0, 0.0)) -> np.ndarray:
    if text is None or text.strip() == "":
        return np.asarray(default, dtype=float)
    parts = [float(x) for x in text.replace(",", " ").split()]
    return np.asarray(parts, dtype=float)


def _parse_links(root: ET.Element) -> Dict[str, Link]:
    links: Dict[str, Link] = {}
    for le in root.findall("link"):
        name = le.attrib["name"]
        mass = 0.0
        inertial = le.find("inertial")
        if inertial is not None:
            me = inertial.find("mass")
            if me is not None and "value" in me.attrib:
                mass = float(me.attrib["value"])
        links[name] = Link(name=name, mass=mass)
    return links


def _parse_joints(root: ET.Element) -> Dict[str, Joint]:
    joints: Dict[str, Joint] = {}
    for je in root.findall("joint"):
        name = je.attrib["name"]
        jtype = je.attrib["type"]

        origin = je.find("origin")
        origin_xyz = _parse_vec3(origin.get("xyz") if origin is not None else None)
        origin_rpy = _parse_vec3(origin.get("rpy") if origin is not None else None)

        axis_el = je.find("axis")
        axis = _parse_vec3(
            axis_el.get("xyz") if axis_el is not None else None, default=(1.0, 0.0, 0.0)
        )
        # Normalize the axis (URDF allows non-unit vectors).
        norm = np.linalg.norm(axis)
        if norm > 0:
            axis = axis / norm

        lower = upper = None
        limit = je.find("limit")
        if limit is not None:
            if "lower" in limit.attrib:
                lower = float(limit.attrib["lower"])
            if "upper" in limit.attrib:
                upper = float(limit.attrib["upper"])

        parent = je.find("parent").attrib["link"]
        child = je.find("child").attrib["link"]

        joints[name] = Joint(
            name=name,
            type=jtype,
            parent=parent,
            child=child,
            origin_xyz=origin_xyz,
            origin_rpy=origin_rpy,
            axis=axis,
            lower=lower,
            upper=upper,
        )
    return joints


def parse_urdf(
    urdf_path: str = DEFAULT_URDF_PATH,
    base_link: str = "base_link",
    tip_link: str = "end_effector",
    lock_joints: Optional[List[str]] = None,
) -> KinematicChain:
    """Parse ``urdf_path`` into an ordered chain from ``base_link`` to ``tip_link``.

    Walks the parent->child joint graph from the tip back to the base, then
    reverses to produce a base-first ordering. Raises ``ValueError`` if no path
    connects the two links.

    ``lock_joints`` names joints to hold at zero: they stay in the chain (so
    their origin transform still contributes to FK and the tip is still reached)
    but are excluded from the actuated set / IK / Jacobian.
    """
    lock_set = set(lock_joints or [])
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    links = _parse_links(root)
    joints_by_name = _parse_joints(root)

    # Map child-link -> joint that produces it, to walk tip -> base.
    joint_by_child: Dict[str, Joint] = {j.child: j for j in joints_by_name.values()}

    ordered_rev: List[Joint] = []
    current = tip_link
    guard = 0
    max_steps = len(joints_by_name) + 2
    while current != base_link:
        if current not in joint_by_child:
            raise ValueError(
                f"No joint produces link '{current}'; cannot reach base "
                f"'{base_link}' from tip '{tip_link}'."
            )
        j = joint_by_child[current]
        ordered_rev.append(j)
        current = j.parent
        guard += 1
        if guard > max_steps:
            raise ValueError("Cycle detected while walking the kinematic chain.")

    ordered = list(reversed(ordered_rev))
    for j in ordered:
        if j.name in lock_set:
            j.locked = True
    return KinematicChain(
        base_link=base_link, tip_link=tip_link, joints=ordered, links=links
    )


def load_default_chain() -> KinematicChain:
    """Parse the repo's default URDF into the **6-DOF arm** chain.

    Reaches ``end_effector`` (so FK matches the physical tip) but holds the
    gripper prismatic at zero, leaving exactly the 6 revolute joints actuated —
    the natural arm-IK problem.
    """
    return parse_urdf(
        DEFAULT_URDF_PATH,
        base_link="base_link",
        tip_link="end_effector",
        lock_joints=["gripper_joint"],
    )


def load_full_chain() -> KinematicChain:
    """Parse the full chain with the gripper prismatic actuated (7 DOF)."""
    return parse_urdf(
        DEFAULT_URDF_PATH, base_link="base_link", tip_link="end_effector"
    )
