"""URDF robot descriptions: parse, forward kinematics and spawning into a `World`, with the same rules and numbers as the TypeScript `robot.ts`."""
from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from ._wire import JOINT_FIXED, JOINT_HINGE, JOINT_SLIDER, MAX_ANGLE_LIMIT, MAX_LINEAR_LIMIT, SHAPE_BOX, SHAPE_CAPSULE, SHAPE_SPHERE
from .world import Handle, JointHandle, World

__all__ = ["RobotError", "parse_urdf", "forward_kinematics", "spawn_robot", "RobotDesc", "SpawnedRobot"]

V3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]
Frame = Tuple[V3, Quat]
IDENTITY: Frame = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
MAX_LINKS = 512
MAX_XML_CHARS, MAX_XML_DEPTH, MAX_XML_ELEMENTS = 16 << 20, 64, 200_000
MIN_HALF, MAX_HALF, MIN_MASS, MAX_MASS, ANCHOR_RADIUS = 1e-3, 1e4, 1e-6, 1e30, 0.002
_NUMBER = re.compile(r"^[+-]?([0-9]+\.?[0-9]*|\.[0-9]+)([eE][+-]?[0-9]+)?$")


class RobotError(ValueError):
    pass


@dataclass
class Shape:
    kind: str
    half: V3
    radius: float
    half_length: float
    frame: Frame


@dataclass
class Link:
    name: str
    mass: Optional[float]
    shape: Optional[Shape]
    static: bool = False


@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    origin: Frame
    axis: V3
    limit: Optional[Tuple[float, float]]


@dataclass
class RobotDesc:
    name: str
    root: str
    links: List[Link]
    joints: List[Joint]
    order: List[Joint]
    warnings: List[str] = field(default_factory=list)


@dataclass
class SpawnedRobot:
    desc: RobotDesc
    links: Dict[str, Handle]
    joints: Dict[str, JointHandle]
    anchors: List[Handle]
    frames: Dict[str, Frame]
    warnings: List[str]


# ---- maths ----
def qmul(a: Quat, b: Quat) -> Quat:
    return (
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    )


def qrot(q: Quat, v: Sequence[float]) -> V3:
    x, y, z, w = q
    t = (2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0]))
    return (v[0] + w * t[0] + (y * t[2] - z * t[1]), v[1] + w * t[1] + (z * t[0] - x * t[2]), v[2] + w * t[2] + (x * t[1] - y * t[0]))


def compose(a: Frame, b: Frame) -> Frame:
    r = qrot(a[1], b[0])
    return ((a[0][0] + r[0], a[0][1] + r[1], a[0][2] + r[2]), qmul(a[1], b[1]))


def axis_angle(axis: Sequence[float], angle: float) -> Quat:
    s = math.sin(angle / 2) / math.hypot(*axis)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2))


def rpy_to_quat(r: float, p: float, y: float) -> Quat:
    return qmul(qmul(axis_angle((0, 0, 1), y), axis_angle((0, 1, 0), p)), axis_angle((1, 0, 0), r))


# ---- parsing ----
def _numbers(text: Optional[str], count: int, what: str, fallback: Optional[Sequence[float]] = None) -> List[float]:
    if text is None:
        if fallback is not None:
            return list(fallback)
        raise RobotError(f"{what} is missing")
    parts = text.split()
    values = [float(s) if _NUMBER.match(s) else math.nan for s in parts]
    if len(parts) != count or any(not math.isfinite(v) for v in values):
        raise RobotError(f"{what} needs {count} finite numbers, got \"{text}\"")
    return values


def _number(text: Optional[str], what: str, positive: bool = False) -> float:
    (v,) = _numbers(text, 1, what)
    if positive and not v > 0:
        raise RobotError(f"{what} must be positive, got {v:g}")
    return v


def _frame(el: Optional[ET.Element], what: str) -> Frame:
    if el is None:
        return IDENTITY
    xyz = _numbers(el.get("xyz"), 3, f"{what} xyz", [0, 0, 0])
    rpy = _numbers(el.get("rpy"), 3, f"{what} rpy", [0, 0, 0])
    return ((xyz[0], xyz[1], xyz[2]), rpy_to_quat(*rpy))


def _shape(link: str, collision: ET.Element, warnings: List[str]) -> Optional[Shape]:
    frame = _frame(collision.find("origin"), f"link {link} collision origin")
    geometry = collision.find("geometry")
    g = geometry[0] if geometry is not None and len(geometry) else None
    if g is None:
        raise RobotError(f"link {link} has a collision with no geometry")
    zero = (0.0, 0.0, 0.0)
    if g.tag == "box":
        size = _numbers(g.get("size"), 3, f"link {link} box size")
        if any(not s > 0 for s in size):
            raise RobotError(f"link {link} box size must be positive")
        return Shape("box", (size[0] / 2, size[1] / 2, size[2] / 2), 0.0, 0.0, frame)
    if g.tag == "sphere":
        return Shape("sphere", zero, _number(g.get("radius"), f"link {link} sphere radius", True), 0.0, frame)
    if g.tag in ("cylinder", "capsule"):
        radius = _number(g.get("radius"), f"link {link} {g.tag} radius", True)
        length = _number(g.get("length"), f"link {link} {g.tag} length", True)
        if g.tag == "cylinder":
            warnings.append(f"link {link}: the cylinder is a capsule with rounded ends")
        return Shape("capsule", zero, radius, max(length / 2 - radius, 0.0), compose(frame, ((0.0, 0.0, 0.0), axis_angle((1, 0, 0), math.pi / 2))))
    warnings.append(f"link {link}: a {g.tag} collision is not supported and was dropped")
    return None


def _check_size(root: ET.Element) -> None:
    count, stack = 0, [(root, 1)]
    while stack:
        el, depth = stack.pop()
        count += 1
        if depth > MAX_XML_DEPTH:
            raise RobotError(f"nesting deeper than {MAX_XML_DEPTH}")
        if count > MAX_XML_ELEMENTS:
            raise RobotError(f"more than {MAX_XML_ELEMENTS} elements")
        stack.extend((c, depth + 1) for c in el)


def parse_urdf(text: str) -> RobotDesc:
    """Differences from the TypeScript reader: XML namespaces are refused, a declared encoding is honoured, and warning texts format numbers in Python's way."""
    if not isinstance(text, str):
        raise RobotError("a URDF is given as text (str)")
    if len(text) > MAX_XML_CHARS:
        raise RobotError(f"document is larger than {MAX_XML_CHARS} characters")
    if re.search(r"<!DOCTYPE|<!ENTITY", re.sub(r"<!--.*?-->|<!\[CDATA\[.*?\]\]>", "", text, flags=re.S), re.I):
        raise RobotError("DOCTYPE and ENTITY declarations are not supported")
    try:
        robot = ET.fromstring(text)
    except (ET.ParseError, ValueError, UnicodeError) as err:
        raise RobotError(f"not well-formed XML: {err}") from err
    _check_size(robot)
    if robot.tag != "robot":
        raise RobotError(f"the root element is <{robot.tag}>, not <robot>")
    warnings: List[str] = []
    links: List[Link] = []
    seen = set()
    for l in robot.findall("link"):
        name = l.get("name")
        if not name:
            raise RobotError("a link has no name")
        if name in seen:
            raise RobotError(f"link {name} is defined twice")
        seen.add(name)
        if len(links) >= MAX_LINKS:
            raise RobotError(f"more than {MAX_LINKS} links")
        inertial = l.find("inertial")
        mass_el = inertial.find("mass") if inertial is not None else None
        mass = _number(mass_el.get("value"), f"link {name} mass", True) if mass_el is not None else None
        shapes = [s for s in (_shape(name, c, warnings) for c in l.findall("collision")) if s is not None]
        if len(shapes) > 1:
            warnings.append(f"link {name}: only the first of {len(shapes)} collision shapes is used (a body has one shape)")
        links.append(Link(name, mass, shapes[0] if shapes else None))
    if not links:
        raise RobotError("the robot has no links")
    joints: List[Joint] = []
    names = set()
    for j in robot.findall("joint"):
        name = j.get("name")
        if not name:
            raise RobotError("a joint has no name")
        if name in names:
            raise RobotError(f"joint {name} is defined twice")
        names.add(name)
        kind = j.get("type")
        if kind not in ("revolute", "continuous", "prismatic", "fixed"):
            raise RobotError(f"joint {name} has type \"{kind}\", which is not supported (revolute, continuous, prismatic and fixed are)")
        parent = j.find("parent").get("link") if j.find("parent") is not None else None
        kid = j.find("child").get("link") if j.find("child") is not None else None
        if parent not in seen:
            raise RobotError(f"joint {name} has an unknown parent link \"{parent}\"")
        if kid not in seen:
            raise RobotError(f"joint {name} has an unknown child link \"{kid}\"")
        axis_el = j.find("axis")
        axis = [1.0, 0.0, 0.0] if kind == "fixed" else _numbers(axis_el.get("xyz") if axis_el is not None else None, 3, f"joint {name} axis", [1, 0, 0])
        length = math.hypot(*axis)
        if not length > 1e-6:
            raise RobotError(f"joint {name} has a zero axis")
        limit = None
        lim = j.find("limit")
        if kind in ("revolute", "prismatic"):
            if lim is None:
                raise RobotError(f"joint {name} is {kind} and needs a <limit>")
            lower, upper = _number(lim.get("lower", "0"), f"joint {name} lower"), _number(lim.get("upper", "0"), f"joint {name} upper")
            if lower > upper:
                raise RobotError(f"joint {name} has lower {lower:g} above upper {upper:g}")
            limit = (lower, upper)
        joints.append(Joint(name, kind, parent, kid, _frame(j.find("origin"), f"joint {name} origin"), (axis[0] / length, axis[1] / length, axis[2] / length), limit))
    parent_of: Dict[str, Joint] = {}
    for j in joints:
        if j.child in parent_of:
            raise RobotError(f"link {j.child} has two parent joints ({parent_of[j.child].name} and {j.name}); a loop is not a tree")
        parent_of[j.child] = j
    roots = [l for l in links if l.name not in parent_of]
    if len(roots) != 1:
        raise RobotError("every link has a parent: the joints form a loop" if not roots else f"the links {', '.join(r.name for r in roots)} have no parent; a robot has one root")
    order: List[Joint] = []
    placed = {roots[0].name}
    progress = True
    while progress and len(order) < len(joints):
        progress = False
        for j in joints:
            if j.child not in placed and j.parent in placed:
                order.append(j)
                placed.add(j.child)
                progress = True
    if len(order) != len(joints):
        raise RobotError("some links are not connected to the root")
    return RobotDesc(robot.get("name", "robot"), roots[0].name, links, joints, order, warnings)


def forward_kinematics(desc: RobotDesc, values: Optional[Dict[str, float]] = None) -> Dict[str, Frame]:
    values = values or {}
    frames: Dict[str, Frame] = {desc.root: IDENTITY}
    for j in desc.order:
        v = values.get(j.name, 0.0)
        if not math.isfinite(v):
            raise RobotError(f"value of joint {j.name} is not finite")
        if j.type == "prismatic":
            motion: Frame = ((j.axis[0] * v, j.axis[1] * v, j.axis[2] * v), (0.0, 0.0, 0.0, 1.0))
        elif j.type == "fixed":
            motion = IDENTITY
        else:
            motion = ((0.0, 0.0, 0.0), axis_angle(j.axis, v))
        frames[j.child] = compose(compose(frames[j.parent], j.origin), motion)
    return frames


def spawn_robot(world: World, desc: RobotDesc, position: Sequence[float] = (0, 0, 0), rotation: Quat = (0, 0, 0, 1), *, fixed_base: bool = False,
                default_mass: float = 1.0, friction: Optional[float] = None, up_axis: str = "y") -> SpawnedRobot:
    """Puts the robot into `world` at zero joint values, with the rules of the TypeScript `spawnRobot` (see docs/robots.md)."""
    if up_axis not in ("y", "z"):
        raise ValueError("up_axis is 'y' or 'z'")
    if not math.isfinite(default_mass) or (friction is not None and not math.isfinite(friction)):
        raise ValueError("default_mass and friction must be finite")
    warnings = list(desc.warnings)
    upright = (0.0, 0.0, 0.0, 1.0) if up_axis == "z" else axis_angle((1, 0, 0), -math.pi / 2)
    base = compose((tuple(float(v) for v in position), tuple(float(v) for v in rotation)), ((0.0, 0.0, 0.0), upright))
    frames = {name: compose(base, f) for name, f in forward_kinematics(desc).items()}
    links: Dict[str, Handle] = {}
    joints: Dict[str, JointHandle] = {}
    anchors: List[Handle] = []
    parent_joint = {j.child: j for j in desc.joints}

    def carrier(name: str) -> Optional[str]:
        n: Optional[str] = name
        while n is not None:
            if n in links:
                return n
            n = parent_joint[n].parent if n in parent_joint else None
        return None

    def fit(value: float, lo: float, hi: float, what: str) -> float:
        v = min(hi, max(lo, value))
        if v != value:
            warnings.append(f"{what} {value:g} was clamped to {v:g}, the engine's bound")
        return v

    try:
        for l in desc.links:
            if l.shape is None:
                if l.mass is not None:
                    warnings.append(f"link {l.name}: it has a mass but no collision shape, so the mass is dropped")
                continue
            s = l.shape
            f = compose(frames[l.name], s.frame)
            pj = parent_joint.get(l.name)
            is_base = pj is None or carrier(pj.parent) is None
            is_static = l.static or (fixed_base and is_base and (pj is None or pj.type == "fixed"))
            mass = l.mass
            if mass is None and not is_static:
                warnings.append(f"link {l.name}: no mass given, using {default_mass:g} kg")
                mass = default_mass
            what = f"link {l.name}"
            half = lambda v, n: fit(v, MIN_HALF, MAX_HALF, f"{what} {n}")
            if s.kind == "box":
                shape, dims = SHAPE_BOX, (half(s.half[0], "half x"), half(s.half[1], "half y"), half(s.half[2], "half z"))
            elif s.kind == "sphere":
                shape, dims = SHAPE_SPHERE, (half(s.radius, "radius"),) * 3
            else:
                shape, dims = SHAPE_CAPSULE, (half(s.radius, "radius"), half(s.half_length, "half length"), half(s.radius, "radius"))
            inv_mass = 0.0 if is_static else 1.0 / fit(mass, MIN_MASS, MAX_MASS, f"{what} mass")
            kwargs = {} if friction is None else {"friction": friction}
            links[l.name] = world.spawn(f[0][0], f[0][1], f[0][2], rotation=f[1], shape=shape, half=dims, inv_mass=inv_mass, **kwargs)
        if not links:
            raise RobotError("the robot has no link with a collision shape, so there is nothing to spawn")
        for j in desc.order:
            if j.child not in links:
                if j.type != "fixed":
                    warnings.append(f"joint {j.name}: its child link {j.child} has no body, so the motion of this {j.type} joint is lost")
                continue
            a = carrier(j.parent)
            jf = compose(frames[j.parent], j.origin)
            if a is None:
                if j.type == "fixed":
                    continue
                carrier_handle = world.spawn(jf[0][0], jf[0][1], jf[0][2], shape=SHAPE_SPHERE, half=(ANCHOR_RADIUS,) * 3, inv_mass=0.0)
                anchors.append(carrier_handle)
                warnings.append(f"joint {j.name}: nothing above it has a body, so a {ANCHOR_RADIUS * 2:g} m static anchor was made at the joint")
            else:
                carrier_handle = links[a]
            kind = JOINT_FIXED if j.type == "fixed" else JOINT_SLIDER if j.type == "prismatic" else JOINT_HINGE
            limits = None
            if j.limit is not None:
                bound = MAX_ANGLE_LIMIT if kind == JOINT_HINGE else MAX_LINEAR_LIMIT
                lo, hi = min(0.0, max(j.limit[0], -bound)), max(0.0, min(j.limit[1], bound))
                if (lo, hi) != j.limit:
                    warnings.append(f"joint {j.name}: limits [{j.limit[0]:g}, {j.limit[1]:g}] were clamped to [{lo:g}, {hi:g}] (zero must lie inside, and the engine's bound is {bound:g})")
                limits = (lo, hi)
            joints[j.name] = world.create_joint(kind, carrier_handle, links[j.child], jf[0], None if kind == JOINT_FIXED else qrot(jf[1], j.axis), limits=limits)
    except BaseException:
        for h in [*links.values(), *anchors]:
            try:
                world.destroy(h)
            except Exception:
                pass
        raise
    return SpawnedRobot(desc, links, joints, anchors, frames, warnings)
