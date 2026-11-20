"""The wire contract with the runtime: header word indices, ring command tags and the 64-byte command slots.

Mirrors `core/src/command_ring.rs` and `sdk/typescript/src/CommandRing.ts`; the tests pin every encoder to bytes the TypeScript ring wrote.
"""
from __future__ import annotations

import struct
from typing import Optional, Sequence, Tuple

SLOT_BYTES = 64

H_EPOCH = 0
H_CAPACITY = 1
H_BUF0 = 2
H_STATE = 5
H_GENERATIONS = 6
H_TICKS = 7
H_STALE = 8
H_MALFORMED = 9
H_SEEKS = 31
H_LIVE = 32
H_PAUSED = 30
H_JSTATE_SEQ = 38
H_JSTATE_ROWS = 39
H_JSTATE_N = 40
H_PSTATE_SEQ = 41
H_PSTATE_ROWS = 42
H_PSTATE_N = 43
HEADER_WORDS = 44

FLOATS_PER_TRANSFORM = 7
JSTATE_ROW_WORDS = 4
PSTATE_ROW_WORDS = 4

TAG_SPAWN, TAG_DESTROY, TAG_FORCE, TAG_GROW, TAG_GRAVITY = 0, 1, 2, 3, 4
TAG_TELEPORT, TAG_RECORD, TAG_SEEK, TAG_PAUSE = 8, 9, 10, 11
TAG_JOINT_CREATE, TAG_JOINT_DESTROY, TAG_JOINT_MOTOR = 12, 13, 14
TAG_CHECKPOINT, TAG_RESET = 18, 19
TAG_PARTICLE_CHAIN, TAG_PARTICLE_CLOTH, TAG_PARTICLE_ATTACH, TAG_PARTICLE_DETACH, TAG_PARTICLE_SOFT = 20, 21, 22, 23, 24

SHAPE_BOX, SHAPE_SPHERE, SHAPE_CAPSULE, SHAPE_HULL = 0, 1, 2, 3
JOINT_BALL, JOINT_HINGE, JOINT_FIXED, JOINT_SLIDER, JOINT_DISTANCE = 0, 1, 2, 3, 4

MAX_CAPACITY = 1 << 18
MAX_JOINT_ID = 1 << 24
MAX_ANGLE_LIMIT = 3.1
MAX_LINEAR_LIMIT = 1e4
MAX_MOTOR_SPEED = 1e3
MAX_MOTOR_FORCE = 1e6
MAX_TELEPORT_COORD = 1e6
MAX_GRAVITY = 1000.0
MAX_PARTICLES = 1 << 14
MAX_CHAIN = 1024
MAX_CLOTH_SIDE = 64
MAX_SOFT_SIDE = 12
MAX_GROUP_ID = 1 << 24
MAX_PARTICLE_GROUPS = 1 << 12

_SPAWN = struct.Struct("<BIB7fBB6fI")
_ENTITY_VEC = struct.Struct("<BIB3f")
_DESTROY = struct.Struct("<BIB")
_VEC3 = struct.Struct("<B5x3f")
_FLAG = struct.Struct("<B5xB")
_U32 = struct.Struct("<BI")
_GROW = struct.Struct("<B5xI")
_MOTOR = struct.Struct("<BI1x2f")
_JOINT = struct.Struct("<BIBBBII3f3f2f2fB")
_CHAIN = struct.Struct("<BIx3f3fHB3x3f")
_CLOTH = struct.Struct("<BIx3f3f3fHHBx4f")
_SOFT = struct.Struct("<BIx3f3f3B3x3f")
_ATTACH = struct.Struct("<BIB3fI")
assert _SPAWN.size == SLOT_BYTES
assert (_CHAIN.size, _CLOTH.size, _SOFT.size, _ATTACH.size) == (48, 64, 48, 22)


def _slot(packed: bytes) -> bytes:
    return packed.ljust(SLOT_BYTES, b"\0")


def spawn(id: int, gen: int, pos: Sequence[float], rot: Sequence[float], shape: int, half: Sequence[float], inv_mass: float, restitution: float, friction: float, hull: int) -> bytes:
    return _SPAWN.pack(TAG_SPAWN, id, gen, *pos, *rot, shape, 0, *half, inv_mass, restitution, friction, hull)


def destroy(id: int, gen: int) -> bytes:
    return _slot(_DESTROY.pack(TAG_DESTROY, id, gen))


def force(id: int, gen: int, x: float, y: float, z: float) -> bytes:
    return _slot(_ENTITY_VEC.pack(TAG_FORCE, id, gen, x, y, z))


def teleport(id: int, gen: int, x: float, y: float, z: float) -> bytes:
    return _slot(_ENTITY_VEC.pack(TAG_TELEPORT, id, gen, x, y, z))


def gravity(x: float, y: float, z: float) -> bytes:
    return _slot(_VEC3.pack(TAG_GRAVITY, x, y, z))


def grow(capacity: int) -> bytes:
    return _slot(_GROW.pack(TAG_GROW, capacity))


def record(on: bool) -> bytes:
    return _slot(_FLAG.pack(TAG_RECORD, int(on)))


def pause(on: bool) -> bytes:
    return _slot(_FLAG.pack(TAG_PAUSE, int(on)))


def seek(tick: int) -> bytes:
    return _slot(_U32.pack(TAG_SEEK, tick))


def checkpoint() -> bytes:
    return _slot(bytes([TAG_CHECKPOINT]))


def reset() -> bytes:
    return _slot(bytes([TAG_RESET]))


def joint_create(id: int, kind: int, a: int, gen_a: int, b: int, gen_b: int, anchor: Sequence[float], axis: Sequence[float], limits: Optional[Tuple[float, float]], motor_velocity: float, motor_max: float) -> bytes:
    lo, hi = limits if limits is not None else (0.0, 0.0)
    return _JOINT.pack(TAG_JOINT_CREATE, id, kind, gen_a, gen_b, a, b, *anchor, *axis, lo, hi, motor_velocity, motor_max, 1 if limits is not None else 0).ljust(SLOT_BYTES, b"\0")


def joint_destroy(id: int) -> bytes:
    return _slot(_U32.pack(TAG_JOINT_DESTROY, id))


def joint_motor(id: int, velocity: float, max_force: float) -> bytes:
    return _slot(_MOTOR.pack(TAG_JOINT_MOTOR, id, velocity, max_force))


def particle_chain(id: int, frm: Sequence[float], to: Sequence[float], count: int, pin_first: bool, pin_last: bool, radius: float, inv_mass: float, stiffness: float) -> bytes:
    return _slot(_CHAIN.pack(TAG_PARTICLE_CHAIN, id, *frm, *to, count, (1 if pin_first else 0) | (2 if pin_last else 0), radius, inv_mass, stiffness))


def particle_cloth(id: int, origin: Sequence[float], u: Sequence[float], v: Sequence[float], nu: int, nv: int, pins: int, radius: float, inv_mass: float, stiffness: float, bend_stiffness: float) -> bytes:
    return _slot(_CLOTH.pack(TAG_PARTICLE_CLOTH, id, *origin, *u, *v, nu, nv, pins, radius, inv_mass, stiffness, bend_stiffness))


def particle_soft(id: int, origin: Sequence[float], size: Sequence[float], n: Sequence[int], radius: float, inv_mass: float, stiffness: float) -> bytes:
    return _slot(_SOFT.pack(TAG_PARTICLE_SOFT, id, *origin, *size, *n, radius, inv_mass, stiffness))


def particle_attach(particle: int, body: int, gen: int, local: Sequence[float]) -> bytes:
    return _slot(_ATTACH.pack(TAG_PARTICLE_ATTACH, particle, gen, *local, body))


def particle_detach(particle: int) -> bytes:
    return _slot(_U32.pack(TAG_PARTICLE_DETACH, particle))
