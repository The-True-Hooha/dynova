"""One world of a batch: commands go in through its ring, reads come straight out of linear memory as numpy views."""
from __future__ import annotations

import math
import struct
from collections import deque
from typing import Deque, List, NamedTuple, Optional, Sequence, Tuple

import numpy as np

from . import _wire as w
from ._wasm import Instance

__all__ = ["World", "Handle", "JointHandle", "ParticleGroup", "WorldFailed", "StaleHandle", "RingFull"]

DEFAULT_MAX_DEFERRED = 65536
_NOT_LIVE = -1

JOINT_STATE_DTYPE = np.dtype([("joint", "<u4"), ("kind", "<u4"), ("position", "<f4"), ("velocity", "<f4")])


class Handle(NamedTuple):
    """An entity: its slot and the generation that makes a reused slot a different entity."""
    id: int
    gen: int


class ParticleGroup(NamedTuple):
    """A group a particle command made: its id, the index of its first particle in `read_particles()` and its size."""
    id: int
    first: int
    count: int


class JointHandle(NamedTuple):
    id: int


class WorldFailed(RuntimeError):
    """The world's wasm instance trapped or was closed; nothing more can be done with it."""


class StaleHandle(ValueError):
    """The handle names an entity that no longer exists (destroyed, or gone after a reset)."""


class RingFull(RuntimeError):
    """The command ring and the deferred queue are both full."""


def _finite(*values: float) -> bool:
    return all(math.isfinite(v) for v in values)


def _encode(encoder, *args):
    """A value a 32-bit float cannot hold is the caller's mistake, reported like every other bad argument."""
    try:
        return encoder(*args)
    except (OverflowError, struct.error) as err:
        raise ValueError(f"a value is outside the range of a 32-bit float: {err}") from err


def _f32_square_sum(vector: Sequence[float]) -> float:
    """The squared length the way the runtime computes it, in 32-bit floats: it must not overflow to infinity."""
    total = np.float32(0.0)
    with np.errstate(over="ignore"):
        for v in vector:
            square = np.float32(float(np.float32(v)) ** 2)
            total = np.float32(float(total) + float(square))
    return float(total)


class World:
    def __init__(self, instance: Instance, layout: Sequence[int], capacity: int, max_deferred: int = DEFAULT_MAX_DEFERRED) -> None:
        header, header_words, slots, head, tail, counter, ring_capacity = layout
        if header_words != w.HEADER_WORDS:
            raise RuntimeError(f"the wasm header has {header_words} words but this package expects {w.HEADER_WORDS}: rebuild the wasm")
        self._inst = instance
        self._header_at, self._slots_at, self._head_at, self._tail_at, self._counter_at = header, slots, head, tail, counter
        self.ring_capacity = ring_capacity
        self._mask = ring_capacity - 1
        self._max_deferred = max_deferred
        self._deferred: Deque[bytes] = deque()
        self._capacity = capacity
        self._live_gen: List[int] = [_NOT_LIVE] * capacity
        self._free: List[Handle] = []
        self._next_joint = 0
        self._next_group = 0
        self._next_particle = 0
        self._next_granular = 0
        self._known_seeks = 0
        self._fatal: Optional[str] = None

    # ----- plumbing -------------------------------------------------------------------------------------------------------------

    def _header(self) -> np.ndarray:
        return self._inst.words(self._header_at, w.HEADER_WORDS)

    def _alive(self) -> None:
        if self._inst.busy:
            raise RuntimeError("the batch is stepping; a world cannot be touched from another thread until step() returns")
        if self._fatal is not None:
            raise WorldFailed(self._fatal)
        if self._inst.failed is not None:
            raise WorldFailed(f"world unavailable: {self._inst.failed}")
        self._sync()

    def mark_failed(self, reason: str) -> None:
        self._fatal = reason

    @property
    def failure(self) -> Optional[str]:
        return self._fatal or (None if self._inst.failed is None else f"world unavailable: {self._inst.failed}")

    def _ring_pending(self) -> int:
        head = int(self._inst.words(self._head_at, 1)[0])
        tail = int(self._inst.words(self._tail_at, 1)[0])
        return (tail - head) & 0xFFFFFFFF

    def _ring_push(self, slot: bytes) -> bool:
        assert len(slot) == w.SLOT_BYTES
        if self._ring_pending() >= self.ring_capacity:
            return False
        tail = self._inst.words(self._tail_at, 1)
        at = self._slots_at + (int(tail[0]) & self._mask) * w.SLOT_BYTES
        self._inst.mem()[at : at + w.SLOT_BYTES] = np.frombuffer(slot, dtype=np.uint8)
        tail[0] = (int(tail[0]) + 1) & 0xFFFFFFFF
        return True

    def _flush(self) -> int:
        while self._deferred and self._ring_push(self._deferred[0]):
            self._deferred.popleft()
        return len(self._deferred)

    def _enqueue(self, slot: bytes) -> bool:
        if self._flush() == 0 and self._ring_push(slot):
            return True
        if len(self._deferred) >= self._max_deferred:
            return False
        self._deferred.append(slot)
        return True

    def _send(self, slot: bytes) -> None:
        if not self._enqueue(slot):
            raise RingFull(f"the command ring is full and {len(self._deferred)} commands are deferred; raise ring_capacity")

    @property
    def pending_commands(self) -> int:
        return self._ring_pending() + len(self._deferred)

    @property
    def deferred_commands(self) -> int:
        return len(self._deferred)

    def after_step(self) -> None:
        """What follows a step: queued commands that did not fit the ring go in, and the entity tables follow the runtime."""
        if self._fatal is None and self._inst.failed is None:
            self._flush()
            self._sync()

    def _sync(self) -> None:
        header = self._header()
        capacity = int(header[w.H_CAPACITY])
        if capacity > self._capacity:
            self._live_gen.extend([_NOT_LIVE] * (capacity - self._capacity))
            self._capacity = capacity
        seeks = int(header[w.H_SEEKS])
        # Only once nothing is in flight: a queued spawn is not in the runtime's tables yet, so resyncing now would hand its id out twice.
        if seeks != self._known_seeks and self.pending_commands == 0:
            self._known_seeks = seeks
            self._resync(header)

    def _resync(self, header: np.ndarray) -> None:
        capacity = int(header[w.H_CAPACITY])
        mem = self._inst.mem()
        generations = mem[int(header[w.H_GENERATIONS]) : int(header[w.H_GENERATIONS]) + capacity]
        live = mem[int(header[w.H_LIVE]) : int(header[w.H_LIVE]) + capacity]
        self._live_gen = [_NOT_LIVE] * self._capacity
        self._free = []
        allocated = min(self._allocated(), capacity)
        for id in range(allocated - 1, -1, -1):
            if live[id]:
                self._live_gen[id] = int(generations[id])
            else:
                self._free.append(Handle(id, int(generations[id])))

    def _allocated(self) -> int:
        return int(self._inst.words(self._counter_at, 1)[0])

    # ----- entities -------------------------------------------------------------------------------------------------------------

    def _alloc(self) -> Handle:
        if self._free:
            handle = self._free.pop()
            self._live_gen[handle.id] = handle.gen
            return handle
        counter = self._inst.words(self._counter_at, 1)
        id = int(counter[0])
        if id >= self._capacity:
            raise RingFull(f"entity capacity exhausted ({self._capacity}); build the batch with a larger capacity")
        counter[0] = id + 1
        self._live_gen[id] = 0
        return Handle(id, 0)

    def is_live(self, handle: Handle) -> bool:
        return 0 <= handle.id < self._capacity and self._live_gen[handle.id] == handle.gen

    def _require_live(self, handle: Handle) -> None:
        if not self.is_live(handle):
            raise StaleHandle(f"stale handle {tuple(handle)}")

    def spawn(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, *, rotation: Sequence[float] = (0.0, 0.0, 0.0, 1.0), shape: int = w.SHAPE_BOX,
              half: Sequence[float] = (0.5, 0.5, 0.5), inv_mass: float = 1.0, restitution: float = 0.3, friction: float = 0.5, hull: int = 0) -> Handle:
        self._alive()
        if shape == w.SHAPE_HULL:
            raise NotImplementedError("hull bodies are not supported by the Python package yet")
        if len(rotation) != 4 or len(half) != 3 or not _finite(x, y, z, *rotation, *half, inv_mass, restitution, friction):
            raise ValueError("a body needs a finite position, a 4-component rotation, 3 half extents and finite mass, restitution and friction")
        handle = self._alloc()
        try:
            slot = _encode(w.spawn, handle.id, handle.gen, (x, y, z), rotation, shape, half, inv_mass, restitution, friction, hull)
            self._send(slot)
        except BaseException:
            # Rust never saw this generation, so give the slot back without bumping it.
            self._live_gen[handle.id] = _NOT_LIVE
            self._free.append(handle)
            raise
        return handle

    def destroy(self, handle: Handle) -> None:
        self._alive()
        self._require_live(handle)
        self._send(w.destroy(handle.id, handle.gen))
        self._live_gen[handle.id] = _NOT_LIVE
        self._free.append(Handle(handle.id, (handle.gen + 1) & 0xFF))

    def force(self, handle: Handle, x: float, y: float, z: float) -> None:
        self._alive()
        self._require_live(handle)
        self._send(_encode(w.force, handle.id, handle.gen, x, y, z))

    def teleport(self, handle: Handle, x: float, y: float, z: float) -> None:
        self._alive()
        if not (_finite(x, y, z) and max(abs(x), abs(y), abs(z)) <= w.MAX_TELEPORT_COORD):
            raise ValueError(f"teleport position must be finite and within +-{w.MAX_TELEPORT_COORD}")
        self._require_live(handle)
        self._send(_encode(w.teleport, handle.id, handle.gen, x, y, z))

    def set_gravity(self, x: float, y: float, z: float) -> None:
        self._alive()
        if not (_finite(x, y, z) and max(abs(x), abs(y), abs(z)) <= w.MAX_GRAVITY):
            raise ValueError(f"gravity must be finite and within +-{w.MAX_GRAVITY}")
        self._send(w.gravity(x, y, z))

    def pause(self, on: bool) -> None:
        self._alive()
        self._send(w.pause(on))

    def record(self, on: bool) -> None:
        self._alive()
        self._send(w.record(on))

    def seek(self, tick: int) -> None:
        self._alive()
        if not isinstance(tick, (int, np.integer)) or not 0 <= tick <= 0xFFFFFFFF:
            raise ValueError(f"tick must be an integer in [0, 2^32), got {tick!r}")
        self._send(w.seek(tick))

    def checkpoint(self) -> None:
        """Remember the world as it is when the runtime reaches this command; `reset` returns to it."""
        self._alive()
        self._send(w.checkpoint())

    def reset(self) -> None:
        """Return to the checkpoint without advancing; refused (counted in `malformed_dropped`) with no checkpoint or while recording."""
        self._alive()
        self._send(w.reset())

    # ----- joints ---------------------------------------------------------------------------------------------------------------

    @staticmethod
    def _check_motor(velocity: float, max_force: float) -> None:
        if not math.isfinite(velocity) or abs(velocity) > w.MAX_MOTOR_SPEED:
            raise ValueError(f"motor velocity must be finite and within +-{w.MAX_MOTOR_SPEED}")
        if not math.isfinite(max_force) or not 0 <= max_force <= w.MAX_MOTOR_FORCE:
            raise ValueError(f"motor max_force must be within 0..{w.MAX_MOTOR_FORCE}")

    def create_joint(self, kind: int, a: Handle, b: Handle, anchor: Sequence[float], axis: Optional[Sequence[float]] = None, *,
                     anchor_b: Optional[Sequence[float]] = None, limits: Optional[Tuple[float, float]] = None,
                     motor: Optional[Tuple[float, float]] = None) -> JointHandle:
        """`motor` is `(velocity, max_force)`; `anchor_b` is the second anchor of a distance joint. The rules of the TypeScript `createJoint`."""
        self._alive()
        if kind not in (w.JOINT_BALL, w.JOINT_HINGE, w.JOINT_FIXED, w.JOINT_SLIDER, w.JOINT_DISTANCE):
            raise ValueError(f"unknown joint kind {kind}")
        self._require_live(a)
        self._require_live(b)
        if a.id == b.id:
            raise ValueError("a joint needs two different bodies")
        vec3 = lambda v: len(v) == 3 and _finite(*v)
        if not vec3(anchor) or max(abs(v) for v in anchor) > w.MAX_TELEPORT_COORD:
            raise ValueError(f"joint anchor must be finite and within +-{w.MAX_TELEPORT_COORD}")
        axial = kind in (w.JOINT_HINGE, w.JOINT_SLIDER)
        distance = kind == w.JOINT_DISTANCE
        if not distance and anchor_b is not None:
            raise ValueError("only a distance joint takes a second anchor")
        if distance and axis is not None:
            raise ValueError("a distance joint is set by its two anchors, not an axis")
        if distance:
            if anchor_b is None or not vec3(anchor_b) or max(abs(v) for v in anchor_b) > w.MAX_TELEPORT_COORD:
                raise ValueError(f"a distance joint needs a second anchor, finite and within +-{w.MAX_TELEPORT_COORD}")
            f32 = lambda v: float(np.float32(v))
            squared = np.float32(0.0)
            for p, q in zip(anchor, anchor_b):
                d = np.float32(f32(p) - f32(q))
                squared = np.float32(squared + np.float32(d * d))
            if not squared >= 1e-6:
                raise ValueError("the two anchors of a distance joint must be at least 1 mm apart")
        vector = tuple(anchor_b) if distance else tuple(axis) if axis is not None else (0.0, 0.0, 0.0)
        if not vec3(vector):
            raise ValueError("joint axis must be finite")
        if axial and not (math.hypot(*vector) > 1e-6 and np.isfinite(_f32_square_sum(vector))):
            raise ValueError("hinge and slider joints need a finite, non-zero axis")
        if motor is not None and not axial:
            raise ValueError("only hinge and slider joints take a motor")
        if limits is not None and not (axial or distance):
            raise ValueError("only hinge, slider and distance joints take limits")
        if limits is not None:
            bound = w.MAX_ANGLE_LIMIT if kind == w.JOINT_HINGE else w.MAX_LINEAR_LIMIT
            lo, hi = limits
            if not _finite(lo, hi) or lo > 0 or hi < 0 or abs(lo) > bound or abs(hi) > bound:
                raise ValueError(f"joint limits must satisfy lo <= 0 <= hi within +-{bound}")
        if motor is not None:
            self._check_motor(*motor)
        if self._next_joint >= w.MAX_JOINT_ID:
            raise RingFull(f"joint limit reached ({w.MAX_JOINT_ID})")
        velocity, max_force = motor if motor is not None else (0.0, 0.0)
        self._send(_encode(w.joint_create, self._next_joint, kind, a.id, a.gen, b.id, b.gen, anchor, vector, limits, velocity, max_force))
        self._next_joint += 1
        return JointHandle(self._next_joint - 1)

    def set_joint_motor(self, joint: JointHandle, velocity: float, max_force: float) -> None:
        self._alive()
        self._check_motor(velocity, max_force)
        self._send(_encode(w.joint_motor, joint.id, velocity, max_force))

    def destroy_joint(self, joint: JointHandle) -> None:
        self._alive()
        self._send(w.joint_destroy(joint.id))

    # ----- reads ----------------------------------------------------------------------------------------------------------------

    def read_transforms(self, copy: bool = False) -> np.ndarray:
        """`(capacity, 7)` float32 rows `x y z qx qy qz qw` of the last published tick; a row with `qw == 0` is an empty slot.

        Without `copy` this is a view onto the module's memory: it keeps the instance alive, but its contents change at the next
        step and it must not be used after any call into the module that can grow memory (a step, a command that allocates).
        """
        self._alive()
        header = self._header()
        state = int(header[w.H_STATE])
        if state & 0b100:
            state = ((state >> 3) & 3) | ((state & 3) << 3)
            header[w.H_STATE] = state
        index = (state >> 3) & 3
        capacity = int(header[w.H_CAPACITY])
        at = int(header[w.H_BUF0 + index])
        view = self._inst.mem()[at : at + capacity * w.FLOATS_PER_TRANSFORM * 4].view(np.float32).reshape(capacity, w.FLOATS_PER_TRANSFORM)
        return view.copy() if copy else self._inst.owned(view)

    def _group(self, count: int, send, granular: bool = False) -> "ParticleGroup":
        if self._next_group >= w.MAX_GROUP_ID:
            raise ValueError("group id limit reached")
        if self._next_group >= w.MAX_PARTICLE_GROUPS:
            raise ValueError(f"a world holds at most {w.MAX_PARTICLE_GROUPS} particle groups")
        if self._next_particle + count > w.MAX_PARTICLES:
            raise ValueError(f"a world holds at most {w.MAX_PARTICLES} particles")
        if granular and self._next_granular + count > w.MAX_FLUID_PARTICLES:
            raise ValueError(f"a world holds at most {w.MAX_FLUID_PARTICLES} fluid and sand particles")
        send(self._next_group)
        if granular:
            self._next_granular += count
        group = ParticleGroup(self._next_group, self._next_particle, count)
        self._next_group += 1
        self._next_particle += count
        return group

    @staticmethod
    def _check_particle(vectors: Sequence[Sequence[float]], radius: float, inv_mass: float, stiffness: float) -> None:
        f32 = lambda x: float(np.float32(x))
        radius, inv_mass, stiffness = (f32(x) if _finite(x) and abs(x) < 3e38 else x for x in (radius, inv_mass, stiffness))
        if not all(len(v) == 3 and _finite(*v) and max(abs(c) for c in v) <= w.MAX_TELEPORT_COORD for v in vectors):
            raise ValueError("particle positions and sizes must be finite 3-vectors within range")
        if not (_finite(radius) and 1e-3 <= radius <= 100):
            raise ValueError(f"particle radius must be within 0.001..100, got {radius!r}")
        if not (_finite(inv_mass) and 0 <= inv_mass <= 1e6):
            raise ValueError(f"particle inv_mass must be within 0..1e6, got {inv_mass!r}")
        if not (_finite(stiffness) and 0 < stiffness <= 1):
            raise ValueError(f"stiffness must be within (0, 1], got {stiffness!r}")

    def create_chain(self, frm: Sequence[float], to: Sequence[float], count: int, *, pin_first: bool = False, pin_last: bool = False, radius: float = 0.02,
                     inv_mass: float = 1.0, stiffness: float = 1.0) -> "ParticleGroup":
        """A rope of `count` particles from `frm` to `to`. The rules of the TypeScript `createChain`; `first` assumes earlier particle commands were accepted."""
        self._alive()
        if not isinstance(count, (int, np.integer)) or not 2 <= count <= w.MAX_CHAIN:
            raise ValueError(f"a chain has 2..{w.MAX_CHAIN} particles, got {count!r}")
        self._check_particle([frm, to], radius, inv_mass, stiffness)
        if not math.dist(frm, to) > 1e-4:
            raise ValueError("the two ends of a chain must be apart")
        return self._group(count, lambda id: self._send(w.particle_chain(id, frm, to, count, pin_first, pin_last, radius, inv_mass, stiffness)))

    def create_cloth(self, origin: Sequence[float], u: Sequence[float], v: Sequence[float], nu: int, nv: int, *, pins: int = 0, radius: float = 0.05,
                     inv_mass: float = 1.0, stiffness: float = 1.0, bend_stiffness: float = 0.3) -> "ParticleGroup":
        self._alive()
        for n in (nu, nv):
            if not isinstance(n, (int, np.integer)) or not 2 <= n <= w.MAX_CLOTH_SIDE:
                raise ValueError(f"a cloth is 2..{w.MAX_CLOTH_SIDE} particles a side, got {n!r}")
        if not isinstance(pins, (int, np.integer)) or not 0 <= pins <= 31:
            raise ValueError("cloth pins is a 5 bit mask")
        if not (_finite(bend_stiffness) and 0 <= bend_stiffness <= 1):
            raise ValueError("bend_stiffness must be within 0..1")
        self._check_particle([origin, u, v], radius, inv_mass, stiffness)
        cross = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        if not math.hypot(*cross) > 1e-4:
            raise ValueError("the edge vectors u and v of a cloth must span an area")
        return self._group(nu * nv, lambda id: self._send(w.particle_cloth(id, origin, u, v, nu, nv, pins, radius, inv_mass, stiffness, bend_stiffness)))

    def create_soft_block(self, origin: Sequence[float], size: Sequence[float], n: Sequence[int], *, radius: float = 0.05, inv_mass: float = 1.0,
                          stiffness: float = 1.0) -> "ParticleGroup":
        self._alive()
        if len(n) != 3 or not all(isinstance(k, (int, np.integer)) and 2 <= k <= w.MAX_SOFT_SIDE for k in n):
            raise ValueError(f"a soft block is 2..{w.MAX_SOFT_SIDE} particles along each axis, got {tuple(n)!r}")
        self._check_particle([origin, size], radius, inv_mass, stiffness)
        if not all(s > 1e-3 for s in size):
            raise ValueError("a soft block needs a size above 1 mm on every axis")
        return self._group(n[0] * n[1] * n[2], lambda id: self._send(w.particle_soft(id, origin, size, n, radius, inv_mass, stiffness)))

    @staticmethod
    def _check_lattice(origin: Sequence[float], n: Sequence[int], spacing: float, name: str, last: float) -> None:
        if len(n) != 3 or not all(isinstance(k, (int, np.integer)) and 2 <= k <= w.MAX_FLUID_SIDE for k in n):
            raise ValueError(f"a lattice is 2..{w.MAX_FLUID_SIDE} particles along each axis, got {tuple(n)!r}")
        if not (len(origin) == 3 and _finite(*origin) and max(abs(c) for c in origin) <= w.MAX_TELEPORT_COORD):
            raise ValueError("the origin must be a finite 3-vector within range")
        f32 = lambda x: float(np.float32(x)) if _finite(x) and abs(x) < 3e38 else x
        if not (_finite(spacing) and f32(0.01) <= f32(spacing) <= 1):
            raise ValueError(f"spacing must be within 0.01..1 m, got {spacing!r}")
        if not (_finite(last) and 0 <= f32(last) <= 1):
            raise ValueError(f"{name} must be within 0..1, got {last!r}")

    def create_fluid(self, origin: Sequence[float], n: Sequence[int], spacing: float, *, viscosity: float = 0.02) -> "ParticleGroup":
        """A block of water particles on a lattice. The rules of the TypeScript `createFluid`; fluid and sand share a cap of 4096 particles."""
        self._check_lattice(origin, n, spacing, "viscosity", viscosity)
        return self._group(n[0] * n[1] * n[2], lambda id: self._send(w.particle_fluid(id, origin, n, spacing, viscosity)), granular=True)

    def create_sand(self, origin: Sequence[float], n: Sequence[int], spacing: float, *, friction: float = 0.6) -> "ParticleGroup":
        """A block of sand grains on a lattice, with Coulomb friction in 0..1. The rules of the TypeScript `createSand`."""
        self._check_lattice(origin, n, spacing, "friction", friction)
        return self._group(n[0] * n[1] * n[2], lambda id: self._send(w.particle_sand(id, origin, n, spacing, friction)), granular=True)

    def attach_particle(self, particle: int, body: Handle, local: Sequence[float]) -> None:
        """Hold particle number `particle` (a row of `read_particles()`) on `body` at `local` in the body's frame."""
        self._alive()
        if not isinstance(particle, (int, np.integer)) or not 0 <= particle < w.MAX_PARTICLES:
            raise ValueError(f"particle must be an index below {w.MAX_PARTICLES}, got {particle!r}")
        if len(local) != 3 or not _finite(*local) or max(abs(c) for c in local) > w.MAX_TELEPORT_COORD:
            raise ValueError("the attachment offset must be finite and within range")
        self._send(w.particle_attach(particle, body.id, body.gen, local))

    def detach_particle(self, particle: int) -> None:
        self._alive()
        if not isinstance(particle, (int, np.integer)) or not 0 <= particle < w.MAX_PARTICLES:
            raise ValueError(f"particle must be an index below {w.MAX_PARTICLES}, got {particle!r}")
        self._send(w.particle_detach(particle))

    def read_particles(self) -> np.ndarray:
        """`(n, 4)` float32 rows `x y z radius` of every particle at the last published tick (a copy, read inside the runtime's sequence word)."""
        self._alive()
        header = self._header()
        for _ in range(64):
            seq = int(header[w.H_PSTATE_SEQ])
            if seq & 1:
                continue
            rows, count = int(header[w.H_PSTATE_ROWS]), int(header[w.H_PSTATE_N])
            data = self._inst.mem()[rows : rows + count * w.PSTATE_ROW_WORDS * 4].tobytes() if rows and count else b""
            if int(header[w.H_PSTATE_SEQ]) == seq:
                return np.frombuffer(data, dtype=np.float32).reshape(-1, w.PSTATE_ROW_WORDS).copy()
        raise RuntimeError("particle rows kept changing while they were read")

    def read_joint_states(self) -> np.ndarray:
        """Structured rows `(joint, kind, position, velocity)` of every hinge, slider and distance joint at the last published tick (a copy)."""
        self._alive()
        header = self._header()
        rows, count = int(header[w.H_JSTATE_ROWS]), int(header[w.H_JSTATE_N])
        if rows == 0 or count == 0:
            return np.zeros(0, dtype=JOINT_STATE_DTYPE)
        raw = self._inst.mem()[rows : rows + count * w.JSTATE_ROW_WORDS * 4]
        return np.frombuffer(raw.tobytes(), dtype=JOINT_STATE_DTYPE)

    @property
    def ticks(self) -> int:
        return int(self._header()[w.H_TICKS])

    @property
    def stale_dropped(self) -> int:
        return int(self._header()[w.H_STALE])

    @property
    def malformed_dropped(self) -> int:
        return int(self._header()[w.H_MALFORMED])

    @property
    def capacity(self) -> int:
        return self._capacity
