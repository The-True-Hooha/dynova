"""Many copies of one task, one per world of a `Batch`, behind `reset` and `step` on numpy arrays.

The semantics are those of the TypeScript `BatchEnv` (`sdk/typescript/src/env.ts`, `docs/env.md`) and the two produce the same numbers: the same
seeded generator, the same NEXT_STEP autoreset, and a world has the same history whatever the number of worlds or workers beside it.
"""
from __future__ import annotations

import math
import numbers
from dataclasses import dataclass
from typing import Any, Callable, List, Optional

import numpy as np

from ._wasm import WasmError
from .world import World

__all__ = ["Rng", "mulberry32", "episode_seed", "EnvSpec", "EnvResult", "BatchEnv", "EnvError"]

_M = 0xFFFFFFFF
_MAX_DRAIN_STEPS = 10_000


class EnvError(RuntimeError):
    """A hook failed, a value was not finite, a command was refused, or a world failed. After one that queued commands the environment is unusable."""


class Rng:
    """mulberry32: integer arithmetic only, so every engine draws the same numbers from the same seed (the TypeScript generator draws the same ones)."""

    def __init__(self, seed: int) -> None:
        self._a = seed & _M

    def next(self) -> float:
        """In [0, 1)."""
        self._a = (self._a + 0x6D2B79F5) & _M
        a = self._a
        t = ((a ^ (a >> 15)) * (1 | a)) & _M
        t = (((t + (((t ^ (t >> 7)) * (61 | t)) & _M)) & _M) ^ t)
        return ((t ^ (t >> 14)) & _M) / 4294967296.0

    def range(self, lo: float, hi: float) -> float:
        return lo + (hi - lo) * self.next()

    def int(self, n: int) -> int:
        """An integer in [0, n)."""
        return math.floor(self.next() * n)


def mulberry32(seed: int) -> Rng:
    return Rng(seed)


def episode_seed(base: int, world: int, episode: int) -> int:
    """The seed of one episode of one world: a different stream for every (world, episode) pair of one base seed."""
    h = ((base ^ 0x9E3779B9) & _M) * 0x85EBCA6B & _M
    h ^= h >> 13
    h = ((h ^ world) & _M) * 0xC2B2AE35 & _M
    h ^= h >> 16
    h = ((h ^ episode) & _M) * 0x27D4EB2F & _M
    h ^= h >> 15
    return h & _M


class EnvSpec:
    """What a task is. `build` returns a per-world context that is handed to every other hook of that world.

    Hooks read worlds through their bulk reads (`read_transforms`, `read_joint_states`) and write through commands. A `reset` hook may only use
    what `build` returned: the entity tables re-sync after the step that applies the reset, so it must not spawn, destroy or create joints.
    """

    obs_size: int
    action_size: int
    max_steps: int

    def build(self, world: World, index: int, count: int) -> Any:
        raise NotImplementedError

    def reset(self, world: World, index: int, ctx: Any, rng: Rng) -> None:
        """Randomise the start of an episode with commands; the world has just returned to its checkpoint."""

    def act(self, world: World, index: int, ctx: Any, action: np.ndarray) -> None:
        raise NotImplementedError

    def observe(self, world: World, index: int, ctx: Any, out: np.ndarray) -> None:
        raise NotImplementedError

    def reward(self, world: World, index: int, ctx: Any, obs: np.ndarray) -> float:
        raise NotImplementedError

    def terminated(self, world: World, index: int, ctx: Any, obs: np.ndarray) -> bool:
        raise NotImplementedError


@dataclass
class EnvResult:
    """One row per world. These arrays are the environment's own and are overwritten by the next `reset` or `step`; copy what must outlive it.

    A world with `resetting` set was reset this step: `obs` is the first observation of a new episode, `reward` is 0 and the action given for it was ignored.
    """
    obs: np.ndarray
    reward: np.ndarray
    terminated: np.ndarray
    truncated: np.ndarray
    resetting: np.ndarray


def _check_seed(seed: int) -> int:
    if isinstance(seed, bool) or not isinstance(seed, (int, np.integer)) or not 0 <= seed <= _M:
        raise ValueError(f"seed must be an integer in [0, 2^32), got {seed!r}")
    return int(seed)


def _at(world: int, hook: str, fn: Callable[[], Any]) -> Any:
    try:
        return fn()
    except Exception as err:
        raise EnvError(f"world {world} {hook}: {err}") from err


def _is_number(value: Any) -> bool:
    return isinstance(value, (numbers.Real, np.floating, np.integer)) and not isinstance(value, (bool, np.bool_))


def _is_bool(value: Any) -> bool:
    return isinstance(value, (bool, np.bool_))


class BatchEnv:
    """Builds every world of `batch`, takes its checkpoint and resets it. Once a call fails after it queued commands the environment is unusable."""

    def __init__(self, batch, spec: EnvSpec, seed: int = 0) -> None:
        for name in ("obs_size", "action_size", "max_steps"):
            value = getattr(spec, name, None)
            if isinstance(value, bool) or not isinstance(value, numbers.Integral) or value < 1:
                raise ValueError(f"{name} must be a positive integer, got {value!r}")
        seed = _check_seed(seed)
        worlds: List[World] = list(batch.worlds)
        if not worlds:
            raise ValueError("an environment needs at least one world")
        self.batch, self.spec, self.size = batch, spec, len(worlds)
        self._worlds = worlds
        self._seed = seed
        self._broken: Optional[str] = None
        self._busy = False
        n = self.size
        self.result = EnvResult(
            obs=np.zeros((n, spec.obs_size), dtype=np.float32), reward=np.zeros(n, dtype=np.float32),
            terminated=np.zeros(n, dtype=bool), truncated=np.zeros(n, dtype=bool), resetting=np.zeros(n, dtype=bool),
        )
        self._steps = np.zeros(n, dtype=np.uint32)
        self._episode = np.zeros(n, dtype=np.uint32)
        self._pending_reset = np.zeros(n, dtype=bool)
        self._refused_before = np.full(n, -1, dtype=np.int64)
        refused = [w.malformed_dropped for w in worlds]
        for w in worlds:
            w.pause(True)
        # The worlds are paused while they are built, so the checkpoint is the scene as `build` left it however many commands that took.
        self._ctx = [_at(i, "build", lambda w=w, i=i: spec.build(w, i, n)) for i, w in enumerate(worlds)]
        for w in worlds:
            w.checkpoint()
        self._drain()
        for w in worlds:
            w.pause(False)
        self._drain()
        for i, w in enumerate(worlds):
            refused_now = w.malformed_dropped - refused[i]
            if refused_now > 0:
                raise EnvError(f"world {i}: {refused_now} command(s) were refused while it was built (is physics enabled, is a recording running, was a spawn malformed?)")
        self.reset()

    @property
    def obs_size(self) -> int:
        return self.spec.obs_size

    @property
    def action_size(self) -> int:
        return self.spec.action_size

    # ----- stepping -------------------------------------------------------------------------------------------------------------

    def _step_batch(self) -> None:
        try:
            self.batch.step()
        except WasmError as err:
            raise EnvError(f"the batch cannot step: {err}") from err
        failed = [i for i, w in enumerate(self._worlds) if w.failure is not None]
        if failed:
            raise EnvError("worlds failed: " + "; ".join(f"{i}: {self._worlds[i].failure}" for i in failed))

    def _drain(self) -> None:
        for _ in range(_MAX_DRAIN_STEPS):
            self._step_batch()
            if all(w.pending_commands == 0 for w in self._worlds):
                return
        raise EnvError(f"the worlds still hold queued commands after {_MAX_DRAIN_STEPS} steps")

    def _begin(self) -> None:
        if self._broken is not None:
            raise EnvError(f"the environment is unusable after an earlier failure: {self._broken}")
        if self._busy:
            raise EnvError("a reset or step is already running")
        self._busy = True

    def _start_episode(self, i: int) -> None:
        world = self._worlds[i]
        self._refused_before[i] = world.malformed_dropped

        def go() -> None:
            world.reset()
            self.spec.reset(world, i, self._ctx[i], mulberry32(episode_seed(self._seed, i, int(self._episode[i]))))
            self._episode[i] += 1

        _at(i, "reset", go)

    def _advance(self) -> None:
        for i, w in enumerate(self._worlds):
            if w.deferred_commands > 0:
                raise EnvError(f"world {i} queued more commands than its ring of {w.ring_capacity} slots takes in one step; raise ring_capacity or send fewer commands")
        self._step_batch()
        for i, w in enumerate(self._worlds):
            before = int(self._refused_before[i])
            if before < 0:
                continue
            self._refused_before[i] = -1
            n = w.malformed_dropped - before
            if n > 0:
                raise EnvError(f"world {i}: {n} command(s) were refused while it was reset (is a recording running, or was a hook command malformed?)")

    def reset(self, seed: Optional[int] = None) -> EnvResult:
        """Starts every world's episode again; `seed` replaces the base seed, and either way the episode counters restart so a seed replays."""
        nxt = self._seed if seed is None else _check_seed(seed)
        self._begin()
        try:
            self._seed = nxt
            self._episode[:] = 0
            self._steps[:] = 0
            self._pending_reset[:] = False
            for i in range(self.size):
                self._start_episode(i)
            self._advance()
            self._collect(True)
            return self.result
        except BaseException as err:
            self._broken = str(err)
            raise
        finally:
            self._busy = False

    def step(self, actions) -> EnvResult:
        """`actions` holds `action_size` numbers per world: shape `(size, action_size)` or flat."""
        size, asize = self.size, self.spec.action_size
        try:
            a = np.asarray(actions, dtype=np.float32)
        except (TypeError, ValueError) as err:
            raise ValueError(f"actions must be an array of numbers: {err}") from err
        if a.shape not in ((size, asize), (size * asize,)):
            raise ValueError(f"expected {size * asize} action values as shape ({size}, {asize}) or flat, got shape {a.shape}")
        a = a.reshape(size, asize)
        bad = np.argwhere(~np.isfinite(a))
        if len(bad):
            i, k = (int(v) for v in bad[0])
            raise ValueError(f"action {i * asize + k} (world {i}) is not finite: {a[i, k]}")
        self._begin()
        try:
            for i in range(size):
                if self._pending_reset[i]:
                    self._start_episode(i)
                else:
                    _at(i, "act", lambda i=i: self.spec.act(self._worlds[i], i, self._ctx[i], a[i]))
            self._advance()
            self._collect(False)
            return self.result
        except BaseException as err:
            self._broken = str(err)
            raise
        finally:
            self._busy = False

    def _collect(self, all_reset: bool) -> None:
        spec, r = self.spec, self.result
        for i, world in enumerate(self._worlds):
            row = r.obs[i]
            _at(i, "observe", lambda i=i, world=world, row=row: spec.observe(world, i, self._ctx[i], row))
            bad = np.flatnonzero(~np.isfinite(row))
            if len(bad):
                raise EnvError(f"world {i} observed a non-finite value at index {int(bad[0])}: {row[bad[0]]}")
            if all_reset or self._pending_reset[i]:
                r.reward[i] = 0
                r.terminated[i] = r.truncated[i] = False
                r.resetting[i] = True
                self._steps[i] = 0
                self._pending_reset[i] = False
                continue
            r.resetting[i] = False
            self._steps[i] += 1
            reward = _at(i, "reward", lambda i=i, world=world, row=row: spec.reward(world, i, self._ctx[i], row))
            if not _is_number(reward):
                raise EnvError(f"world {i} gave a reward that is not a number: {reward!r}")
            if not math.isfinite(reward):
                raise EnvError(f"world {i} gave a non-finite reward: {reward}")
            r.reward[i] = reward
            done = _at(i, "terminated", lambda i=i, world=world, row=row: spec.terminated(world, i, self._ctx[i], row))
            if not _is_bool(done):
                raise EnvError(f"world {i} terminated hook must return a bool, got {done!r}")
            done = bool(done)
            r.terminated[i] = done
            r.truncated[i] = (not done) and self._steps[i] >= spec.max_steps
            self._pending_reset[i] = done or bool(r.truncated[i])
