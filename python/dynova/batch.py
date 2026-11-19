"""Many worlds stepped together, on one wasm instance or spread over several that run on threads (wasmtime releases the GIL)."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from typing import List, Optional, Sequence

from ._wasm import WasmBatch, WasmError, WasmTrap
from .world import World

__all__ = ["Batch", "partition"]


def partition(worlds: int, workers: int) -> List[int]:
    """How many worlds each worker gets: contiguous, every worker at least one, no two counts more than one apart."""
    for name, value in (("worlds", worlds), ("workers", workers)):
        if not isinstance(value, int) or value < 1:
            raise ValueError(f"{name} must be a positive integer, got {value!r}")
    n = min(workers, worlds)
    return [worlds // n + (1 if i < worlds % n else 0) for i in range(n)]


class _Group:
    def __init__(self, batch: WasmBatch, worlds: List[World], first: int) -> None:
        self.batch, self.worlds, self.first = batch, worlds, first
        self.failed: Optional[str] = None


class Batch:
    """`worlds` independent worlds, each with room for `capacity` bodies and a ring of `ring_capacity` commands.

    Every world ends in exactly the state it would reach alone, whatever `workers` is. A trap in one worker fails only the worlds it holds.
    """

    def __init__(self, worlds: int, capacity: int = 64, ring_capacity: int = 256, workers: int = 1, physics: bool = True) -> None:
        counts = partition(worlds, workers)
        self._groups: List[_Group] = []
        self._pool: Optional[ThreadPoolExecutor] = None
        self._closed = False
        try:
            first = 0
            for count in counts:
                batch = WasmBatch(count, capacity, ring_capacity, physics)
                try:
                    members = []
                    for i in range(count):
                        layout = batch.world_layout(i)
                        if layout is None:
                            raise WasmError(f"the batch has no layout for world {i}")
                        if layout[6] != ring_capacity:
                            raise WasmError(f"world {i} has a ring of {layout[6]} slots but {ring_capacity} were asked for")
                        members.append(World(batch.instance, layout, capacity))
                except BaseException:
                    batch.free()
                    raise
                self._groups.append(_Group(batch, members, first))
                first += count
            if len(self._groups) > 1:
                self._pool = ThreadPoolExecutor(len(self._groups))
        except BaseException:
            self.close()
            raise
        self.worlds: Sequence[World] = [w for g in self._groups for w in g.worlds]

    def __len__(self) -> int:
        return len(self.worlds)

    @property
    def workers(self) -> int:
        return len(self._groups)

    def failed_worlds(self) -> List[int]:
        return [g.first + k for g in self._groups if g.failed is not None for k in range(len(g.worlds))]

    def _fail(self, group: _Group, reason: str) -> None:
        group.failed = reason
        for world in group.worlds:
            world.mark_failed(f"worker failed: {reason}")

    def _step_group(self, group: _Group) -> None:
        instance = group.batch.instance
        instance.busy = True
        try:
            group.batch.step_all()
            instance.busy = False
            for world in group.worlds:
                world.after_step()
        except Exception as err:  # a trap, or anything else that went wrong in this group, fails this group and no other
            instance.busy = False
            self._fail(group, str(err))
        finally:
            instance.busy = False

    def step(self) -> List[int]:
        """Steps every world whose instance is still alive, all instances at once; returns the indices of the failed worlds (none when all is well).

        Raises when no instance is left.
        """
        if self._closed:
            raise WasmError("the batch is closed")
        live = [g for g in self._groups if g.failed is None]
        if not live:
            raise WasmError("every worker has failed")
        if self._pool is None:
            self._step_group(live[0])
        else:
            for future in [self._pool.submit(self._step_group, g) for g in live]:
                future.result()
        return self.failed_worlds()

    def close(self) -> None:
        self._closed = True
        if self._pool is not None:
            self._pool.shutdown(wait=True)
            self._pool = None
        for g in self._groups:
            g.batch.free()
            for world in g.worlds:
                world.mark_failed("batch closed")

    def __enter__(self) -> "Batch":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
