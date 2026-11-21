"""Many worlds stepped together, on one wasm instance or spread over several that run on threads (wasmtime releases the GIL)."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor, wait
from typing import List, NamedTuple, Optional, Sequence

import numpy as np

from ._wasm import WasmBatch, WasmError, WasmTrap
from .camera import NO_BODY, Camera
from .world import World

__all__ = ["Batch", "Images", "partition"]


def partition(worlds: int, workers: int) -> List[int]:
    """How many worlds each worker gets: contiguous, every worker at least one, no two counts more than one apart."""
    for name, value in (("worlds", worlds), ("workers", workers)):
        if not isinstance(value, (int, np.integer)) or value < 1:
            raise ValueError(f"{name} must be a positive integer, got {value!r}")
    n = min(workers, worlds)
    return [worlds // n + (1 if i < worlds % n else 0) for i in range(n)]


class Images(NamedTuple):
    """`depth` is metres along the view axis (0 where nothing is hit), `id` the body each ray hits (`NO_BODY` for none); both `(worlds, height, width)`, row 0 at the top."""

    depth: np.ndarray
    id: np.ndarray


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

    def _render_group(self, group: _Group, camera: Camera, poses: bytes, ignore: int) -> Optional[Images]:
        instance = group.batch.instance
        instance.busy = True
        try:
            group.batch.render_all(camera.width, camera.height, camera.words(), poses, ignore)
            instance.busy = False
            depth, ids = group.batch.images()
            shape = (len(group.worlds), camera.height, camera.width)
            return Images(depth.reshape(shape), ids.reshape(shape))
        except WasmTrap as err:
            self._fail(group, str(err))
            return None
        finally:
            instance.busy = False

    def render(self, camera: Camera, poses, ignore: Optional[int] = None) -> Images:
        """One depth and one id image per world from the poses the last step left, all instances at once. `poses` is `(worlds, 7)`: `x, y, z, qx, qy, qz, qw` (see `look_at`).

        The arrays are copies. The images of a world whose instance has failed are all misses. Raises `WasmError` when the camera or a pose is declined (nothing is then
        rendered and no instance is harmed), and when no instance is left.
        """
        if self._closed:
            raise WasmError("the batch is closed")
        table = np.asarray(poses, dtype=np.float64)
        if table.shape != (len(self.worlds), 7):
            raise ValueError(f"render needs one pose of 7 numbers per world: expected shape ({len(self.worlds)}, 7), got {table.shape}")
        if not np.all(np.isfinite(table)):
            raise ValueError("a camera pose needs a finite position and rotation")
        camera.words()
        live = [g for g in self._groups if g.failed is None]
        if not live:
            raise WasmError("every worker has failed")
        ignore = NO_BODY if ignore is None else ignore
        if not isinstance(ignore, (int, np.integer)) or not 0 <= ignore <= NO_BODY:
            raise ValueError(f"ignore must be a body id or None, got {ignore!r}")
        packed = {id(g): table[g.first : g.first + len(g.worlds)].astype("<f4").tobytes() for g in live}
        if self._pool is None:
            results = [self._render_group(live[0], camera, packed[id(live[0])], ignore)]
        else:
            futures = [self._pool.submit(self._render_group, g, camera, packed[id(g)], ignore) for g in live]
            wait(futures)  # none may still be running when an error is raised, or a later step would share its instance
            results = [f.result() for f in futures]
        shape = (len(self.worlds), camera.height, camera.width)
        depth = np.zeros(shape, dtype=np.float32)
        ids = np.full(shape, NO_BODY, dtype=np.uint32)
        for group, images in zip(live, results):
            if images is not None:
                depth[group.first : group.first + len(group.worlds)] = images.depth
                ids[group.first : group.first + len(group.worlds)] = images.id
        return Images(depth, ids)

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
