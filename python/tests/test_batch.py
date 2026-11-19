import struct

import pytest

from dynova import Batch, WasmError, WasmTrap, WorldFailed, partition
from dynova._wasm import Instance
from helpers import bits, drop_scene, run

# Pinned in core/src/determinism.rs and docs/decisions/determinism.md: native, Chromium and WebKit produce these.
SCENE = [17533006936851158103, 7396268202019089366, 7396268202019089366, 17533006936851158103]
JOINT_SCENE = [15465293547640079592, 15165457805995745632, 15165457805995745632, 15465293547640079592]


def scene_of(instance: Instance, export: str):
    pointer, length = instance.call(export)
    values = list(struct.unpack(f"<{length}Q", bytes(instance.mem()[pointer : pointer + 8 * length])))
    instance.call("__wbindgen_free", pointer, length * 8, 8)
    return values


def test_the_wasm_this_package_runs_reproduces_the_pinned_hashes_of_the_native_and_browser_builds():
    instance = Instance()
    assert scene_of(instance, "determinism_scene") == SCENE
    assert scene_of(instance, "determinism_joint_scene") == JOINT_SCENE


def test_partition_shares_worlds_out_contiguously_and_evenly():
    assert partition(10, 4) == [3, 3, 2, 2]
    assert partition(3, 8) == [1, 1, 1]
    assert partition(1, 1) == [1]
    counts = partition(256, 7)
    assert sum(counts) == 256 and max(counts) - min(counts) <= 1


@pytest.mark.parametrize("bad", [0, -1, 1.5, None, "x"])
def test_partition_rejects_counts_that_are_not_positive_integers(bad):
    with pytest.raises(ValueError):
        partition(bad, 2)
    with pytest.raises(ValueError):
        partition(2, bad)


def scenes(batch):
    for i, world in enumerate(batch.worlds):
        drop_scene(world, 3.0 + i)


def final_bits(workers: int, worlds: int = 7, ticks: int = 60):
    with Batch(worlds, capacity=16, ring_capacity=64, workers=workers) as b:
        scenes(b)
        run(b, ticks)
        return [bits(w.read_transforms()[:2]) for w in b.worlds]


def test_every_world_ends_as_the_same_world_run_alone_whatever_the_number_of_workers():
    alone = []
    for i in range(7):
        with Batch(1, capacity=16, ring_capacity=64) as b:
            drop_scene(b.worlds[0], 3.0 + i)
            run(b, 60)
            alone.append(bits(b.worlds[0].read_transforms()[:2]))
    assert len({tuple(a) for a in alone}) == 7, "the worlds differ, so a mix-up between them would show"
    for workers in (1, 2, 3, 5, 7, 12):
        assert final_bits(workers) == alone, f"workers = {workers}"


def test_workers_run_on_real_threads_one_instance_each():
    with Batch(6, capacity=8, ring_capacity=16, workers=3) as b:
        assert b.workers == 3 and len(b) == 6
        instances = {id(w._inst) for w in b.worlds}
        assert len(instances) == 3


def test_a_command_into_one_world_changes_that_world_only():
    with Batch(3, capacity=16, ring_capacity=64, workers=2) as b:
        scenes(b)
        b.worlds[1].force(__import__("dynova").Handle(1, 0), 60, 0, 0)
        run(b, 20)
        xs = [float(w.read_transforms()[1, 0]) for w in b.worlds]
        assert xs[0] == xs[2] == 0.0 and xs[1] > 0.5


def test_a_worker_that_traps_fails_its_worlds_only_and_the_others_keep_stepping():
    with Batch(6, capacity=16, ring_capacity=64, workers=3) as b:
        scenes(b)
        def trap():
            raise WasmTrap("unreachable")
        b._groups[1].batch.step_all = trap
        assert b.step() == [2, 3]
        assert b.step() == [2, 3]
        for i, world in enumerate(b.worlds):
            if i in (2, 3):
                assert "unreachable" in world.failure
                with pytest.raises(WorldFailed, match="worker failed"):
                    world.spawn()
            else:
                assert world.failure is None
                assert world.ticks == 2


def test_a_real_wasm_trap_poisons_that_instance_and_is_handled_like_any_other_failure():
    with Batch(4, capacity=16, ring_capacity=64, workers=2) as b:
        scenes(b)
        instance = b._groups[0].batch.instance
        with pytest.raises(WasmTrap):
            instance.call("batch_step_world", 1 << 30, 0)
        with pytest.raises(WasmTrap, match="already failed"):
            instance.call("batch_step_all", 0)
        assert b.step() == [0, 1]
        assert b.worlds[2].ticks == 1 and b.worlds[0].failure is not None


def test_step_raises_once_no_worker_is_left():
    with Batch(2, capacity=8, ring_capacity=16, workers=2) as b:
        for g in b._groups:
            g.batch.step_all = lambda: (_ for _ in ()).throw(WasmTrap("gone"))
        assert b.step() == [0, 1]
        with pytest.raises(WasmError, match="every worker has failed"):
            b.step()


@pytest.mark.parametrize("kwargs, error, message", [
    (dict(worlds=0), ValueError, "worlds"),
    (dict(worlds=1, workers=0), ValueError, "workers"),
    (dict(worlds=1, capacity=0), WasmError, "capacity"),
    (dict(worlds=1, ring_capacity=12), WasmError, "power of two"),
    (dict(worlds=1, ring_capacity=0), WasmError, "power of two"),
    (dict(worlds=100_000, capacity=1, ring_capacity=1), WasmError, "MiB"),
    (dict(worlds=1, capacity=2 ** 32), ValueError, "capacity"),
    (dict(worlds=1, capacity=-1), ValueError, "capacity"),
])
def test_hostile_sizes_are_refused_with_a_reason(kwargs, error, message):
    with pytest.raises(error, match=message):
        Batch(**kwargs)


def test_a_refused_batch_leaves_the_module_usable_for_the_next_one():
    for _ in range(3):
        with pytest.raises(WasmError):
            Batch(100_000, capacity=1, ring_capacity=1)
    with Batch(2, capacity=8, ring_capacity=16) as b:
        run(b, 2)
        assert [w.ticks for w in b.worlds] == [2, 2]


def test_a_stale_wasm_header_size_fails_loudly(monkeypatch):
    from dynova import world
    with Batch(1, capacity=8, ring_capacity=16) as b:
        layout = [0, 999, 0, 0, 0, 0, 16]
        with pytest.raises(RuntimeError, match="999 words"):
            world.World(b.worlds[0]._inst, layout, 8)


def test_close_is_idempotent_and_stops_the_pool():
    b = Batch(4, capacity=8, ring_capacity=16, workers=2)
    pool = b._pool
    b.close()
    b.close()
    assert b._pool is None and pool._shutdown
    with pytest.raises(WorldFailed, match="closed"):
        b.worlds[0].spawn()


def test_a_stepped_closed_batch_is_refused_not_run_on_freed_memory():
    b = Batch(2, capacity=8, ring_capacity=16, workers=2)
    b.close()
    with pytest.raises(WasmError, match="closed"):
        b.step()
    group = b._groups[0].batch
    for call in (group.step_all, lambda: group.step_world(0), lambda: group.world_layout(0), lambda: group.world_count):
        with pytest.raises(WasmError, match="freed"):
            call()


def test_a_rust_panic_reports_its_message_without_the_garbage_a_wrong_stack_hook_would_add():
    from dynova._wasm import WasmBatch
    batch = WasmBatch(1, 8, 16)
    batch.free()
    with pytest.raises(WasmTrap) as err:
        batch.instance.call("batch_step_all", batch._pointer)
    text = str(err.value)
    assert text.startswith("panicked at") and len(text) < 2000


def test_a_trap_does_not_report_the_message_of_an_earlier_refused_request():
    instance = Instance()
    _, error, failed = instance.call("batch_new", 0, 8, 16)
    assert failed, "a batch of no worlds is refused, which leaves its message with the instance"
    with pytest.raises(WasmTrap) as err:
        instance.call("batch_step_world", 1 << 30, 0)
    assert "at least one world" not in str(err.value)
    assert instance.take_message("none") == "none"


def test_instances_of_a_batch_step_at_the_same_time():
    import threading
    with Batch(4, capacity=8, ring_capacity=16, workers=4) as b:
        barrier = threading.Barrier(4, timeout=10)
        for g in b._groups:
            real = g.batch.step_all
            g.batch.step_all = lambda real=real: (barrier.wait(), real())[1]
        assert b.step() == [], "four steps that wait for one another can only finish if they run on four threads"


def test_a_failure_while_finishing_a_group_fails_that_group_not_the_step():
    with Batch(4, capacity=8, ring_capacity=16, workers=2) as b:
        def broken():
            raise RuntimeError("after_step broke")
        b._groups[1].worlds[0].after_step = broken
        assert b.step() == [2, 3]
        assert "after_step broke" in b.worlds[2].failure
        assert b.worlds[0].failure is None


def test_a_constructor_that_fails_part_way_frees_every_wasm_batch_it_made(monkeypatch):
    import dynova.batch as batch_module
    made = []

    class Tracked(batch_module.WasmBatch):
        def __init__(self, *args):
            super().__init__(*args)
            self.freed_count = 0
            made.append(self)

        def free(self):
            self.freed_count += 1
            super().free()

    monkeypatch.setattr(batch_module, "WasmBatch", Tracked)
    real_world = batch_module.World
    calls = []

    def world(*args):
        calls.append(1)
        if len(calls) == 3:
            raise RuntimeError("third world refused")
        return real_world(*args)

    monkeypatch.setattr(batch_module, "World", world)
    with pytest.raises(RuntimeError, match="third world"):
        Batch(4, capacity=8, ring_capacity=16, workers=2)
    assert len(made) == 2 and [m.freed_count for m in made] == [1, 1]


def test_a_source_checkouts_build_is_preferred_to_the_copy_a_package_build_left_behind(monkeypatch):
    from dynova import _wasm
    monkeypatch.delenv("DYNOVA_WASM", raising=False)
    order = [p.parent.name if p.parent.name != "_data" else "package" for p in _wasm._candidates()]
    assert order == ["wasm-st", "package"]
    monkeypatch.setenv("DYNOVA_WASM", "/nonexistent/x.wasm")
    assert str(_wasm._candidates()[0]) == "/nonexistent/x.wasm" and len(_wasm._candidates()) == 3
