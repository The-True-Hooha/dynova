import numpy as np
import pytest

from dynova import (Batch, JOINT_BALL, JOINT_DISTANCE, JOINT_HINGE, RingFull, SHAPE_SPHERE, StaleHandle, WorldFailed)
from dynova import _wire as w
from helpers import GOLD, bits, drop_scene, run


def one(capacity=16, ring=64):
    return Batch(1, capacity=capacity, ring_capacity=ring)


def test_a_dropped_ball_ends_bit_identical_to_the_typescript_run():
    with one() as b:
        drop_scene(b.worlds[0])
        run(b, 40)
        got = bits(b.worlds[0].read_transforms()[:2])
    assert ",".join(map(str, got)) == GOLD["run_bits"]


def test_handles_count_up_and_a_destroyed_one_is_stale_and_its_slot_comes_back_with_the_next_generation():
    with one() as b:
        world = b.worlds[0]
        a, c = world.spawn(), world.spawn()
        assert (a, c) == ((0, 0), (1, 0))
        world.destroy(a)
        with pytest.raises(StaleHandle):
            world.force(a, 1, 0, 0)
        with pytest.raises(StaleHandle):
            world.destroy(a)
        again = world.spawn()
        assert again == (0, 1)
        run(b, 2)
        assert world.malformed_dropped == 0 and world.stale_dropped == 0


def test_a_generation_wraps_from_255_to_0_without_the_runtime_dropping_a_command():
    with one() as b:
        world = b.worlds[0]
        seen = []
        for _ in range(258):
            h = world.spawn()
            seen.append(h.gen)
            world.destroy(h)
            run(b, 1)
        assert seen[:3] == [0, 1, 2] and seen[255:258] == [255, 0, 1]
        assert (world.malformed_dropped, world.stale_dropped) == (0, 0)


def test_a_destroyed_slot_reads_as_an_empty_row_and_a_copy_does_not_follow_the_world():
    with one() as b:
        world = b.worlds[0]
        h = world.spawn(2, 3, 4)
        run(b, 1)
        before = world.read_transforms(copy=True)
        assert before[h.id, 6] != 0
        world.destroy(h)
        run(b, 1)
        assert world.read_transforms()[h.id, 6] == 0
        assert before[h.id, 6] != 0
        assert before.shape == (16, 7) and before.dtype == np.float32


def test_teleport_force_and_gravity_act_on_the_body():
    with one() as b:
        world = b.worlds[0]
        world.set_gravity(0, 0, 0)
        h = world.spawn(0, 0, 0)
        world.teleport(h, 5, 6, 7)
        run(b, 1)
        assert tuple(world.read_transforms()[h.id, :3]) == (5.0, 6.0, 7.0)
        world.force(h, 30, 0, 0)
        run(b, 10)
        assert world.read_transforms()[h.id, 0] > 5.3
        world.set_gravity(0, -9.81, 0)
        y = world.read_transforms()[h.id, 1]
        run(b, 10)
        assert world.read_transforms()[h.id, 1] < y


@pytest.mark.parametrize("call, error", [
    (lambda w, h: w.teleport(h, float("nan"), 0, 0), ValueError),
    (lambda w, h: w.teleport(h, 1e7, 0, 0), ValueError),
    (lambda w, h: w.set_gravity(0, -5000, 0), ValueError),
    (lambda w, h: w.set_gravity(float("inf"), 0, 0), ValueError),
    (lambda w, h: w.seek(-1), ValueError),
    (lambda w, h: w.seek(2 ** 32), ValueError),
    (lambda w, h: w.force(h._replace(gen=9), 1, 0, 0), StaleHandle),
    (lambda w, h: w.force(h._replace(id=99), 1, 0, 0), StaleHandle),
    (lambda w, h: w.spawn(shape=3), NotImplementedError),
    (lambda w, h: w.spawn(inv_mass=1e39), ValueError),
    (lambda w, h: w.spawn(x=float('nan')), ValueError),
    (lambda w, h: w.spawn(half=(0.5, float('inf'), 0.5)), ValueError),
    (lambda w, h: w.spawn(rotation=(0, 0, 1)), ValueError),
    (lambda w, h: w.force(h, 1e39, 0, 0), ValueError),
    (lambda w, h: w.teleport(h, 1e39, 0, 0), ValueError),
])
def test_hostile_calls_raise_and_change_nothing(call, error):
    with one() as b:
        world = b.worlds[0]
        h = world.spawn()
        run(b, 1)
        before, pending = world.read_transforms(copy=True), world.pending_commands
        with pytest.raises(error):
            call(world, h)
        assert world.pending_commands == pending
        run(b, 1)
        assert world.malformed_dropped == 0
        assert world.spawn().id == 1, "a failed spawn gave its slot back, so no id was skipped"
        assert before.shape == world.read_transforms().shape


def test_a_full_ring_defers_commands_and_delivers_them_in_order_on_later_steps():
    with Batch(1, capacity=32, ring_capacity=4) as b:
        world = b.worlds[0]
        handles = [world.spawn(i, 5, 0) for i in range(10)]
        assert world.deferred_commands == 6 and world.pending_commands == 10
        steps = 0
        while world.pending_commands:
            run(b, 1)
            steps += 1
            assert steps < 10
        assert steps == 3
        assert [round(float(world.read_transforms()[h.id, 0])) for h in handles] == list(range(10))


def test_a_ring_and_deferred_queue_that_are_both_full_raise_and_release_the_id():
    with Batch(1, capacity=32, ring_capacity=2) as b:
        world = b.worlds[0]
        world._max_deferred = 1
        world.spawn(); world.spawn(); world.spawn()
        with pytest.raises(RingFull):
            world.spawn()
        run(b, 3)
        assert world.spawn().id == 3, "the refused spawn did not consume id 3"


def test_spawning_past_the_capacity_raises_and_a_grow_command_makes_room():
    with Batch(1, capacity=4, ring_capacity=16) as b:
        world = b.worlds[0]
        for _ in range(4):
            world.spawn()
        with pytest.raises(RingFull, match="capacity exhausted"):
            world.spawn()
        world._send(w.grow(12))
        run(b, 1)
        assert world.capacity == 12 and world.read_transforms().shape == (12, 7)
        assert world.spawn().id == 4
        run(b, 1)
        assert world.read_transforms()[4, 6] != 0


def test_a_paused_world_keeps_its_tick_and_still_takes_commands():
    with one() as b:
        world = b.worlds[0]
        h = world.spawn(0, 0, 0)
        run(b, 2)
        world.pause(True)
        run(b, 1)
        tick = world.ticks
        world.teleport(h, 9, 9, 9)
        run(b, 3)
        assert world.ticks == tick
        assert tuple(world.read_transforms()[h.id, :3]) == (9.0, 9.0, 9.0)


def test_a_seek_returns_to_a_recorded_tick_and_the_handles_follow():
    with one() as b:
        world = b.worlds[0]
        world.record(True)
        run(b, 1)
        h = drop_scene(world)
        run(b, 10)
        at_ten = world.read_transforms(copy=True)
        t10 = world.ticks
        late = world.spawn(5, 5, 5)
        run(b, 10)
        world.pause(True)
        world.seek(t10)
        run(b, 1)
        assert bits(world.read_transforms()) == bits(at_ten)
        assert world.is_live(h) and not world.is_live(late)


def test_a_reset_returns_to_the_checkpoint_and_the_ids_follow():
    with one() as b:
        world = b.worlds[0]
        ball = drop_scene(world)
        world.checkpoint()
        run(b, 1)
        start = None
        world.reset()
        run(b, 1)
        start = world.read_transforms(copy=True)
        run(b, 30)
        mid = world.spawn(3, 3, 3)
        run(b, 1)
        assert not np.array_equal(world.read_transforms(), start)
        world.reset()
        run(b, 1)
        assert bits(world.read_transforms()) == bits(start)
        assert world.is_live(ball) and not world.is_live(mid)
        assert world.spawn().id == mid.id
        assert world.malformed_dropped == 0


def test_a_reset_with_no_checkpoint_is_counted_and_the_world_goes_on():
    with one() as b:
        world = b.worlds[0]
        drop_scene(world)
        run(b, 1)
        tick = world.ticks
        world.reset()
        run(b, 1)
        assert world.malformed_dropped == 1 and world.ticks == tick + 1


def test_a_hinge_with_a_motor_turns_and_reports_its_state_until_it_is_destroyed():
    with one() as b:
        world = b.worlds[0]
        world.set_gravity(0, 0, 0)
        pivot = world.spawn(0, 0, 0, half=(0.1, 0.1, 0.1), inv_mass=0)
        link = world.spawn(0.5, 0, 0, half=(0.5, 0.05, 0.1))
        joint = world.create_joint(JOINT_HINGE, pivot, link, (0, 0, 0), (0, 0, 1), limits=(-2.5, 2.5), motor=(0.0, 20.0))
        run(b, 1)
        assert joint.id == 0
        world.set_joint_motor(joint, 1.5, 20)
        run(b, 30)
        rows = world.read_joint_states()
        assert rows.dtype.names == ("joint", "kind", "position", "velocity") and len(rows) == 1
        assert (int(rows[0]["joint"]), int(rows[0]["kind"])) == (0, JOINT_HINGE)
        assert rows[0]["position"] > 0.3 and rows[0]["velocity"] > 0.5
        world.destroy_joint(joint)
        run(b, 1)
        assert len(world.read_joint_states()) == 0
        assert world.malformed_dropped == 0


@pytest.mark.parametrize("kwargs, message", [
    (dict(kind=9, axis=(0, 0, 1)), "unknown joint kind"),
    (dict(kind=JOINT_HINGE, axis=None), "non-zero axis"),
    (dict(kind=JOINT_HINGE, axis=(0, 0, 0)), "non-zero axis"),
    (dict(kind=JOINT_HINGE, axis=(0, 0, 1), limits=(0.5, 1.0)), "lo <= 0 <= hi"),
    (dict(kind=JOINT_HINGE, axis=(0, 0, 1), limits=(-4.0, 1.0)), "lo <= 0 <= hi"),
    (dict(kind=JOINT_BALL, motor=(1.0, 1.0)), "only hinge and slider"),
    (dict(kind=JOINT_BALL, limits=(-1.0, 1.0)), "only hinge, slider and distance"),
    (dict(kind=JOINT_BALL, anchor_b=(1, 1, 1)), "only a distance joint"),
    (dict(kind=JOINT_DISTANCE, axis=(0, 0, 1), anchor_b=(1, 0, 0)), "not an axis"),
    (dict(kind=JOINT_DISTANCE), "second anchor"),
    (dict(kind=JOINT_DISTANCE, anchor_b=(0.5001, 0, 0)), "1 mm apart"),
    (dict(kind=JOINT_HINGE, axis=(0, 0, 1), motor=(5000.0, 1.0)), "motor velocity"),
    (dict(kind=JOINT_HINGE, axis=(0, 0, 1), motor=(1.0, -1.0)), "max_force"),
    (dict(kind=JOINT_BALL, anchor=(float("nan"), 0, 0)), "anchor"),
    (dict(kind=JOINT_HINGE, axis=(1e30, 0, 0)), "non-zero axis"),
    (dict(kind=JOINT_HINGE, axis=(1e39, 0, 0)), "axis"),
    (dict(kind=JOINT_BALL, anchor=(1e39, 0, 0)), "anchor"),
])
def test_a_bad_joint_is_refused_before_it_reaches_the_ring(kwargs, message):
    with one() as b:
        world = b.worlds[0]
        a, c = world.spawn(), world.spawn(1, 0, 0)
        kwargs = dict(kwargs)
        kind = kwargs.pop("kind")
        anchor = kwargs.pop("anchor", (0.5, 0, 0))
        with pytest.raises(ValueError, match=message):
            world.create_joint(kind, a, c, anchor, **kwargs)
        pending = world.pending_commands
        run(b, 1)
        assert world.malformed_dropped == 0 and pending == 2


def test_a_joint_needs_two_different_live_bodies():
    with one() as b:
        world = b.worlds[0]
        a = world.spawn()
        with pytest.raises(ValueError, match="two different bodies"):
            world.create_joint(JOINT_BALL, a, a, (0, 0, 0))
        with pytest.raises(StaleHandle):
            world.create_joint(JOINT_BALL, a, a._replace(id=5), (0, 0, 0))


def test_a_closed_batch_fails_its_worlds_with_a_message():
    b = one()
    world = b.worlds[0]
    b.close()
    b.close()
    with pytest.raises(WorldFailed, match="closed"):
        world.spawn()


def test_a_view_of_the_transforms_keeps_its_instance_alive_until_the_view_is_gone():
    import gc
    import weakref
    b = one()
    world = b.worlds[0]
    drop_scene(world)
    run(b, 3)
    view = world.read_transforms()
    keep = world.read_transforms()[:2]
    owner = weakref.ref(world._inst)
    b.close()
    del b, world
    gc.collect()
    assert owner() is not None and view[0, 6] == 1.0, "a view that outlives its batch still has memory behind it"
    del view, keep
    gc.collect()
    assert owner() is None, "and the instance is released once no view is left"


def test_a_world_refuses_to_be_touched_while_its_batch_steps():
    with Batch(2, capacity=8, ring_capacity=16) as b:
        seen = []
        real = b._groups[0].batch.step_all

        def step_and_poke():
            for call in (lambda: b.worlds[0].spawn(), lambda: b.worlds[1].read_transforms()):
                try:
                    call()
                except RuntimeError as err:
                    seen.append(str(err))
            real()

        b._groups[0].batch.step_all = step_and_poke
        assert b.step() == []
        assert len(seen) == 2 and all("stepping" in m for m in seen)
        b.worlds[0].spawn()
