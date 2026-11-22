import numpy as np
import pytest

from dynova import Batch, ParticleGroup, SHAPE_BOX


def one(capacity=32, gravity=True):
    batch = Batch(1, capacity=capacity, ring_capacity=256)
    w = batch.worlds[0]
    if not gravity:
        w.set_gravity(0, 0, 0)
    return batch, w


def test_a_rope_pinned_at_one_end_hangs_from_it_and_one_read_returns_every_particle():
    batch, w = one()
    with batch:
        g = w.create_chain((0, 5, 0), (2, 5, 0), 11, pin_first=True)
        assert g == ParticleGroup(0, 0, 11)
        for _ in range(360):
            assert batch.step() == []
        p = w.read_particles()
        assert p.shape == (11, 4) and p.dtype == np.float32
        assert p[0, :3].tolist() == [0, 5, 0] and abs(p[0, 3] - 0.02) < 1e-6
        assert 2.9 < p[10, 1] < 3.4


def test_cloth_and_soft_block_on_the_floor_settle_where_the_typescript_ones_do():
    batch, w = one()
    with batch:
        w.spawn(0, -0.5, 0, half=(20, 0.5, 20), inv_mass=0)
        w.spawn(0, 0.6, 0, half=(0.4, 0.5, 0.4), inv_mass=0)
        cloth = w.create_cloth((-1, 2.2, -1), (2, 0, 0), (0, 0, 2), 21, 21, radius=0.06)
        block = w.create_soft_block((3, 1, 3), (0.8, 0.8, 0.8), (4, 4, 4), radius=0.13)
        assert (cloth.first, cloth.count, block.first, block.count) == (0, 441, 441, 64)
        for _ in range(600):
            batch.step()
        p = w.read_particles()
        assert p.shape == (505, 4)
        assert 1.1 < p[10 * 21 + 10, 1] < 1.25
        assert 0.1 < p[441:, 1].min() < 0.17


def test_attached_rope_follows_a_body_and_detaching_frees_it_and_a_seek_republishes():
    batch, w = one(gravity=False)
    with batch:
        box = w.spawn(0, 0, 0, inv_mass=1)
        rope = w.create_chain((1, 0, 0), (2, 0, 0), 2)
        w.attach_particle(rope.first, box, (0.5, 0, 0))
        w.force(box, 20, 0, 0)
        for _ in range(30):
            batch.step()
        x = float(w.read_transforms()[box.id][0])
        assert x > 0.5 and abs(w.read_particles()[0, 0] - (x + 0.5)) < 0.1
        w.detach_particle(rope.first)
        w.force(box, -40, 0, 0)
        for _ in range(30):
            batch.step()
        assert w.read_particles()[0, 0] - float(w.read_transforms()[box.id][0]) > 1.5
        assert w.malformed_dropped == 0


def test_water_spreads_and_sand_holds_its_block_on_the_same_floor():
    batch, w = one()
    with batch:
        w.spawn(0, -0.5, 0, half=(20, 0.5, 20), inv_mass=0)
        rope = w.create_chain((0, 5, 0), (1, 5, 0), 11, pin_first=True)
        water = w.create_fluid((-3, 0.05, 0), (6, 6, 6), 0.05)
        sand = w.create_sand((3, 0.05, 0), (6, 6, 6), 0.05, friction=1)
        assert [rope, water, sand] == [ParticleGroup(0, 0, 11), ParticleGroup(1, 11, 216), ParticleGroup(2, 227, 216)]
        for _ in range(300):
            batch.step()
        p = w.read_particles()
        width = lambda g: float(np.ptp(p[g.first : g.first + g.count, 0]))
        assert abs(p[water.first, 3] - 0.025) < 1e-6
        assert width(water) > 0.6
        assert width(sand) < 0.5
        assert w.malformed_dropped == 0


def test_seek_before_the_rope_reads_no_particles_and_after_it_the_same_rows():
    batch, w = one()
    with batch:
        w.record(True)
        batch.step()
        for t in range(40):
            if t == 20:
                w.create_chain((0, 5, 0), (1, 5, 0), 5, pin_first=True)
            batch.step()
        end = w.read_particles().copy()
        assert end.shape == (5, 4)
        w.seek(10)
        batch.step()
        assert w.read_particles().shape == (0, 4)
        w.seek(41)
        batch.step()
        assert np.array_equal(w.read_particles(), end)


@pytest.mark.parametrize("call,message", [
    (lambda w: w.create_chain((0, 1, 0), (1, 1, 0), 1), "2..1024"),
    (lambda w: w.create_chain((0, 1, 0), (0, 1, 0), 3), "apart"),
    (lambda w: w.create_chain((0, 1, 0), (1, 1, 0), 3, radius=0), "radius"),
    (lambda w: w.create_chain((0, 1, 0), (1, 1, 0), 3, inv_mass=-1), "inv_mass"),
    (lambda w: w.create_chain((0, 1, 0), (1, 1, 0), 3, stiffness=0), "stiffness"),
    (lambda w: w.create_chain((float("nan"), 1, 0), (1, 1, 0), 3), "finite"),
    (lambda w: w.create_cloth((0, 0, 0), (1, 0, 0), (0, 0, 1), 65, 4), "2..64"),
    (lambda w: w.create_cloth((0, 0, 0), (1, 0, 0), (2, 0, 0), 4, 4), "span an area"),
    (lambda w: w.create_cloth((0, 0, 0), (1, 0, 0), (0, 0, 1), 4, 4, pins=32), "mask"),
    (lambda w: w.create_cloth((0, 0, 0), (1, 0, 0), (0, 0, 1), 4, 4, bend_stiffness=2), "bend_stiffness"),
    (lambda w: w.create_soft_block((0, 0, 0), (1, 1, 1), (13, 3, 3)), "2..12"),
    (lambda w: w.create_soft_block((0, 0, 0), (0, 1, 1), (3, 3, 3)), "size"),
    (lambda w: w.create_fluid((0, 0, 0), (33, 4, 4), 0.05), "2..32"),
    (lambda w: w.create_fluid((0, 0, 0), (1, 4, 4), 0.05), "2..32"),
    (lambda w: w.create_fluid((0, 0, 0), (4, 4), 0.05), "2..32"),
    (lambda w: w.create_fluid((0, 0, 0), (4, 4, 4), 0.009), "spacing"),
    (lambda w: w.create_sand((0, 0, 0), (4, 4, 4), 1.5), "spacing"),
    (lambda w: w.create_fluid((0, 0, 0), (4, 4, 4), float("nan")), "spacing"),
    (lambda w: w.create_fluid((0, 1e7, 0), (4, 4, 4), 0.05), "origin"),
    (lambda w: w.create_fluid((0, 0, 0), (4, 4, 4), 0.05, viscosity=1.5), "viscosity"),
    (lambda w: w.create_sand((0, 0, 0), (4, 4, 4), 0.05, friction=-0.1), "friction"),
    (lambda w: w.create_sand((0, 0, 0), (4, 4, 4), 0.05, friction=float("nan")), "friction"),
    (lambda w: w.attach_particle(-1, w.spawn(), (0, 0, 0)), "index"),
    (lambda w: w.attach_particle(0, w.spawn(), (float("nan"), 0, 0)), "offset"),
    (lambda w: w.detach_particle(1.5), "index"),
])
def test_bad_requests_are_refused_with_a_message_before_anything_is_queued(call, message):
    batch, w = one()
    with batch:
        base = w.pending_commands
        with pytest.raises(ValueError, match=message):
            call(w)
        spawned = 1 if "spawn" in str(call.__code__.co_names) else 0
        assert w.pending_commands == base + spawned, "a refused request queued a particle command"


def test_the_particle_budget_is_checked_on_the_host():
    batch, w = one()
    with batch:
        for _ in range(16):
            w.create_soft_block((0, 0, 0), (1, 1, 1), (12, 12, 7))
        with pytest.raises(ValueError, match="at most 16384"):
            w.create_soft_block((0, 0, 0), (1, 1, 1), (12, 12, 12))


def test_fluid_and_sand_share_one_cap_checked_on_the_host():
    batch, w = one()
    with batch:
        assert w.create_fluid((0, 0, 0), (32, 32, 2), 0.05).count == 2048
        assert w.create_sand((0, 0, 0), (32, 32, 2), 0.05).first == 2048
        base = w.pending_commands
        for make in (w.create_fluid, w.create_sand):
            with pytest.raises(ValueError, match="at most 4096 fluid and sand"):
                make((0, 0, 0), (2, 2, 2), 0.05)
        assert w.pending_commands == base
        assert w.create_chain((0, 0, 0), (1, 0, 0), 3).first == 4096


def test_lattice_limits_are_judged_on_the_f32_values():
    batch, w = one()
    with batch:
        w.create_fluid((0, 0, 0), (2, 2, 2), 0.0099999999)
        w.create_fluid((0, 0, 0), (2, 2, 2), 1.0000000001)
        w.create_fluid((0, 0, 0), (2, 2, 2), 0.05, viscosity=1.0000000001)
        w.create_sand((0, 0, 0), (2, 2, 2), 0.05, friction=1.0000000001)
        for bad in (0.00999, 1.0001):
            with pytest.raises(ValueError, match="spacing"):
                w.create_fluid((0, 0, 0), (2, 2, 2), bad)
        for _ in range(2):
            batch.step()
        assert w.malformed_dropped == 0


def test_the_viscosity_and_friction_given_reach_the_ring_slot_not_a_default():
    batch, w = one()
    with batch:
        sent = []
        send = w._send
        w._send = lambda slot: (sent.append(slot), send(slot))[1]
        w.create_sand((0, 0, 0), (2, 2, 2), 0.05, friction=0.3)
        w.create_fluid((0, 0, 0), (2, 2, 2), 0.05, viscosity=0.7)
        import struct
        assert [(s[0], struct.unpack_from("<f", s, 28)[0]) for s in sent] == [(26, pytest.approx(0.3, abs=1e-6)), (25, pytest.approx(0.7, abs=1e-6))]


def test_numpy_integers_are_accepted_wherever_a_count_is_and_a_float_count_is_not():
    batch = Batch(np.int64(1), capacity=np.int32(32), ring_capacity=np.int64(256))
    w = batch.worlds[0]
    with batch:
        rope = w.create_chain((0, 5, 0), (2, 5, 0), np.int64(6))
        sheet = w.create_cloth((0, 5, 1), (1, 0, 0), (0, 0, 1), np.int32(3), np.int64(3))
        water = w.create_fluid((0, 1, 0), (np.int64(2), np.int64(2), np.int64(2)), 0.1)
        assert (rope.count, sheet.count, water.count) == (6, 9, 8)
        assert batch.step() == []
        assert w.read_particles().shape == (23, 4)
        with pytest.raises(ValueError):
            w.create_chain((0, 5, 0), (2, 5, 0), 6.0)


def test_a_reset_or_a_seek_back_past_a_group_gives_its_particles_and_fluid_cap_back():
    batch, w = one()
    with batch:
        w.checkpoint()
        batch.step()
        w.create_fluid((0, 0, 0), (32, 32, 2), 0.05)
        assert w.create_sand((0, 0, 0), (32, 32, 2), 0.05).first == 2048
        batch.step()
        assert w.read_particles().shape == (4096, 4)
        w.reset()
        batch.step()
        assert w.read_particles().shape == (0, 4)
        assert w.create_fluid((0, 0, 0), (32, 32, 2), 0.05).first == 0
        batch.step()
        assert w.read_particles().shape == (2048, 4)

    batch, w = one()
    with batch:
        w.checkpoint()
        batch.step()
        w.create_chain((0, 5, 0), (1, 5, 0), 100)
        batch.step()
        w.reset()
        w.create_fluid((0, 0, 0), (32, 32, 2), 0.05)
        batch.step()
        assert w.read_particles().shape == (2048, 4)
        assert w.create_sand((0, 0, 0), (32, 32, 2), 0.05).first == 2048
        with pytest.raises(ValueError, match="at most 4096 fluid and sand"):
            w.create_fluid((0, 0, 0), (2, 2, 2), 0.05)

    batch, w = one()
    with batch:
        w.create_fluid((0, 0, 0), (32, 32, 2), 0.05)
        w.checkpoint()
        batch.step()
        w.create_sand((0, 0, 0), (32, 32, 2), 0.05)
        batch.step()
        w.reset()
        batch.step()
        assert w.create_sand((0, 0, 0), (32, 32, 2), 0.05).first == 2048
        with pytest.raises(ValueError, match="at most 4096 fluid and sand"):
            w.create_fluid((0, 0, 0), (2, 2, 2), 0.05)

    batch, w = one()
    with batch:
        w.record(True)
        batch.step()
        for t in range(30):
            if t == 20:
                w.create_chain((0, 5, 0), (1, 5, 0), 5)
            batch.step()
        w.seek(10)
        batch.step()
        assert w.create_chain((0, 3, 0), (1, 3, 0), 3).first == 0
