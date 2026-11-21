import numpy as np
import pytest

from dynova import SHAPE_SPHERE, Batch, Camera, NO_BODY, WasmError, id_colors, look_at
from helpers import bits

FACING = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
WALL = dict(half=(50, 50, 0.5), inv_mass=0)


def walls(batch):
    return [w.spawn(0, 0, -(i + 3.5), **WALL) for i, w in enumerate(batch.worlds)]


def test_a_flat_wall_reads_its_distance_at_every_pixel_of_every_world_and_ids_are_entity_ids():
    with Batch(3, capacity=8) as batch:
        handles = walls(batch)
        assert batch.step() == []
        images = batch.render(Camera.from_fov(32, 24, np.pi / 3), [FACING] * 3)
        assert images.depth.shape == images.id.shape == (3, 24, 32)
        assert images.depth.dtype == np.float32 and images.id.dtype == np.uint32
        for i in range(3):
            assert np.abs(images.depth[i] - (i + 3)).max() < 1e-3, f"world {i}"
            assert set(np.unique(images.id[i])) == {handles[i].id}


def test_a_sphere_reads_the_analytic_distance_and_row_zero_is_the_top():
    with Batch(1, capacity=8) as batch:
        w = batch.worlds[0]
        w.spawn(0, 0, -20, **WALL)
        ball = w.spawn(0, 0, -6, shape=SHAPE_SPHERE, half=(1, 1, 1), inv_mass=0)
        w.spawn(0, 3.5, -10, half=(1, 1, 1), inv_mass=0)
        batch.step()
        images = batch.render(Camera.from_fov(33, 33, np.pi / 3), [FACING])
        assert 4.999 < images.depth[0, 16, 16] < 5.0001
        assert images.id[0, 16, 16] == ball.id
        top = images.id[0, 4, 16]
        assert top != ball.id and top != NO_BODY and images.depth[0, 4, 16] < 10, "the box above the axis appears in the upper rows"
        assert images.id[0, 28, 16] != top, "and not in the lower ones"


def test_every_worker_count_gives_the_same_bits():
    def images(workers):
        with Batch(5, capacity=8, workers=workers) as batch:
            for i, w in enumerate(batch.worlds):
                w.spawn(0, 0, -(i + 3.5), **WALL)
                w.spawn(0.3 * i, 0.2, -2, shape=SHAPE_SPHERE, half=(0.4, 0.4, 0.4), inv_mass=0)
            batch.step()
            poses = [look_at((0.1 * i, 0.5, 1), (0, 0, -5)) for i in range(5)]
            return batch.render(Camera.from_fov(20, 16, 1.0), poses)

    one = images(1)
    for workers in (2, 5):
        other = images(workers)
        assert bits(other.depth) == bits(one.depth) and np.array_equal(other.id, one.id), f"{workers} workers"
    assert (one.id != NO_BODY).any() and len(np.unique(one.depth)) > 20


def test_matches_the_image_hash_pinned_in_rust_and_typescript():
    with Batch(1, capacity=8) as batch:
        w = batch.worlds[0]
        w.spawn(0.3, 0.2, -5, shape=SHAPE_SPHERE, half=(1, 1, 1), inv_mass=0)
        w.spawn(-2, -1, -7, half=(1, 0.5, 2), inv_mass=0)
        w.spawn(0, -3, -5, half=(40, 0.5, 40), inv_mass=0)
        batch.step()
        images = batch.render(Camera(48, 48, 40.0, 40.0, 24.0, 24.0, 0.0, 100.0), [(0.25, 0.5, 0, 0.0625, 0.125, 0, 1)])
    h = 0xCBF29CE484222325
    for word in list(images.depth.view(np.uint32).ravel()) + list(images.id.ravel()):
        for k in range(8):
            h ^= (int(word) >> (8 * k)) & 0xFF if k < 4 else 0
            h = (h * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    assert h == 10825879982174112534


def test_a_declined_camera_or_pose_raises_and_leaves_the_batch_usable():
    with Batch(2, capacity=8, workers=2) as batch:
        walls(batch)
        batch.step()
        cam = Camera.from_fov(8, 8, 1.0)
        with pytest.raises(WasmError, match="pixels in total"):
            batch.render(Camera.from_fov(4096, 4096, 1.0), [FACING] * 2)
        with pytest.raises(WasmError, match="focal lengths"):
            batch.render(Camera(8, 8, 0.0, 1.0, 4, 4), [FACING] * 2)
        with pytest.raises(WasmError, match="finite position within range"):
            batch.render(cam, [FACING, (1e9, 0, 0, 0, 0, 0, 1)])
        with pytest.raises(WasmError, match="non-zero"):
            batch.render(cam, [FACING, (0, 0, 0, 0, 0, 0, 0)])
        assert batch.failed_worlds() == []
        assert np.abs(batch.render(cam, [FACING] * 2).depth[1] - 4).max() < 1e-3


def test_bad_arguments_are_named():
    with Batch(2, capacity=8) as batch:
        cam = Camera.from_fov(8, 8, 1.0)
        with pytest.raises(ValueError, match=r"shape \(2, 7\), got \(1, 7\)"):
            batch.render(cam, [FACING])
        with pytest.raises(ValueError, match="finite"):
            batch.render(cam, [FACING, (float("nan"), 0, 0, 0, 0, 0, 1)])
        with pytest.raises(ValueError, match="width"):
            batch.render(Camera(0, 8, 1, 1, 0, 0), [FACING] * 2)
        with pytest.raises(ValueError, match="ignore"):
            batch.render(cam, [FACING] * 2, ignore=-1)
        with pytest.raises(ValueError, match="fov_y"):
            Camera.from_fov(8, 8, 0.0)


def test_a_camera_on_a_body_can_ignore_it():
    with Batch(1, capacity=8) as batch:
        w = batch.worlds[0]
        holder = w.spawn(0, 0, 0, half=(1, 1, 1), inv_mass=0)
        w.spawn(0, 0, -5, **WALL)
        batch.step()
        cam = Camera.from_fov(4, 4, 1.0)
        assert set(np.unique(batch.render(cam, [FACING]).id)) == {holder.id}
        seen = batch.render(cam, [FACING], ignore=holder.id)
        assert holder.id not in seen.id and np.abs(seen.depth - 4.5).max() < 1e-3


def test_a_closed_batch_and_a_failed_worker_are_reported():
    batch = Batch(2, capacity=8, workers=2)
    walls(batch)
    batch.step()
    batch._groups[1].batch.instance.failed = "boom"
    images = batch.render(Camera.from_fov(8, 8, 1.0), [FACING] * 2)
    assert batch.failed_worlds() == [1]
    assert np.abs(images.depth[0] - 3).max() < 1e-3
    assert (images.depth[1] == 0).all() and (images.id[1] == NO_BODY).all()
    batch.close()
    with pytest.raises(WasmError, match="closed"):
        batch.render(Camera.from_fov(8, 8, 1.0), [FACING] * 2)


def test_look_at_faces_the_target_and_refuses_degenerate_views():
    x, y, z, qx, qy, qz, qw = look_at((1, 2, 3), (1, 2, -7))
    assert (x, y, z) == (1, 2, 3) and np.allclose((abs(qx), abs(qy), abs(qz), abs(qw)), (0, 0, 0, 1))
    for target, up in [((0, 0, -1), (0, 1, 0)), ((1, 0, 0), (0, 1, 0)), ((0, -1, 0), (0, 0, -1)), ((0, 0, 1), (0, 1, 0)), ((-1, 0, 0), (0, 1, 0)), ((0, 1, 0), (1, 0, 0)), ((1, 1, 1), (0, 1, 0))]:
        assert abs(np.linalg.norm(look_at((0, 0, 0), target, up)[3:]) - 1) < 1e-9
    qx, qy, qz, qw = look_at((0, 0, 0), (5, 0, 0))[3:]
    t = 2 * np.cross((qx, qy, qz), (0, 0, -1))
    direction = np.array((0, 0, -1)) + qw * t + np.cross((qx, qy, qz), t)
    assert np.allclose(direction, (1, 0, 0))
    with pytest.raises(ValueError, match="parallel"):
        look_at((0, 5, 0), (0, 0, 0))
    with pytest.raises(ValueError, match="apart"):
        look_at((1, 1, 1), (1, 1, 1))


def test_id_colors_match_the_typescript_helper_and_leave_no_body_clear():
    rgba = id_colors(np.array([0, 1, 1, NO_BODY], dtype=np.uint32))
    assert rgba.shape == (4, 4) and rgba.dtype == np.uint8
    assert (rgba[1] == rgba[2]).all() and not (rgba[0] == rgba[1]).all()
    assert (rgba[3] == 0).all() and rgba[0, 3] == 255
    # id 0: (0 + 1) * 0x9e3779b1 = 0x9e3779b1, as the TypeScript helper computes it.
    assert tuple(rgba[0, :3]) == (64 + (0xB1 & 0xBF), 64 + (0x79 & 0xBF), 64 + (0x37 & 0xBF))

def test_a_refusal_in_one_group_waits_for_the_others_so_the_next_step_is_safe():
    with Batch(4, capacity=8, workers=2) as batch:
        walls(batch)
        batch.step()
        bad = [FACING] * 3 + [(1e9, 0, 0, 0, 0, 0, 1)]
        with pytest.raises(WasmError):
            batch.render(Camera.from_fov(8, 8, 1.0), bad)
        assert batch.step() == []

