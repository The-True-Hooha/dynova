import json
from pathlib import Path

import numpy as np

from dynova import SHAPE_SPHERE

GOLD = json.loads((Path(__file__).parent / "golden_ts.json").read_text())


def bits(a: np.ndarray) -> list:
    return [int(v) for v in np.ascontiguousarray(a, dtype=np.float32).view(np.uint32).ravel()]


def drop_scene(world, height: float = 3.0):
    """A floor and a ball above it; returns the ball."""
    world.spawn(0, -1, 0, half=(20, 1, 20), inv_mass=0)
    return world.spawn(0, height, 0, shape=SHAPE_SPHERE)


def run(batch, ticks: int) -> None:
    for _ in range(ticks):
        assert batch.step() == []
