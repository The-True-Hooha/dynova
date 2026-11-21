"""Depth and body-id images from the simulation's own raycasts. A camera reads the poses the last step left and changes nothing."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

__all__ = ["Camera", "look_at", "id_colors", "NO_BODY"]

NO_BODY = 0xFFFFFFFF


@dataclass(frozen=True)
class Camera:
    """A pinhole lens in pixel units. The camera looks down its local -z with +x to the right and +y up; row 0 is the top row.

    `from_fov` builds the usual square-pixel lens from a vertical field of view, which the simulation cannot do itself because it has no `tan`.
    """

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    near: float = 0.05
    far: float = 100.0

    @classmethod
    def from_fov(cls, width: int, height: int, fov_y: float, near: float = 0.05, far: float = 100.0) -> "Camera":
        if not (math.isfinite(fov_y) and 0 < fov_y < math.pi):
            raise ValueError(f"fov_y must be in (0, pi) radians, got {fov_y!r}")
        f = height / 2 / math.tan(fov_y / 2)
        return cls(width, height, f, f, width / 2, height / 2, near, far)

    def words(self) -> bytes:
        for name in ("width", "height"):
            value = getattr(self, name)
            if not isinstance(value, int) or value < 1:
                raise ValueError(f"image {name} must be a positive integer, got {value!r}")
        return np.array([self.fx, self.fy, self.cx, self.cy, self.near, self.far], dtype="<f4").tobytes()


def look_at(eye: Sequence[float], target: Sequence[float], up: Sequence[float] = (0.0, 1.0, 0.0)) -> Tuple[float, ...]:
    """The pose `(x, y, z, qx, qy, qz, qw)` of a camera at `eye` looking at `target`, with `up` pointing up the image."""
    e, t, u = (np.asarray(v, dtype=np.float64) for v in (eye, target, up))
    f = t - e
    if not np.linalg.norm(f) > 1e-9 or not np.linalg.norm(u) > 1e-9:
        raise ValueError("look_at needs eye and target apart and a non-zero up")
    f = f / np.linalg.norm(f)
    side = np.cross(f, u)
    if np.linalg.norm(side) < 1e-6:
        raise ValueError("look_at: the view direction is parallel to up")
    x = side / np.linalg.norm(side)
    y = np.cross(x, f)
    z = -f
    m = np.stack([x, y, z], axis=1)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = math.sqrt(trace + 1) * 2
        q = ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = (s / 4, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s)
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = ((m[0, 1] + m[1, 0]) / s, s / 4, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s)
    else:
        s = math.sqrt(1 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, s / 4, (m[1, 0] - m[0, 1]) / s)
    return (float(e[0]), float(e[1]), float(e[2])) + tuple(float(c) for c in q)


def id_colors(ids: np.ndarray) -> np.ndarray:
    """RGBA bytes for a body-id image (shape `ids.shape + (4,)`): a fixed colour per id, transparent black for no body. For looking at, not for learning."""
    ids = np.asarray(ids, dtype=np.uint32)
    h = (ids.astype(np.uint64) + 1) * np.uint64(0x9E3779B1) & np.uint64(0xFFFFFFFF)
    out = np.empty(ids.shape + (4,), dtype=np.uint8)
    out[..., 0] = 64 + (h & np.uint64(0xBF))
    out[..., 1] = 64 + ((h >> np.uint64(8)) & np.uint64(0xBF))
    out[..., 2] = 64 + ((h >> np.uint64(16)) & np.uint64(0xBF))
    out[..., 3] = 255
    out[ids == NO_BODY] = 0
    return out
