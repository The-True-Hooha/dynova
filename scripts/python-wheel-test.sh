#!/bin/sh
# Installs the built wheel into a clean virtualenv away from the checkout and runs a world from it, so the test cannot pass by finding the repository's wasm.
set -eu
root=$(cd "$(dirname "$0")/.." && pwd)
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
wheel=$(ls "$root"/python/dist/dynova-*.whl | head -n 1)
uv venv --quiet --python 3.12 "$work/venv"
uv pip install --quiet --python "$work/venv/bin/python" "$wheel" pytest
cp "$root"/python/tests/golden_ts.json "$work/"
cd "$work"
env -u DYNOVA_WASM "$work/venv/bin/python" - <<'PY'
import json, pathlib, numpy as np
import dynova
from dynova import Batch, SHAPE_SPHERE, find_wasm

installed = pathlib.Path(dynova.__file__).resolve()
assert "site-packages" in installed.parts, installed
wasm = find_wasm()
assert wasm.parent.name == "_data" and "site-packages" in wasm.parts, wasm

with Batch(1, capacity=16, ring_capacity=64) as b:
    w = b.worlds[0]
    w.spawn(0, -1, 0, half=(20, 1, 20), inv_mass=0)
    w.spawn(0, 3, 0, shape=SHAPE_SPHERE)
    for _ in range(40):
        b.step()
    got = ",".join(str(int(v)) for v in np.ascontiguousarray(w.read_transforms()[:2], dtype=np.float32).view(np.uint32).ravel())
assert got == json.load(open("golden_ts.json"))["run_bits"], got

from dynova import BatchEnv
from dynova.envs.reacher import ReacherEnv
with Batch(2, capacity=16, ring_capacity=64) as b:
    env = BatchEnv(b, ReacherEnv(), seed=1)
    assert env.step(np.zeros((2, 1))).obs.shape == (2, 3)
print("wheel ok:", wasm)
PY
