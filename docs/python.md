# Python

`pip install -e "python[test]"` from a checkout installs the `dynova` package; `make package-python` builds a wheel that carries the wasm (needs `uv`), and `make consumer-test-python` installs that wheel into a clean virtualenv away from the checkout and runs a world from it. The wheel is not published. It hosts the same `dynova_core_bg.wasm` the browser runs (see `docs/decisions/python.md` for why and for the native alternative), so a world is bit-identical in Python and in the browser.

```python
from dynova import Batch, BatchEnv
from dynova.envs.reacher import ReacherEnv

with Batch(256, capacity=16, ring_capacity=64, workers=4) as batch:
    env = BatchEnv(batch, ReacherEnv(), seed=1)        # builds, checkpoints and resets every world
    obs = env.result.obs                               # (256, 3) float32
    r = env.step(actions)                              # (256, 1) array; r.obs, r.reward, r.terminated, r.truncated, r.resetting
```

The wasm is found at `$DYNOVA_WASM`, then in the package (`dynova/_data/`), then in a checkout at `sdk/typescript/wasm-st/` (`make build-st`). `make test-python` builds it and runs the tests.

## What is there

- **`Batch(worlds, capacity, ring_capacity, workers, physics)`**: `worlds` independent worlds spread over `workers` wasm instances (`partition`), stepped on threads (wasmtime releases the GIL). `step()` returns the indices of failed worlds, `[]` when all is well. Every world ends in exactly the state it would reach alone, whatever `workers` is. A trap in one instance fails only the worlds it holds; `step()` raises once none is left. The batch refuses sizes outside the budget with the runtime's reason (`docs/batch.md`).
- **`World`**: `spawn`, `destroy`, `force`, `teleport`, `set_gravity`, `create_joint`, `set_joint_motor`, `destroy_joint`, `pause`, `record`, `seek`, `checkpoint`, `reset`, with the SDK's id allocation (a destroyed id returns with the next generation, and the tables re-sync after a seek or reset) and its deferred queue when the ring is full. Reads: `read_transforms()` (a `(capacity, 7)` float32 view; a row with `qw == 0` is empty), `read_joint_states()`, `ticks`, `malformed_dropped`, `stale_dropped`. Bad arguments raise `ValueError`, `StaleHandle` or `RingFull` before anything reaches the ring; a value a 32-bit float cannot hold is a `ValueError`.
- **`BatchEnv(batch, spec, seed)`**: the environment of `docs/env.md`, same semantics and the same numbers: the same seeded generator (`mulberry32`, `episode_seed`), NEXT_STEP autoreset, truncation at `max_steps`, per-world per-episode seeds, a failure that queued commands makes the environment unusable. A task is an `EnvSpec` subclass (`build`, `reset`, `act`, `observe`, `reward`, `terminated`); `dynova.envs.reacher.ReacherEnv` is the one-arm example shared with `examples/reacherEnv.ts`.

## Checked

- `python/tests` (157 tests, real wasm). The two golden files (`tests/golden_ts.json`, `tests/golden_env_ts.json`) are pinned from both sides: the Python tests check them, and `sdk/typescript/src/__tests__/pythonGolden.test.ts` checks the same files against the TypeScript ring and `BatchEnv`, so a change in either language that moves a byte or a trajectory fails in both. They cover: every ring encoder against bytes the TypeScript ring wrote; a dropped ball bit-identical to the TypeScript run; the module reproduces the pinned native and browser hashes of the 700-tick and the joint scene; a 130-step environment trajectory (both ways of ending an episode, 19 resets) equals the TypeScript `BatchEnv`'s digest bit for bit for one, two and three workers (`tests/golden_env_ts.json`); 256 worlds on four workers equal the same on one; hostile arguments, a trap, a closed batch, a full ring.
- Step rate, one machine, 256 reacher worlds, 60 steps, hooks included: 32 000 world-steps per second on one worker, 46 000 on four, 54 000 on eight (Python 3.12, `wasmtime` 50). The TypeScript environment reaches 58 000 and 175 000 in headless Chromium. Python is slower because the three hooks per world run in the interpreter under the GIL; only the wasm step runs in parallel. A first version rebuilt the memory view on every access and reached 20 000 on one worker; it is now rebuilt only after a call into the module.

## Limits

- No hull bodies, kinematic batches, raycasts, overlaps, inspector or recording reads yet.
- A view from `read_transforms()` keeps its instance alive but its contents change at the next step; use it before the next call that steps or sends commands, or pass `copy=True`.
- A world's memory must not be touched from another thread while `step()` runs: every read and command raises then. A reset or step started from inside a hook raises (the environment's guard).
- Tested with Python 3.12 on one machine; the code parses under 3.9 but the suite was not run there. Linux and Windows were not run (the wasm is the same; the `wasmtime` wheel is theirs).
- A reset hook may not spawn, destroy or create joints (same reason as in `docs/env.md`).
