# Python bindings: run the wasm, keep a native backend for later

Decision (Phase 12C spike, 2026-11-19): the first Python package runs the same `dynova_core_bg.wasm` the browser runs, hosted by `wasmtime`, behind a small backend interface. A native PyO3 wheel can be added later as a faster backend without changing the Python API. This is a choice between two measured options, written down so it can be reversed.

## What was measured

One machine (Apple M-series, 11 cores, macOS), Python 3.12, `wasmtime` 50, PyO3 0.26 and maturin, release builds. The native module wrapped the core crate directly; the wasm host instantiated `wasm-st/dynova_core_bg.wasm` with six import stubs (the panic hook, `throw`, `Error`, the externref table init).

| | native (PyO3) | wasm (wasmtime) |
|---|---|---|
| 700-tick mixed scene (`determinism_scene`), all four hashes equal the pinned values | yes, 0.476 s | yes, 0.571 s |
| 16 scenes on 1 / 4 / 8 threads (GIL released) | 7.7 / 2.4 / 1.5 s | 8.5 / 2.1 / 1.3 s |
| `step_all` of 256 empty worlds | 693 µs | 1160 µs |
| one Python call into an export | well under 1 µs | 13 µs |
| numpy view of 1 MiB of module memory | n/a (raw pointer) | 27 µs, no copy |

- Bit-identity is not a differentiator: native and wasm both produce the same four hashes, and native already matched the browsers.
- Speed: native is 1.2x faster on a real scene and 1.7x on many tiny worlds. Both scale across threads, because wasmtime releases the GIL; each thread needs its own instance, as each worker does in the browser. A Python call costs 13 µs, which is nothing against a step of hundreds of worlds.
- The linear memory base moves when the module grows during a step (observed: the base pointer changed after one `step_all`). A numpy view must be re-taken after every call that can allocate, exactly as the JS side re-acquires its views.

## Why wasm first

- **One artifact.** The package ships the wasm the browser ships, so "the same world gives the same bits in Python and in the browser" holds by construction, not by a CI matrix of builds that each have to be re-pinned.
- **Packaging I can verify.** A native wheel needs a Rust nightly toolchain and a build per platform and architecture (macOS arm64 and x86_64, Linux manylinux x86_64 and aarch64, Windows) with a CI that does not exist yet; only one of those can be run in this sandbox. The `wasmtime` wheel already exists for all of them and the package is pure Python.
- **The price.** 1.2x to 1.7x of single-thread speed, a 1 GiB memory cap per instance (the batch budget of 512 MiB per instance applies; larger batches use several instances), and the wasm-bindgen calling convention reimplemented in about 100 lines of Python.

## The interface that keeps native possible

The Python layer talks to a batch only through: construct, `enable_physics`, `step_all`, `step_world`, `world_layout(i)` (the addresses of a world's header, ring, head, tail and id counter), the memory those addresses point into, and free. Everything else (command encoding, id allocation, transform and joint-state reads, environments) is written once in Python over that memory. A native backend would hand back real process addresses instead of offsets into linear memory.

## Revisit when

A user needs more than one instance's memory in one call, or measured throughput on their workload is limited by the 1.2x to 1.7x, or Phase 18 puts a native compute backend in the same process. Then build the PyO3 backend against the same interface and re-measure.
