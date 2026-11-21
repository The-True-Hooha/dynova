# Contributing to Dynova

Fork it, run it, change it, send it back. The code is MIT licensed (see `LICENSE`); by contributing you agree your contribution is under the same licence.

## Getting set up

Prerequisites: Rust nightly with the `wasm32-unknown-unknown` target (`rust-toolchain.toml` pins it), `wasm-pack`, Node 20 or newer and `pnpm`.

```bash
make install
make build build-st        # the two wasm builds the tests and demos load
make dev                   # demos at http://localhost:5173
```

## The checks a change must pass

```bash
cargo test && cargo clippy --all-targets -- -D warnings
make check && pnpm -r test
pnpm exec playwright test --project=chromium --project=webkit     # needs make build build-st first
make consumer-test                                                # only if you touched packaging
```

`NOT_VERIFIED.txt`-style gaps (Firefox, other platforms) are listed in `PLAN.md`; a Firefox or real-x86_64 run is a welcome contribution.

## House rules

- **A test per behaviour, including the awkward inputs** (NaN, huge values, stale ids, full rings, off-by-one boundaries). A new test should fail when the behaviour it names is removed; try the mutation.
- **Comments say why, not what.**
- **Conventional Commits** (`feat:`, `fix:`, `docs:`, `test:`, `refactor:`, `chore:`), one logical change per commit, gate green first.
- **Determinism is a claim with evidence.** A change that moves a pinned hash (`core/src/determinism.rs`, the pins in `e2e/`) must say why and update the pins and `docs/decisions/determinism.md` together. Never reach for floating-point functions the browsers disagree on (`sin`, `cos`, `atan2`, `exp`, `pow`).
- **The boundary stays narrow:** commands in, buffers out, no per-entity calls on the read path.
- **Scope:** `PLAN.md` has the rules and the parking lot. Something outside what you set out to do goes in the parking lot or an issue, not in the same change.

## Where things are

`core/` the Rust runtime, `sdk/typescript/` the SDK, `renderer/` the adapters, `inspector/` the debug panel, `examples/` the demos, `e2e/` the browser tests, `docs/` the guides.
