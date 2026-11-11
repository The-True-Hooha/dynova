# Decision: f32 determinism for lockstep (Stage 5A)

**Decision: do not migrate the solver to fixed-point now. f32 is bit-identical across every configuration tested, so lockstep is feasible on the current math. Commit to the f32 guarantee, scoped to what was tested, and keep a pinned-hash check in the suite so a change that breaks it is caught.**

The roadmap's question was whether lockstep needs fixed-point math. The Stage 1B smoke test only covered sin, cos, atan2 and sqrt in a toy integrator and one 30-body scene. This step extends the same harness to a scene that uses most of the solver.

## What was run

`core/src/determinism.rs` builds one scene from a fixed seed entirely inside the module (so no JS-side input rounding is involved) and returns four 64-bit hashes of the full simulation state (`Runtime::state_hash`: all body columns, sleep timers, island ids, contacts, warm-start cache, gravity, generations and liveness; not the hull table):

- 400 bodies plus a projectile and a thin wall: 398 dynamic boxes, spheres, capsules and convex hulls dropped in a 14 x 14 grid, about two layers deep, onto a static floor with a static backstop wall; a sphere fired at 95 m/s at a 0.1 m thick wall, which is stopped only by continuous collision (a test checks it ends on the near side of the wall);
- friction, restitution and island sleeping (most of the pile, about 380 of 400 bodies, is asleep by tick 400);
- scripted commands: a force at tick 100, a teleport at 200, a destroy and respawn with the next generation at 300, then a teleport at 310, a force at 325 and a gravity change at 340 (all three between the keyframe at tick 300 and the seek target), a gravity change at 400, another force at 500;
- 700 ticks, recorded, then a seek back to tick 350 and forward to the end.

The four values are the live end state, the live state at tick 350, tick 350 after a seek, and the end after seeking back. A seek must reproduce the live run, so the second and third (and the first and fourth) must be equal. The seek to tick 351 restores the keyframe at tick 300 and replays 51 ticks including the teleport, force and gravity commands logged at 310, 325 and 340; the seek back to the end restores the last keyframe and replays almost nothing, so the end-state match is mostly a keyframe round trip.

| build | live end | live tick 350 | tick 350 after seek | end after seek back |
|---|---|---|---|---|
| native aarch64 (Apple silicon) | 1177776368450856606 | 15081626903481916999 | = | = |
| native x86_64 (`--target x86_64-apple-darwin`, under Rosetta) | same | same | = | = |
| Chromium, wasm (`determinism_scene`) | same | same | = | = |
| WebKit, wasm (`determinism_scene`) | same | same | = | = |

### Joint scene

`determinism::run_joint_scene` (wasm: `determinism_joint_scene`) adds a 1230-tick world with every joint kind: a six-link ball chain, a hinged door with limits pushed by forces, a motor-driven wheel and slider (the slider's motor target flips at ticks 100 and 610), a fixed-joint pair that falls onto a floor, a joint destroyed by command at tick 640 and a body destroyed under a joint at tick 800. It is recorded and sought like the first scene. Keyframes fall every 60 ticks, so the seek target (tick 650) replays the motor change, the force and the joint destroy at 610, 625 and 640 from the keyframe at 600, and the seek back to the end replays a motor change at 1210 from the keyframe at 1200; removing replay from `Runtime::seek` turns this test red. Live end `10886105467219054376`, live tick 650 `16090749575230081017`; a seek reproduces both. The same four rows hold: native aarch64 (opt-level 3 and debug), native x86_64 under Rosetta, Chromium and WebKit. The 700-tick pins above are unchanged because a joint-free world hashes as before.

### Kinematic scene

`determinism::run_kinematic_scene` (wasm: `determinism_kinematic_scene`) runs 700 ticks of telemetry-fed static bodies: a platform carrying six boxes, a wall shoving six balls, a joint anchor with a hanging bob, and samples rejected on purpose (aimed at dynamic bodies), plus a teleport. Keyframes fall every 60 ticks, so the middle seek replays the samples at 370 and 385 from the keyframe at 360, and the seek back replays the sample at 690 from the keyframe at 660. Live end `11061039626079077839`, live tick 400 `4197951501668953752`; a seek reproduces both. Native aarch64 (opt-level 3 and debug), native x86_64 under Rosetta, Chromium and WebKit agree. Earlier pins are unchanged.

### Sensor results

`determinism::run_sensor_scene` (wasm: `determinism_sensor_scene`) settles the 700-tick scene's first 120 ticks, then answers 320 seeded raycasts and 40 sphere overlaps and hashes every result row: `7037252248839586730`. It matches native aarch64 (opt-level 3 and debug), native x86_64 under Rosetta, Chromium and WebKit. The queries use only add, subtract, multiply, divide, square root and comparisons, like the solver.

The older checks still hold in the same two browsers: the 30-body golden scene (`4599892428385582693`) and the sin/cos/atan2/sqrt probe (`7496710723231958662`) match the native build. Firefox could not be launched in the development sandbox, so it is untested; run `pnpm exec playwright test e2e/determinism.spec.ts --project=firefox` to add it.

The native rows are debug builds (the test profile); the pinned test was also run at opt-level 3 on both architectures. The browser rows run the release wasm (opt-level 3, LTO). Reproduce: `cargo test --lib determinism` (native, pinned), `cargo test --lib --target x86_64-apple-darwin determinism` (needs the x86_64 std; the repo's nightly `build-std` setup provides it), and `pnpm exec playwright test e2e/determinism.spec.ts --project=chromium --project=webkit` (needs `make build-st`).

## Why this is believable

- The runtime's simulation code (outside tests) calls no sine, cosine, arctangent, exponential or power function: only IEEE-754 single-precision add, subtract, multiply, divide, square root, reciprocal, min, max, clamp, absolute value, sign, floor and integer casts, which are correctly rounded or exact and so identical on every conforming engine. Rust does not contract a multiply and an add into a fused multiply-add unless asked, and WebAssembly has no implicit fusion, so the instruction sequences are the same.
- The scene fails loudly if it stops being a real test: unit tests require all four shapes and sleeping bodies to be present, the projectile to be stopped by the thin wall, and the middle and end ticks to hash differently; removing a scripted command, changing the seed or the middle tick, or leaving contacts or the warm cache out of the hash each turn the pinned test red.
- The pinned values are asserted in Rust and in Playwright, so any change to the solver or the scene must update both, on purpose.

## What this does not show

- **One build.** Hashes match between independently compiled native and wasm builds because they run the same source, but the guarantee in the roadmap ("within the single-build determinism guarantee") is about one wasm binary on different machines. That is what the browser rows test; they ran on one machine (Apple silicon) in two engines.
- **Not every browser or CPU.** Firefox is untested; x86_64 browsers were not run (x86_64 native was, which exercises the x86 floating-point path). Browser engines are allowed to differ on `sin`, `cos`, `atan2`, `exp` and `pow` of the JavaScript `Math` object; the wasm module does not call them (it would use Rust's compiled `libm`), which is why the probe matches; the simulation itself does not call them at all. Any future code that calls JavaScript math from the simulation would break this.
- **Not cross-version.** Rust compiler or `libm` updates can change a transcendental function's last bit. The pinned hashes exist to catch exactly that.
- **No networking.** This is the determinism precondition for lockstep, not lockstep: no input exchange, rollback or desync detection was built.

## Consequences

- Stage 5B's "fixed-point math migration for solver" is **not greenlit**: there is no failure to fix.
- Keep `e2e/determinism.spec.ts` in the default suite. Add Firefox when it can run.
- Before relying on lockstep in production, run the determinism spec on the real target browsers and CPUs (at least Firefox and a Windows or Linux x86_64 browser). If any row differs, the fix is to remove the differing operation, not to move everything to fixed-point.
- The state hash and the seek machinery from Stage 4B are the building blocks for desync detection: exchange `World.recording.hash` per tick between peers.
