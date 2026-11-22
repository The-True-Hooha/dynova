# Particles

Phase 16 adds position-based particles to the core: ropes first (16A), then cloth, attachments and soft bodies. This page describes what exists.

## Data

Columns for positions, velocities, inverse masses and radii; one table of distance links `(a, b, rest, stiffness)`; one table of groups `(id, first, count)`. A world with no particles holds empty vectors, allocates nothing and hashes exactly as before. Particles are in the snapshot and the state hash, so a seek, a reset and a keyframe replay reproduce them bit for bit.

## Making a rope

The ring command `ParticleChain` (tag 20, logged and replayed): `id`, `from`, `to`, `count` (2 to 1024), pins at either end (inverse mass 0), `radius`, `inv_mass` of the free particles and a link `stiffness` in (0, 1]. A bad description, a taken id or a full table is dropped and counted as malformed. Limits: 16,384 particles and 4,096 groups per world.

## Step

After the rigid step, in four substeps of a tick: move the free particles, pull each link towards its length (8 iterations), push particles out of bodies, then take the velocity from the motion. Only `+ - * /` and `sqrt` are used, so every target agrees to the bit.

- A particle is pushed out of a body by at most its radius a substep, and may leave at most 2 m/s because of that push, so a rope spawned inside a body leaves it gently.
- A particle moves at most 100 m/s. Collision is tested once per substep, so a body thinner than about 0.5 m can still be crossed at that speed.
- A particle tests at most 64 bodies (the lowest ids), chosen from the bodies near its group (see Cloth), so cost is particles x nearby bodies x 4. A thousand particles against a thousand nearby bodies took about 11 ms a tick in a native release build.
- Friction removes 0.4 of the sideways motion per substep and per body touched. It does not depend on load or on a moving body's speed.

## Cloth

The ring command `ParticleCloth` (tag 21, logged and replayed): a `nu` by `nv` sheet (2 to 64 a side) spanned by the edge vectors `u` and `v` from `origin`, with the same radius, mass and stiffness fields as a rope, a `bend_stiffness` in [0, 1] and pins (bits 0 to 3 pin the corners, bit 4 the whole first row). Links: structural (neighbours), shear (diagonals) and, when `bend_stiffness` > 0, bending (the particle two along a row or column, so a fold shortens it). Rest lengths are the distances at creation, so a skewed sheet is allowed. A 21 by 21 cloth dropped on a box drapes over it, rests on top and hangs down the sides (tested). Stiffness is the share of a length error removed per iteration: a heavy hanging cloth stretches, so it is not inextensible (the draped test allows 25 percent).

- No particle-particle or self collision: a folded cloth passes through itself.
- A body smaller than the gap between particles (spacing minus two radii) can pass between them; choose a radius of at least half the spacing.
- Per tick, each group finds the bodies near it once (a box of its extent plus 100 m/s of travel), so bodies far away cost nothing. A 64 by 64 cloth is 4,096 particles and about 25,000 links.

## Soft blocks

The ring command `ParticleSoft` (tag 24, logged and replayed): an axis-aligned lattice of 2 to 12 particles a side filling a box (`origin`, `size`), with radius, inverse mass and link stiffness. Each particle is linked to its neighbours along the axes, across the faces and through the cell corners (13 of the 26 directions), so the block resists stretch and shear and much of a collapse. A block dropped on the floor rests on it keeping at least 80 percent of its height, a softer one squashes more in the impact, and one landing on a box pushes it down (tests).

- There is no volume term and no collision between particles (or between groups), so a hard enough impact can squash a block flat and a block can be pushed slightly into a thin body. A 60 m drop with radius 0.03 put particle centres 2 cm into the floor.
- Blocks are axis-aligned when made; to tilt one, make it and attach it to a rotating body.

Cost is bounded for the whole world by `MAX_LINKS` (32,768 links, about 18 ms a tick in a native release build for the particle step at the limit): a command that would pass it is refused, and a 64 by 64 cloth is about 24,000 links.

## Attachments and coupling

Ring commands `ParticleAttach` (tag 22: particle, body id and generation, offset in the body's frame) and `ParticleDetach` (tag 23), logged and replayed. The particle number is its index in the world's particle columns (the first particle of a group is the group's `first`, which is the sum of the counts of the groups made before it). A particle has at most one attachment; a particle with no row is free. A destroyed body releases its attachments in the same step, even if a new body takes its slot.

- An attached particle is held at its point on the body, moving with it (its velocity is the body's there); it does not fall or collide. Its links still pull on it, and the body feels the opposite: a compliant pull that closes at most a quarter of the gap per tick (`ATTACH_GAIN`) and shares the body among every particle attached to it, so a whole cloth attached to one box cannot spin it up. A load hung from a rope sags: the attachment is a soft spring, not a rigid joint, and a heavy hanging body stretches it (a 1 kg box swinging on a 2 m rope reached 3 m from the pin in the test).
- Particles also push bodies they touch: a body that hits a rope slows and the rope moves on, and a rope dropped on a box adds momentum to it (tested). The reaction is the momentum the particle loses closing on the body, charged to the deepest body it touches, and a contact does not bounce.
- A body that carries or is touched by particles can still sleep when the pushes stop; a push of zero does not wake it. A rope that rests on a body keeps pushing on it, so that body stays awake while loaded.
- A particle with inverse mass 0 can be attached: it rides the body and adds no pull.
- One body's total gain from particles in a substep is bounded to 100 m/s and 100 rad/s.

## Fluid

Position-based fluid (the ring command `ParticleFluid`, tag 25, logged and replayed): a block of 2 to 32 particles per axis on a lattice of `spacing` (0.01 to 1 m), with an XSPH `viscosity` in [0, 1]. A particle weighs 1000 kg/m3 times its spacing cubed, so bodies feel real momentum. Each substep the neighbours within 2 spacings are found (cells sorted by key and index, so identical on every target) and 8 density iterations move each particle to cancel its density error; no libm is used.

- An 8 by 14 by 8 column of water (0.4 by 0.7 by 0.4 m, 0.05 m spacing) released in one corner of a 0.6 m basin spreads and settles to a level within 5 percent of the analytic volume (tested: about 4 percent low, because the surface layer feels no pressure; bounded 5 percent low to 1 percent high).
- Cost is about 7 microseconds a particle a tick in a native release build, so a world holds at most 4,096 fluid particles (about 30 ms).
- Fluid groups do not see each other: two pours into one place pass through one another. One group is one body of water.
- Collision is the same per-substep sphere test as other particles, so a body thinner than a substep's travel can be crossed at speed. Coarse, deep fluid is not supported: an 8 by 8 by 8 block at 0.5 m spacing (4 m deep) fell through a 1 m thick floor in the measurement, and a block spawned inside a thin slab spreads about a metre sideways. Spacings of 0.1 m and below, with bodies a few spacings thick, are what is tested.
- The level agrees with the analytic one to a few percent at spacings of 0.04 and 0.1; at 0.01 m a measurement found it 30 percent high, which is not understood (the level helper averages the top tenth of a small block). Fine fluid is untested for accuracy.

## Buoyancy

Water lifts a body as much as the water it displaces weighs (Archimedes). Each substep's poses are held for the tick, so the displaced volume is measured once a tick: the body is cut into 20 by 20 vertical columns across its bounding sphere, a ray up and a ray down give where each column enters and leaves it (every shape is convex), and the length of that inside the water is summed. The water is the group's footprint between its lowest particle and its surface (the mean of its top tenth plus half a spacing), measured along gravity. The lift acts at the centre of the displaced volume, so a half-submerged body is also turned. The water takes speed out of a body in proportion to the water it displaces (`WATER_DRAG`, 6 per second, capped at half a body's speed a substep). A fluid particle that lands on a body faster than 1.5 m/s also gives it its momentum. Sand keeps the push-out pressure charge (bounded at 0.5 m/s a substep, shared among the grains pressing on the body).

A cube of density ratio 0.25, 0.5 and 0.75 dropped into 0.6 m of water settles at a draft of 0.070, 0.141 and 0.211 m against Archimedes' 0.075, 0.15 and 0.225 (6 percent shallow; tested within 12 percent). A cube held still in settled water feels the weight of the water it displaces to within 15 percent (tested). The error is the column grid: its cells are about 9 percent of the width of an axis-aligned cube, and the way a face falls across them moves the volume by up to that.

Limits:
- The water is the group's bounding region, not the particles: a stream, a splash or water poured onto a body from above is not seen as water, so it neither lifts nor weighs on the body (a landing particle still gives its momentum). A plate lying under still water that has nothing under it is lifted as if the water were under it.
- Groups poured into one place overlap; a body takes the lift of the group that displaces most of it.
- Drag is against still water, so a current does not carry a body.
- Floating bodies never sleep.
- A thin body is sampled by few columns; a body larger than the pond may get no lift.
- Cost: up to 800 rays a tick for each dynamic body in the water's region.

## Sand

The ring command `ParticleSand` (tag 26, logged and replayed; same slot layout as fluid, with Coulomb `friction` in 0 to 1 where fluid has viscosity): a lattice of 2 to 32 grains per axis, spacing equal to the grain diameter. A grain weighs 1600 kg/m3 times its diameter cubed. Fluid and sand share the 4,096 particle cap.

- Contact: each substep, pairs closer than a diameter are pushed apart in 8 iterations, with the sliding they did against each other limited by a Coulomb term.
- Static friction: a grain that rests on something (it touches a body, or is linked to a grain that does, through contacts) stays put while the slope of the contact normal under it is no steeper than `atan(friction)`, and slides when steeper; grounded grains that touch another lose a tenth of their speed a substep. A column of sand in free fall is not grounded, so it falls freely (tested).
- Heap angle: a 10 by 10 by 10 block of 0.05 m grains dropped on a floor stands at about 11 degrees with no friction and at 23, 49 and 57 degrees at friction 0.3, 0.6 and 1.0 (height over the radius that holds nine tenths of the grains, after 600 ticks). The test asserts that the angle grows with friction and is at least `atan(friction)` (17, 31 and 45 degrees). The hold is sticky, so a heap stands steeper than that: this is a lower bound, not the angle of repose a granular pour reaches, and a block that barely spreads is not a pour.
- A heavy body landing on a heap can end up overlapping grains: held grains are put back where the substep began without testing bodies again. Pinned grains (inverse mass 0) are skipped by contact and friction.

## From TypeScript and Python

```ts
const rope = world.createChain({ from: [0, 5, 0], to: [2, 5, 0], count: 11, pinFirst: true });   // { id, first, count }
const cloth = world.createCloth({ origin, u, v, nu: 21, nv: 21, pins: 0b10001 });
const block = world.createSoftBlock({ origin, size, n: [4, 4, 4] });
const water = world.createFluid({ origin, n: [16, 12, 16], spacing: 0.05, viscosity: 0.02 });   // 2..32 a side, spacing 0.01..1 m
const sand = world.createSand({ origin, n: [8, 8, 8], spacing: 0.05, friction: 0.6 });          // friction 0..1
world.attachParticle(rope.first + 10, body, [0, 0.5, 0]);
const rows = world.readParticles();   // Float32Array [x, y, z, radius] per particle, one read, one tick
```
```python
rope = world.create_chain((0, 5, 0), (2, 5, 0), 11, pin_first=True)
water = world.create_fluid((0, 0.05, 0), (16, 12, 16), 0.05, viscosity=0.02)
sand = world.create_sand((3, 0.05, 0), (8, 8, 8), 0.05, friction=0.6)
rows = world.read_particles()         # (n, 4) float32
```

- The read is one call over a header-announced buffer (words 41 to 43, the same sequence-word pattern as joint states): rewritten after every step and seek, empty before the first particle, never one call per particle. The header grew from 41 to 44 words, so the wasm package must be rebuilt; a stale one is refused at load.
- `first` is predicted by the host as the sum of the sizes of the groups it has made. It is wrong if an earlier particle command was refused (counted in `malformedDropped`) or a world already holds 4,096 groups or 32,768 links (the core refuses those and counts them in `malformedDropped`), so read `readParticles()` after a step if in doubt. After a seek or a `Reset` the host sets the next `first` from the core's particle count and rebuilds the fluid and sand share of the cap from the groups it queued, once the runtime has drained the queue (the next create after a step). Groups queued after the seek or reset command are kept as made. The share is still a prediction: a group the core refused (memory, the group or link limit) or one from a timeline the host has forgotten (a seek forward after others were made) can leave it too high or too low, and the core is the judge. Group ids only grow. A seek forward into a timeline the host has forgotten (it made other groups after seeking back) still gets the right `first`, but the fluid and sand share is then under-counted and the core is the judge.
- Requests are checked on the host with the same bounds as the core, so a bad one throws before anything is queued. The wire bytes of every particle command are pinned in `python/tests/golden_particles_ts.json`, written by the TypeScript ring and read by the Rust and Python tests.
- Fluid and sand share a cap of 4,096 particles a world, checked on the host too (`at most 4096 fluid and sand particles`). A refused request never reaches the ring, so it cannot shift later groups. Spacing, viscosity and friction are judged on the f32 values the core sees (the origin and counts are judged as given). The host count, like `first`, is set again from the core's count after a `Reset` or a seek, with the same caveat. Python counts may be `int` or numpy integers; floats such as `2.0` are refused, TypeScript accepts `2.0`.
- Particles are not part of `Batch.render` cameras (cameras image bodies only).

## Budget

The step is the cost: reading rows is one copy. Measured in the browser demo (`examples/fluid.ts`, `e2e/fluid.spec.ts`): a 1.2 m basin with 3,072 water particles, a floating cube and 512 sand grains, 3,584 particles in all, median step 27 ms in Chromium and 26 ms in WebKit (p95 29 and 32 ms, ticks 450 to 600) on an arm64 laptop, which is about 7.5 microseconds a particle. A 60 fps frame (16.7 ms) therefore holds about 2,000 fluid or sand particles with nothing else running, and the 4,096 cap would cost about 31 ms (extrapolated at that rate, not measured). Ropes, cloth and soft blocks are far cheaper a particle (see their sections). Firefox and x86_64 timings were not measured.
