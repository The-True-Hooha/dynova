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

## Not yet

The host reads no particle positions yet (a double-buffered transform buffer comes with the SDK step), and the TypeScript and Python rings do not write tag 20.
