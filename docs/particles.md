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
- A particle tests at most 64 bodies (the lowest ids), and every body, so cost is particles x bodies x 4. A thousand particles against a thousand bodies took about 11 ms a tick in a native release build. There is no broadphase yet.
- Friction removes 0.4 of the sideways motion per substep and per body touched. It does not depend on load or on a moving body's speed.
- Coupling is one way: particles do not push bodies. Attachment is the next step.

## Not yet

The host reads no particle positions yet (a double-buffered transform buffer comes with the SDK step), and the TypeScript and Python rings do not write tag 20.
