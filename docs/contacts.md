# Contact solver (Phase 13)

What changed, measured against the scenes in `core/src/physics/contact_accuracy.rs`:

- **Sub-steps for every world.** Two sub-steps per tick without joints (four with, as before), twelve solver iterations per sub-step in both. A free fall now matches gravity (speed after one second within 1 percent, distance within 2).
- **Damping only where it is needed.** The global 0.99 linear and 0.97 angular damping slowed free fall by 25 percent. Now a jointed body keeps the old damping (joints need it), a body touching something loses 2 percent of its spin per tick, a body in the air loses nothing. The micro-velocity clamp that zeroed a slow body's velocity is gone.
- **Circular friction cone.** The two tangent rows are one constraint stepped together and pulled back onto the disc `|lt| <= mu * lambda`. The old per-row clamp gave a square, whose friction limit is sqrt(2) times larger along a diagonal, so a box on a slope facing diagonally to the tangent axes slid 18 percent less far than the same slope facing along one (measured: 1.9 m against 2.8 m after a second). Sliding on the floor now loses speed at the same rate in every direction (4 percent).
- **Warm starts found by position.** A contact's accumulated impulses are matched to last tick's by where the point is (within 0.3 of the smaller body's extent), not by its index in the manifold, and friction is carried as an impulse in space, so the tangent basis switch at |nx| = 0.57 no longer throws it away. The warm entry grew: position, normal, first body.
- **Bottom-up solve order.** Contacts are sorted along gravity and swept up then down, so support travels through a stack in one sweep. Twenty boxes stay up and still; forty stay up (checked by position; all forty were also seen asleep in a probe, not asserted).
- **Rest by acceleration.** A touching body whose velocity still changes faster than 0.5 m/s^2 is not at rest, so a box creeping down a slope its friction cannot hold keeps sliding instead of falling asleep and freezing.
- **A grasp holds.** Two sliding fingers with a motor pinch and lift a box without slipping (it already did; now tested).

## Cost

Release-speed native, one machine, measured with a throwaway test that is not in the repo, per tick: a settling pile of 1000 boxes 0.87 ms before, about 2.0 to 2.7 ms now (all asleep after the run); a rain of 3000 spheres 6.6 ms before, about 10.8 ms now. The 5000-entity browser frame budget specs still pass in Chromium.

## Review findings kept as limits

- The two friction rows are stepped with the larger of their two effective masses, which is not proven below the stability limit for strongly anisotropic bodies; a capsule on its side stops sliding in the test, no divergence was found.
- `before` (velocities) and `order` (contact order) are allocated every sub-step and `touching`/`penetrating` every tick; reusing scratch vectors would recover part of the cost increase.
- A running bond at half-brick offset puts the end bricks' centre of mass exactly over the edge of their single support. The old solver held that, the new one does not; the demos moved to a quarter offset instead of the solver being made to hold it. The car chassis height change is the same kind of regression.

## Behaviour that changed on purpose

- Every pinned hash changed (`docs/decisions/determinism.md`), including worlds without joints, which Phase 11 left alone.
- Bricks offset by half a box over the one below put the upper brick's centre of mass over the edge of its support; the wall demos use a quarter offset now. The car demo's chassis sits 1 cm higher so its wheels, not its belly, carry it.
- A column that topples is no longer slowed by air: boxes fly tens of metres. The collapsing-column test uses a wide floor.

## Not done

Torsional and rolling friction as real constraints, compliant contacts, a block or Newton solver, and a comparison against MuJoCo. A stack of 60 boxes still falls over; 40 holds.
