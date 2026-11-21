# Assemblies: named parts, mates and large coordinates

A CAD assembly is parts joined by mates. In Dynova a mate is a joint and an assembly is a thin naming layer over a `World`, plus one thing the joints cannot do for themselves: keep a model that sits far from the origin simulating accurately.

## Mates

```ts
const asm = new Assembly(world, [500_000, 0, 500_000]);   // the model's centre, in the model's own coordinates
asm.addPart('base',  { x: 499_999.7, z: 500_000, hx: 0.1, hy: 0.25, invMass: 0 });
asm.addPart('crank', { x: 500_000.5, z: 500_000, hx: 0.44, hy: 0.05 });
asm.addMate('crank', { type: 'revolute', a: 'base', b: 'crank', at: [500_000, 0, 500_000], axis: [0, 0, 1], range: [-1.2, 1.2] });
asm.drive({ crank: 0.9 }, gains);     // call after each step: PD control through the motors
asm.states();                         // { crank: { position, velocity } }
```

| mate | joint | free coordinate |
|---|---|---|
| `fixed` | fixed | none |
| `ball` | ball | none |
| `revolute` | hinge | angle (rad) |
| `slider` | slider | offset (m) along the axis |
| `distance` | distance | stretch: the distance between two points minus its length at creation |

- A mate is made with the parts in the pose they are in now. `range` is `[lo, hi]` measured from that pose, so `lo <= 0 <= hi`.
- A `distance` mate holds a point on one part at a fixed distance from a point on the other (a connecting rod, a strut). With a `range` it only limits the distance (a rope: slack inside the range, taut at the ends). The points must be at least 1 mm apart. On the wire the second point travels in the joint's axis words, so the ring format is unchanged.
- `states()` reads the joint state table (`docs/robotics.md`) and covers revolute, slider and distance mates; fixed and ball mates have no free coordinate. `drive()` works on revolute and slider mates only: a distance mate has no motor, so driving one throws.
- A mechanism with a closed loop works: `examples/assembly.ts` is a crank, a connecting rod (distance mate) and a slider on a rail. Driving the crank through +-0.9 rad three times keeps the carriage within 1 cm of the kinematic position and the rod's stretch under 1 cm (`e2e/assembly.spec.ts`).

## Large coordinates

The simulation stores positions as 32-bit floats. A float has 24 bits of precision, so the spacing between representable positions grows with distance from the origin: about 1 mm at 10 km, 8 mm at 100 km, 3 cm at 500 km and 6 cm past 524 km (the runtime accepts coordinates up to 1e6). A model far out loses motions smaller than that and its joints go slack by the same amount.

The crank-slider test (`e2e/assembly.spec.ts`) asserts only that, with raw coordinates 500 km out and no floating origin, the carriage ends more than 4 cm from where the crank puts it and the rod stretches by more than 1.5 cm; the same run at the origin stays within 1 cm and 1 cm. In one-off runs (not kept as tests) the carriage was off by up to about 6 cm and the rod stretched by up to 2.5 cm at 500 km, about 1.7 cm off at 100 km and 0.4 cm at 20 km. The cause of the extra error beyond the grid spacing was not investigated. Re-ordering the joint solver's anchor arithmetic was also tried in a one-off run on a pendulum chain and did not help at 500 km (worst anchor gap 0.134 m against 0.116 m before), which points at the coarse positions rather than the solver; that run is not kept either.

The fix is to keep the large numbers out of the runtime. An `Assembly` holds an origin in 64-bit floats and subtracts it before anything is sent, so the runtime simulates near zero whatever the original coordinates. With the origin at the model's centre, the same run 500 km out gives the same state hash, bit for bit, as at the origin (`e2e/assembly.spec.ts`). That is exact by construction (the offset is removed in 64 bits before 32-bit rounding and whole numbers like 500000 are exact), so the test guards the wiring, not solver accuracy far out. The final hash is pinned and equal in Chromium and WebKit.

- Choose the origin once, when the assembly is built, at the centre of the model. It cannot be moved afterwards: re-basing a running world would have to shift every body and keep their velocities, and no command does that (`teleport` zeroes velocity). If the model must travel far, move it with kinematic targets (`docs/digital-twin.md`) in local coordinates and keep the origin where it is.
- `toLocal(p)` and `toGlobal(p)` convert a point. Anything read back from the world (`readTransforms()`, query results, the inspector) is in local coordinates; add the origin back with `toGlobal`.
- Local coordinates that are a multiple of the float grid convert exactly. A global coordinate such as 500000.126 becomes a local 0.126 rounded to 32 bits, which keeps the 1 mm resolution; subtracting after rounding to 32 bits would not.
- Parts can still be added at any global position; the origin only decides where the 32-bit grid is densest. A model larger than about 100 km across has the same problem at its edges.

## Limits

- Mates are made from the current pose; there is no solver that assembles parts from constraints alone.
- Ranges are relative to that pose, not absolute angles.
- Distance mates inherit the joint solver's limits (`docs/joints.md`): a light body on a long rod swung hard stretches it.
- Importing parts and mates from a file is the next step (Phase 9B).
