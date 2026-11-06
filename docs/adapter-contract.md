# Renderer adapter contract

A renderer adapter turns the transform buffer into draw state. It only reads the buffer through `source.readTransforms()`, never calls the runtime, and keeps no per-entity state between updates, so swapping adapters at runtime is calling a different `update()` on the same `World`. Swapping does not remove the previous renderer's output: clear the Canvas2D canvas yourself, and Three.js meshes keep their last matrices until you stop rendering them.

## Buffer layout (read side)

`source.readTransforms()` returns a `Float32Array` with `FLOATS_PER_TRANSFORM = 7` floats per entity id:

| Offset | Field |
|---|---|
| 0, 1, 2 | position `x y z` |
| 3, 4, 5, 6 | rotation quaternion `qx qy qz qw` |

- Entity id `N` starts at float `N * 7`. The length of the array is the entity capacity.
- A slot whose quaternion is exactly `(0, 0, 0, 0)` is empty or destroyed. Live entities never use it.
- The array is valid until the next `readTransforms()` call (and, in fallback mode, the next `step()`). An adapter must copy what it needs inside `update()`.
- Sizes are not in the buffer. Callers pass `scales`: 3 floats per entity id, default 1.

## Decoding (shared, in `@dynova/sdk`)

`decodeTransforms(source, capacity, sink, options)` applies the same rules for every renderer:

- slot `i` shows entity `i + idOffset` (`idOffset` is a non-negative integer); `capacity` is a non-negative number (fractions round down, `Infinity` allowed), otherwise a `RangeError`;
- empty slots and any entity with a non-finite pose or scale are hidden;
- the quaternion handed to the renderer is normalised;
- slots from the last live entity up to `capacity` are hidden so stale instances never linger (skipped when `capacity` is `Infinity`);
- the result is `{ live, hidden, truncated }`, where `truncated` counts entities with no slot.

A renderer supplies an `InstanceSink`: `show(slot, id, x, y, z, qx, qy, qz, qw, sx, sy, sz)` and `hide(slot)`.

## Adapters

| Package | Entry | Capacity | Notes |
|---|---|---|---|
| `@dynova/three` | `threeAdapter(source, instancedMesh, options)` | `mesh.count` | writes instance matrices, turns off frustum culling, dynamic draw usage |
| `@dynova/canvas2d` | `canvas2dAdapter(source, ctx, options)` | unlimited | a 2D view of the x/y plane, `+y` up. Angle is the body x axis projected onto the plane, so it is exact for rotation about z and only approximate for 3D tumbling. Size is scale x/y times `pixelsPerUnit`; a circle uses scale x. Entities draw in id order; z is ignored. The caller clears the canvas |

## Writing another adapter

1. Implement an `InstanceSink` and call `decodeTransforms` from `update()`.
2. Do not cache poses between calls and do not call the runtime.
3. Validate options at construction (`assertIdOffset` for the offset).
4. Return the `UpdateResult` so callers can see hidden and truncated counts.

If an adapter needs anything from `core/`, the contract is missing something; extend the buffer layout here first.
