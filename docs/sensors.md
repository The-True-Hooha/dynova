# Sensors: raycast and overlap queries

A robot, an agent or a tool asks the world what is around a point. Queries go in through the command ring like every other command and are answered in a result buffer that only the runtime writes.

## API

```ts
const id = world.raycast({ origin: [0, 1, 0], direction: [1, 0, 0], maxDistance: 20, ignore: robotBody });
const near = world.overlapSphere({ center: [4, 1, 0], radius: 2 });
world.step();
const { results, lost } = world.readQueryResults();
// { query: 0, kind: 'hit', entity, distance, point, normal } | { kind: 'miss' }
// { query: 1, kind: 'overlap', entity, depth, point, normal } ... | { kind: 'overlap-none' } | { kind: 'overlap-more' }
```

- `raycast` returns the nearest body along the ray: `distance` is in world units along the (normalized) direction, `point` is where it enters, `normal` is the surface normal facing the ray. A ray that starts inside a body hits it at distance 0 with the normal pointing back along the ray. Ties go to the lowest entity id. Spheres, boxes, capsules and convex hulls are hit; static, sleeping and kinematic bodies are included.
- `overlapSphere` reports every body the sphere penetrates, lowest entity ids first, up to 64. `depth` is how far the sphere reaches into the body and `normal` points from the body towards the sphere's centre. A sphere that only touches does not overlap. When more than 64 bodies overlap, an `overlap-more` row follows the 64th.
- `ignore` names a body to skip, for example the robot that carries the sensor.
- Every query leaves at least one row, so a reader can tell "answered, nothing there" (`miss`, `overlap-none`) from "not answered yet". Results come back oldest first and carry the id `raycast` or `overlapSphere` returned. Ids count up from 0 for the life of the `World`.
- The runtime answers while it applies the command, from the poses the last step left, so the rows are readable after the next `step()`. A paused world still answers when the runtime drains its ring.
- A query changes nothing: it is not part of the state hash, not recorded and not replayed, so a seek does not answer old queries again.

## How it works

- Wire: tags 16 (`Raycast`) and 17 (`OverlapSphere`), one slot each; the byte layouts are pinned by fixtures in both test suites. The runtime rejects non-finite values, coordinates past 1e6, a zero or overflowing direction, and a range or radius outside (0, 1e6]; the SDK rejects the same before queueing, including values that round to zero in 32 bits.
- Result buffer: 4096 rows of 10 words (`query id, kind, entity, distance or depth, point xyz, normal xyz`), allocated when the first query arrives, written by the runtime only and overwritten oldest first. The header has two words for it: the buffer address and a running count of rows written. A reader keeps its own position; if it falls 4096 rows behind, `lost` counts the rows overwritten and the newest 4095 come back (the slot the runtime writes next still holds the oldest row). The runtime stores every row of a query first and announces them with one count update, so a reader on another thread sees an overlap whole or not at all.
- Search: a straight pass over every body with a bounding-sphere reject, so a query costs time linear in the body count. 5000 bodies and a few dozen queries per frame is far below a frame; thousands of queries per frame against a large world are not what this is for.
- Numbers: the sphere test measures from the vector to the centre, and so does the bounding check before it, so a ray aimed dead at a sphere hits it at long range (tested with oblique rays out to 900 km; `maxDistance` is capped at 1e6). The results are bit-identical across native aarch64, native x86_64, Chromium and WebKit for a fixed batch against a settled scene (`docs/decisions/determinism.md`).

## Limits

- Raycasts and sphere overlaps only; no capsule or box sweeps, no collision filters or layers, and no all-hits ray.
- A ray hits a convex hull where the hull is, which is the hull built from at most 32 points, not the mesh it was fitted to.
- A query that was queued but not yet applied has no row; there is no per-query completion callback.
