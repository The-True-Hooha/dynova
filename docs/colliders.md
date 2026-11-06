# Scene colliders

```ts
import { collidersFromScene, spawnColliders } from '@dynova/three';

const { colliders, skipped } = collidersFromScene(gltf.scene);
const { spawned, failed } = spawnColliders(world, colliders, {
  body: (node) => (node.userData.dynamic ? { invMass: 1, restitution: 0.2 } : {}),
});
```

`collidersFromScene` walks every `Mesh`, bakes the node's world scale into its vertices and fits one shape per mesh:

| Shape | When |
|---|---|
| box (oriented) | all vertices lie in one plane (ramps, tilted floors): a thin box aligned to the plane, thickness 0.005 |
| sphere | vertices within 5% of one radius and the convex hull fills at least 80% of that sphere |
| box | the convex hull fills at least 92% of the axis-aligned bounds, or the mesh is flat along one axis (thickness clamped to 0.005) |
| hull | anything else; at most 32 vertices (farthest-point sampling, independent of vertex order) |

Classification uses the convex hull volume, so double-sided, overlapping or inconsistently wound meshes and internal cavities do not change the result (collision only sees the outer hull). A hull the runtime would reject for being too small or too thin falls back to a bounding box.

- Boxes and spheres are fitted in the mesh's own frame and posed with the node's world rotation, so rotated nodes give rotated boxes.
- A hull body's origin is its volume centroid. `ColliderDesc.position` already includes that offset.
- Hull ids are immutable, so identical geometry at identical scale uploads one hull.
- Unusable meshes are returned in `skipped` with a reason: `no-geometry`, `unsupported` (instanced, skinned, indices not in triples), `non-finite`, `too-large` (above 1e4), `degenerate` (a point, a line, or a mesh thinner than 0.01 in two axes, e.g. a small prop under a centimetre).
- Colliders are static unless `body(node)` returns `invMass`. Per-collider failures (ring full, hull limit) land in `failed` and do not stop the others.

Limits: a hull that cannot be recentred within 1e4 of its centroid is skipped as `too-large`; world positions beyond 1e6 are rejected by the runtime and appear in `failed` or the runtime's malformed counter; skew from a non-uniform parent scale combined with rotation is approximated by decomposing the world matrix; capsules are not fitted; the dropped bodies' rendering scale is not derived from colliders (pass your own `scales` to the Three adapter).
