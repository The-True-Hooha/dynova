# Three.js adapter

```ts
import { threeAdapter } from '@dynova/three';

const adapter = threeAdapter(world, instancedMesh, { scales, idOffset });
// every frame
const { live, hidden, truncated } = adapter.update();
renderer.render(scene, camera);
```

- **Source:** anything with `readTransforms(): Float32Array` (7 floats per entity: `x y z qx qy qz qw`). Entity id `N` drives instance `N - idOffset`.
- **Scale:** the transform buffer carries no size. Pass `scales` (3 floats per entity id, default 1) with the dimensions you spawned; it is read on every update, so you can change it at any time.
- **Hidden instances:** destroyed slots (zero quaternion) and any non-finite value become zero-scale matrices.
- **Truncation:** entities beyond `mesh.count` are not drawn and are reported in `truncated`; stale instances past the available entities are zeroed.
- **Mesh state:** the adapter sets `frustumCulled = false` (three caches the bounding sphere on first render, so moving instances would be culled) and dynamic draw usage.
- **Ownership:** Dynova never creates a `Scene`, camera, renderer or per-entity object.
