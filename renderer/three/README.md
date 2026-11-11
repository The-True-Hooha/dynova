# @dynova/three

Three.js adapter for `@dynova/sdk`: one `InstancedMesh` mirrors a world, plus helpers that turn a loaded scene into colliders and assemblies. Peer dependencies: `three` (0.170 is the version tested) and, for TypeScript, `@types/three`.

```ts
import { threeAdapter } from '@dynova/three';

const mesh = new InstancedMesh(geometry, material, world.capacity);
const adapter = threeAdapter(world, mesh, { scales });   // scales: Float32Array of x y z per entity id
scene.add(mesh);
// each frame, after the world has stepped:
adapter.update();
```

- `collidersFromScene`, `spawnColliders`, `fitShape`: meshes to box, sphere or convex hull colliders.
- `importAssembly`: a glTF scene to a mechanism of parts and mates (see `docs/assembly-import.md`).
- `@dynova/three/loaders`: `loadModelUrl`, `loadModelFiles`, `loadGltfScene` (glTF with Draco and Meshopt, OBJ, STL, PLY).
