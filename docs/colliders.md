# Scene colliders

```ts
import { collidersFromScene, spawnColliders } from '@dynova/three';

const { colliders, skipped } = collidersFromScene(gltf.scene);
const { spawned, failed } = spawnColliders(world, colliders, {
  body: (node) => (node.userData.dynamic ? { invMass: 1, restitution: 0.2 } : {}),
});
```

## Loading models

```ts
import { loadModelFiles, loadModelUrl } from '@dynova/three/loaders';
import { collidersFromScene } from '@dynova/three';

const loaded = await loadModelFiles(files, { dracoDecoderPath: '/draco/' }); // files: { name, data: ArrayBuffer }[]
// or: await loadModelUrl('/level.glb', { dracoDecoderPath: '/draco/' });
if (loaded.ok) {
  const { colliders } = collidersFromScene(loaded.scene);
} else {
  console.error(loaded.error);
}
```

The loaders live in their own entry point, `@dynova/three/loaders`, so the adapter and collider code do not pull them in. Both functions return `{ ok: true, scene, format }` or `{ ok: false, error }` and never throw.

| Format | Notes |
|---|---|
| `.glb` | self-contained |
| `.gltf` | pass the `.gltf` together with its `.bin` and image files ; each reference binds to the file with the same path, or with the same name when only one such file exists. Missing files are listed in the error, and a name shared by several files is rejected as ambiguous. From a URL, resources resolve relative to it |
| `.obj` | one mesh per object; `.mtl` is ignored |
| `.stl`, `.ply` | ASCII or binary; one mesh named after the file |

glTF extras: Meshopt-compressed files load with no setup. **Draco** needs `dracoDecoderPath`, the URL of a directory (a trailing slash is added if missing) that serves `draco_decoder.js`, `draco_decoder.wasm` and `draco_wasm_wrapper.js` (they ship in `three/examples/jsm/libs/draco/gltf/`; the demo dev and preview servers serve them at `/draco/`). **KTX2** textures are replaced by blank textures when optional, because colliders do not need textures. A file that lists KTX2 as required fails with an error unless you pass `ktx2Loader` (a configured `KTX2Loader`). Decoding real KTX2 data through a supplied loader is not covered by the tests.

When several files are given, the primary one is the first `.glb`, then `.gltf`, `.obj`, `.stl`, `.ply`. STL and PLY carry no units or up axis and are used as given (Y up, 1 unit = 1 metre).

Not supported: folders (pick the files), skinned and instanced meshes (skipped as `unsupported`), and other formats (FBX, 3MF, USD).

The browser demo `examples/gltf.html` opens a built-in level (a small GLB generated in memory), or `?url=...`, or any of the files above chosen with the picker or dropped on the page (several at once for a `.gltf`); each load builds a fresh runtime.

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
- Hull ids are immutable. Within one `spawnColliders` call, identical geometry at identical scale uploads one hull; separate calls upload again, and the runtime allows 1024 hulls in total.
- Unusable meshes are returned in `skipped` with a reason: `no-geometry`, `unsupported` (instanced, skinned, indices not in triples), `non-finite`, `too-large` (above 1e4), `degenerate` (a point, a line, or a mesh thinner than 0.01 in two axes, e.g. a small prop under a centimetre).
- Colliders are static unless `body(node)` returns `invMass`. Per-collider failures (ring full, hull limit) land in `failed` and do not stop the others.

Limits: a hull that cannot be recentred within 1e4 of its centroid is skipped as `too-large`; world positions beyond 1e6 are rejected by the runtime and appear in `failed` or the runtime's malformed counter; skew from a non-uniform parent scale combined with rotation is approximated by decomposing the world matrix; capsules are not fitted; the dropped bodies' rendering scale is not derived from colliders (pass your own `scales` to the Three adapter).
