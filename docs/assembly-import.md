# Importing an assembly from glTF

`importAssembly(root, world, options)` (in `@dynova/three`) turns a loaded glTF scene into an `Assembly` (`docs/assemblies.md`): every mesh becomes a part, and the mates are read from the nodes' extras. It does not parse files itself; load the scene with `GLTFLoader` or `loadModelFiles` (`docs/colliders.md`) and hand it the root.

```ts
const loaded = await loadGltfScene(url);
if (!loaded.ok) throw new Error(loaded.error);
const { assembly, parts, mates, skipped, errors } = importAssembly(loaded.scene, world);
assembly.drive({ crank: 0.5 }, gains);
```

## Parts

Each mesh goes through the same shape fitting as `collidersFromScene` (box, sphere or convex hull, scale baked in), at its world pose, under its **node name**. Names must be unique; a second three.js node with the same name is reported and left out.

`GLTFLoader` changes node names before the importer sees them: spaces, dots and brackets are removed (`my part.1` becomes `my_part1`) and duplicates get a suffix (`dup`, `dup_1`), so two nodes called `dup` in the file arrive as two different parts. The file's own name stays in `node.userData.name`; `with` accepts the part name first and otherwise the file's name, and says so when that name belongs to several parts. The extras must be on the mesh node: when a glTF node has several primitives the loader makes a group and puts the extras on the group, where they are ignored. Meshes that cannot be fitted (no geometry, instanced, skinned, too large) come back in `skipped` with the reason.

A part is dynamic with mass 1 unless its extras say otherwise. glTF exposes node `extras` to three.js as `node.userData`, so the file carries:

```json
"extras": { "dynova": { "static": true } }
"extras": { "dynova": { "mass": 2.5, "friction": 0.3, "restitution": 0.1 } }
```

`static: true` or `mass: 0` pins the part. The importer checks what the runtime would otherwise drop without a word: `static` must be a boolean, a mass `0` or from `1e-6`, friction from 0 to 2, restitution from 0 to 1, and the part within 1,000,000 units of the assembly origin. A part that fails is a row and is not spawned.

## Mates

A mate sits on one part (the child) and joins it to a partner, made from the pose the file was authored in. A part may carry one mate or an array of them (a connecting rod carries two); an array's mates are named `part#0`, `part#1`, and a single mate takes the part's name.

```json
"extras": { "dynova": { "mate": { "type": "revolute", "with": "base", "at": [-0.5, 0, 0], "axis": [0, 0, 1], "range": [-1.2, 1.2] } } }
```

| field | meaning |
|---|---|
| `type` | `fixed`, `ball`, `revolute`, `slider` or `distance` |
| `with` | partner part name; default the first mesh above this node in the hierarchy (plain groups in between are skipped; if that mesh is not a part, the mate says so) |
| `at` | the joint point in **this node's frame**; default the node origin. Not used by `distance` |
| `axis` | hinge or slider axis in this node's frame (required for those two) |
| `range` | `[lo, hi]` from the authored pose: radians, metres, or stretch; must contain 0 |
| `from`, `to` | `distance` only: the point on the partner (in the partner's frame) and the point on this part (in this node's frame) |

Points and axes are in node frames, so they follow the node if the model is moved, rotated, scaled or mirrored in the editor: the point goes through the node's world matrix, and the axis goes through it too and is normalised, so a mirrored or stretched node turns its axis the way it turns the part.

## Errors

A bad part or mate does not throw. `errors` holds `{ name, error }` rows (a node with no finite pose such as a zero scale, a duplicate name, a bad part property, a part too far from the origin, a part the world refuses, an unknown mate type, a missing, failed or ambiguous partner, a mate to itself, malformed points or a range, a joint the world rejects such as a range excluding the authored pose) and the rest of the assembly is still built. A geometry that makes `collidersFromScene` itself throw is not caught. Check `errors` before trusting the result: a mechanism with a missing mate imports without complaint otherwise. A malformed `origin` option does throw.

## Large coordinates

`options.origin` is the assembly's floating origin: a point, or `'centre'` (default) for the middle of the bounding box of the parts' shape centres. A file modelled far from the origin therefore simulates near zero. Pass `[0, 0, 0]` to switch it off. `examples/assemblyImport.ts` reads a crank-slider from a glTF file; modelled 500 km from the origin it gives the same state hash, bit for bit, as at the origin (`e2e/assemblyImport.spec.ts`). The importer works in 64 bits and rounds once, so this holds for that file; a model with arbitrary coordinates can differ in the last bit of a coordinate now and then, because the centre and the world points round differently in 64 bits.

## Limits

- Extras are the only way to describe mates; there is no import of a CAD format's own constraints (STEP, Onshape, Fusion), because glTF has no place for them.
- Part positions come from the mesh's shape (a hull is placed at its centroid), not the node origin, so a mate point given in the node's frame is correct while a point you compute from the mesh vertices is your responsibility.
- A mate that names a partner in a different hierarchy branch works by name; ancestors are only the default.
- Mates are made from the authored pose; the importer does not solve an assembly whose parts overlap or whose mates disagree.
