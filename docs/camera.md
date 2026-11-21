# Camera sensors

A camera renders a depth image and a body-id image of a world from the poses the last step left. It is a read: it changes nothing, is not a command, is not logged or replayed and is not part of the state hash. Every pixel is one raycast (`PhysicsWorld::raycast`), which uses only `+ - * /` and `sqrt`, so an image is bit-identical on every target, like the state it reads. One image of a fixed scene has the same hash in Rust, in Node on the shipped wasm and in Python (`PINNED_IMAGE_HASH` in `core/src/camera.rs`, `camera.test.ts`, `test_camera.py`).

## The lens

A pinhole, given as intrinsics in pixels: `fx, fy, cx, cy`, the image size, and `near` and `far`. The simulation takes intrinsics rather than a field of view because a field of view needs `tan`, and the simulation uses no libm. The SDKs turn a vertical field of view into square-pixel intrinsics on the host:

```ts
const camera = { width: 64, height: 64, fovY: Math.PI / 3, near: 0.1, far: 60 };
```
```python
camera = Camera.from_fov(64, 64, math.pi / 3, near=0.1, far=60)
```

The camera looks down its local -z, with +x to the right and +y up (the three.js convention). Row 0 is the top row. `lookAt` / `look_at` build a pose from an eye and a target.

There is no lens distortion and no rolling shutter; both are possible later without a wire change, because the lens is a small array of words.

## What an image holds

- `depth`: distance along the view axis, in metres, as `f32`. A flat wall square to the camera reads one value at every pixel, which is what a depth camera reports and not the longer length of an edge ray.
- `id`: the body each pixel's ray hits, which is the entity id the world gave it at spawn. `NO_ENTITY` (`0xffffffff`) means no body.
- A pixel that hits nothing within `[near, far]` has depth `0` and id `NO_ENTITY`. The id is the authority: with `near = 0` a camera inside a body also reads depth `0`, with that body's id. A hit nearer than `near` is skipped; the ray starts at `near`.
- Where two bodies are hit at the same distance, the lower id wins (the rule of `raycast`).
- A body the camera is inside is hit at `near`. Mount a camera on a body and pass its id as `ignore` to see through it.

`idColors` (TypeScript) and `id_colors` (Python) turn an id image into RGBA bytes for looking at. They are not part of the simulation and learning code should read the ids.

## Batches

Images for every world come from one call, into one buffer each, world `i` at pixels `i * width * height` onward. All worlds share one lens; each has its own pose.

```ts
const images = set.render(camera, poses);        // WorldBatch: BatchImages { depth, id, world(i) }
const images = await parallel.render(camera, poses);   // ParallelBatch: (WorldImage | null)[]
```
```python
images = batch.render(camera, poses)             # Images(depth, id): (worlds, height, width) copies
```

- `WorldBatch.render` returns views of wasm memory; the next render or a call that grows memory invalidates them. Copy what must outlive that.
- `ParallelBatch.render` has each worker render its own worlds at the same time. The entry of a world whose worker has failed is `null`. A lens or pose the core declines (a size past the limit, a pose outside range, a zero rotation) rejects the call and leaves every worker alive.
- Python returns copies. The images of a failed instance are all misses, and `failed_worlds()` says which.
- A refused render (bad lens or pose) is checked before any image is written, so no image changes. In Python, with several instances, the others finish before the error is raised.
- Limits: focal lengths 1e-3 to 1e7 and principal point within 1e7; 4,194,304 pixels in one image and 16,777,216 pixels across a batch; `far` and every position within 1e6.

## Cost

Measured once each, on one machine (an M-series laptop), so read them as orders of magnitude.

| Scene | Render | Time |
| --- | --- | --- |
| 64 worlds x 64x64, 31 static bodies each, Chromium | one thread | 41.8 ms |
| same | four workers | 19.1 ms |
| same, WebKit | one thread / four workers | 35.1 ms / 9.6 ms |
| 64 worlds x 64x64, the 26-body mixed determinism scene, native opt-3 | one thread | about 280 ms |

A pixel tests every body whose bounding sphere is inside the view pyramid, so cost grows with the bodies in view times the pixels. There is no spatial grid or tile culling yet; a scene of hundreds of bodies in view will be slow, and that is the first thing to add if a measured need appears.

## Limits

- Bodies only. No terrain or mesh other than the shapes the world has (box, sphere, capsule, hull).
- No colour, normals or optical flow. Normals are available from the raycast and can be added without a layout change.
- Cameras are placed by the caller each call; there is no camera body or joint.
- The Python package renders through the same module and cannot spawn hull bodies yet.
