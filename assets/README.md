# Assets

Robot descriptions that ship with Dynova, listed in `manifest.json` with their licence and a SHA-256 of each file. Every asset here was written for this repository and is released under CC0 1.0 (public domain), so it can be used, changed and redistributed without conditions. Nothing is copied from another project. A test (`assets.test.ts`) fails if a file is not in the manifest, if its hash differs, or if it does not load.

Load one with `parseUrdf` and `spawnRobot` (TypeScript); see `docs/robots.md`.

| File | What | Notes |
| --- | --- | --- |
| `robots/arm.urdf` | a bolted three-joint arm | forward kinematics reference |
| `robots/humanoid.urdf` | a 37 kg biped, 12 driven joints, straight legs and flat feet | zero pose is standing; stands (no walking gait yet) |
| `robots/quadruped.urdf` | a 12-joint four-legged robot, 10.6 kg | zero pose is standing; stands and trots (`trot` in `legged.ts`) |

To add an asset: put the file here, give it a header comment that names its author and licence, add it to `manifest.json` with its hash (`shasum -a 256 file`), and add a test that loads it.
