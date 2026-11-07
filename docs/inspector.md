# Inspector (read-only)

The inspector shows what the runtime is doing without touching the simulation: a profiler, the list of entities, one selected entity (position, rotation, velocity, angular velocity, state) and its contact points. It reads the same shared memory the game reads and has no write path other than switching the snapshot on and off.

```ts
import { InspectorView } from '@dynova/sdk';
import { mountInspector } from '@dynova/inspector';

const panel = mountInspector(document.getElementById('panel')!, {
  view: new InspectorView(memory, headerByteOffset),
  setEnabled: (on) => world.setInspector(on),
});
// ...
panel.dispose(); // switches the snapshot off and removes the panel
```

Run `make dev` and open `/inspector.html` for a live pile with the panel beside it.

## How it works

- `world.setInspector(true)` queues an `Inspect` command (tag 7) on the command ring. The runtime allocates the snapshot on the first one. While it is off the runtime copies nothing.
- Each tick, while on, the runtime writes one row per entity id (`position, rotation, velocity, angular velocity, state`) and up to 4096 contact rows (`point, normal, depth, a, b`) into its own buffer. State is `unused`, `awake`, `sleeping` or `static`. The transform is copied into the row on purpose: the transform triple buffer supports one consumer, which is the game, so the panel must not read it.
- A sequence word guards the frame (odd while it is being written). `InspectorView.snapshot()` copies the rows, re-checks the sequence and the layout epoch, and retries up to 8 times; after that it returns `null` and counts a missed read. The copy stays valid while the simulation keeps running.
- The wasm module has no clock, so the thread that calls `step()` times it: `World.step()` in fallback mode and the worker loop in worker mode write the last step time and a wrapping total (microseconds) into the header. The panel derives ticks per second and the mean step time from two samples.
- Contact `normal` is the direction the contact pushes the selected entity. A snapshot holds at most 4096 contacts even when the runtime has more; the profiler shows the full count.

## What the profiler shows

Tick, ticks per second, mean and last step time, command queue depth, contacts (dropped), worker overruns, memory size and buffer growths. It also warns when the snapshot is more than 30 ticks old (simulation paused, or not publishing).

## Cost

`e2e/inspector.spec.ts` runs a 5000-body pile with the inspector closed and open (alternating, best of two) and fails if the open median step is more than 15% slower; it prints both medians and the time of one snapshot read (a fraction of a millisecond on the development laptop). The step timer does not include the snapshot read, which happens on the reader's thread. Opening the inspector does not change the simulation: the same scene produces identical transforms with and without the panel.

## Limits

- Read-only. Moving entities, snapshots and replay are Stage 4B.
- One panel per `InspectorView` (a second `mountInspector` on the same view throws); no popup window or browser extension.
- The entity list draws the first 200 matching entities; the note says how many matched.
- A snapshot older than the latest tick is shown as it was; the panel warns instead of hiding it.
