# @dynova/inspector

A debug panel for `@dynova/sdk`: entity list and state, a selected entity's contacts and joints, profiler counters, and an optional recording timeline with scrubbing. It reads a read-only snapshot from shared memory and does not touch the command ring or the transform buffers.

```ts
import { InspectorView } from '@dynova/sdk';
import { mountInspector } from '@dynova/inspector';

const view = new InspectorView(world.memory, world.layout.headerByteOffset);
const panel = mountInspector(document.getElementById('panel')!, { view, setEnabled: (on) => world.setInspector(on) });
// each frame: panel.update();   when done: panel.dispose();
```

Pass `controls` (record, pause, seek, teleport) to enable editing and the timeline.
