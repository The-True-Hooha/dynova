# @dynova/canvas2d

Canvas2D adapter for `@dynova/sdk`: draws a world as a 2D view of the x/y plane (position, rotation about z, scale). No dependencies besides the SDK.

```ts
import { canvas2dAdapter } from '@dynova/canvas2d';

const adapter = canvas2dAdapter(world, ctx, { scales, view: { pixelsPerUnit: 20, originX: 200, originY: 280 }, style: (id) => ({ color: '#44aaff' }) });
// each frame: clear the canvas, then
adapter.update();
```
