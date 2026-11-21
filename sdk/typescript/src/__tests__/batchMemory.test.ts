import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';

// Alone in its file on purpose: wasm memory only grows, so a module that already ran other batches could hide the cost of a new one.
const glue = await import('../../wasm-st/dynova_core.js');
const wasm = glue.initSync({ module: readFileSync(new URL('../../wasm-st/dynova_core_bg.wasm', import.meta.url)) });

describe('what a batch costs in wasm memory', () => {
    it('256 small worlds take a few MiB, not a megabyte each', () => {
        const before = wasm.memory.buffer.byteLength;
        const batch = new glue.Batch(256, 16, 64);
        batch.enable_physics();
        batch.step_all();
        const grown = (wasm.memory.buffer.byteLength - before) / (1 << 20);
        expect(grown).toBeLessThan(30);
        expect(batch.world_count()).toBe(256);
    });
});
