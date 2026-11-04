import { World, WorldConfig, WorldLayout } from '../World.js';
import { SLOT_BYTES } from '../CommandRing.js';
import { FLOATS_PER_TRANSFORM } from '../TransformView.js';
import { createMemory } from '../memory.js';

/** Test double for the layout Rust normally exports. Production code never computes this. */
export function fakeLayout(c: WorldConfig): { layout: WorldLayout; bytes: number } {
    let o = 0;
    const take = (n: number) => { const at = o; o += n; return at; };
    const transformBytes = c.capacity * FLOATS_PER_TRANSFORM * 4;
    const layout: WorldLayout = {
        transformAByteOffset: take(transformBytes),
        transformBByteOffset: take(transformBytes),
        writeIndexByteOffset: take(4),
        ringSlotsByteOffset: take(c.ringCapacity * SLOT_BYTES),
        ringHeadByteOffset: take(4),
        ringTailByteOffset: take(4),
        idCounterByteOffset: take(4),
        generationsByteOffset: take(c.capacity),
    };
    return { layout, bytes: o };
}

export function makeWorld(config: WorldConfig) {
    const { layout, bytes } = fakeLayout(config);
    const pages = Math.max(1, Math.ceil(bytes / 65536));
    const { memory, mode } = createMemory(pages, pages);
    return { world: World.attach(config, memory, mode, layout), memory, layout };
}
