import { CommandRing, SLOT_BYTES } from '@dynova/sdk';

interface InitMessage {
    type: 'init';
    buffer: SharedArrayBuffer;
    ringCapacity: number;
    entityCount: number;
    halfW: number;
    halfH: number;
}

// Shared ring layout: [slots: ringCapacity*SLOT_BYTES][head: 4][tail: 4]
self.onmessage = (e: MessageEvent<InitMessage>) => {
    if (e.data.type !== 'init') return;
    const { buffer, ringCapacity, entityCount, halfW, halfH } = e.data;

    const memory = { buffer } as Pick<WebAssembly.Memory, 'buffer'> as WebAssembly.Memory;
    const ring = new CommandRing(
        memory,
        0,
        ringCapacity * SLOT_BYTES,
        ringCapacity * SLOT_BYTES + 4,
        ringCapacity,
    );

    for (let i = 0; i < entityCount; i++) {
        const x = (Math.random() * 2 - 1) * halfW;
        const y = (Math.random() * 2 - 1) * halfH;
        ring.spawn({ id: i, gen: 0, x, y, z: 0, qx: 0, qy: 0, qz: 0, qw: 1 });
    }

    self.postMessage({ type: 'ready' });
};
