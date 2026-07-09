import init, { Runtime } from '../sdk/typescript/wasm/dynova_core.js';

interface InitMessage {
    type: 'init';
    wasmUrl: string;
    capacity: number;
    ringCapacity: number;
}

interface ReadyMessage {
    type: 'ready';
    buffer: SharedArrayBuffer;
    transformAOffset: number;
    transformBOffset: number;
    writeIndexOffset: number;
    ringSlotsOffset: number;
    ringHeadOffset: number;
    ringTailOffset: number;
}

let runtime: Runtime | null = null;

self.onmessage = async (e: MessageEvent<InitMessage>) => {
    const msg = e.data;
    if (msg.type !== 'init') return;

    try {
        const wasmExports = await init(msg.wasmUrl) as unknown as { memory: WebAssembly.Memory };
        const wasmMemory = wasmExports.memory;

        console.log('[worker] memory shared?', wasmMemory.buffer instanceof SharedArrayBuffer);

        if (!(wasmMemory.buffer instanceof SharedArrayBuffer)) {
            self.postMessage({ type: 'error', message: 'WASM memory is not shared — rebuild with --shared-memory flag' });
            return;
        }

        runtime = new Runtime(msg.capacity, msg.ringCapacity);

        const ready: ReadyMessage = {
            type: 'ready',
            buffer:            wasmMemory.buffer as SharedArrayBuffer,
            transformAOffset:  runtime.transforms_buf_a_ptr(),
            transformBOffset:  runtime.transforms_buf_b_ptr(),
            writeIndexOffset:  runtime.transforms_write_index_ptr(),
            ringSlotsOffset:   runtime.ring_slots_ptr(),
            ringHeadOffset:    runtime.ring_head_ptr(),
            ringTailOffset:    runtime.ring_tail_ptr(),
        };

        self.postMessage(ready);
        loop();
    } catch (err) {
        console.error('[worker] error:', err);
        self.postMessage({ type: 'error', message: String(err) });
    }
};

function loop() {
    if (!runtime) return;
    runtime.step();
    setTimeout(loop, 16);
}
