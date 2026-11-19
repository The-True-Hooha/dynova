import init, { Batch } from '../wasm/dynova_core.js';
import { createBatchHost, type BatchRequest, type BatchReply } from './batchHost.js';

const host = createBatchHost((worlds, capacity, ringCapacity) => new Batch(worlds, capacity, ringCapacity), (m: BatchReply) => (self as unknown as Worker).postMessage(m));
let ready: Promise<void> | null = null;

self.onmessage = (e: MessageEvent<BatchRequest>) => {
    const request = e.data;
    if (request.type === 'init') {
        ready ??= init({ module_or_path: new URL('../wasm/dynova_core_bg.wasm', import.meta.url), memory: request.memory }).then(() => undefined);
        ready.then(() => host(request), (err) => (self as unknown as Worker).postMessage({ type: 'error', message: err instanceof Error ? err.message : String(err) } satisfies BatchReply));
    } else {
        host(request);
    }
};
