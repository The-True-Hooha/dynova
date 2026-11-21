import type { BatchRuntime } from './batch.js';

/** The wasm `Batch` as a worker drives it: one world at a time, so each world's step can be timed. */
export interface HostBatch extends BatchRuntime {
    step_world(i: number): boolean;
}

export type BatchRequest =
    | { type: 'init'; memory: WebAssembly.Memory; worlds: number; capacity: number; ringCapacity: number; physics: boolean }
    | { type: 'step' }
    | { type: 'render'; width: number; height: number; lens: Float32Array; poses: Float32Array; ignore: number };

export type BatchReply =
    | { type: 'ready'; memory: WebAssembly.Memory; layouts: Uint32Array[] }
    | { type: 'stepped'; micros: number[] }
    | { type: 'rendered'; depthPtr: number; idPtr: number; pixels: number }
    | { type: 'error'; message: string }
    /** A render the core declined (a bad lens or pose). Nothing ran, so the worker is still good. */
    | { type: 'refused'; message: string };

/**
 * The message handler of one batch worker, with the wasm and the clock passed in so tests run the same code the worker runs.
 * After any error the host stays failed: a trapped wasm instance is not stepped again.
 */
export function createBatchHost(
    newBatch: (worlds: number, capacity: number, ringCapacity: number) => HostBatch,
    post: (reply: BatchReply) => void,
    now: () => number = () => performance.now(),
): (request: BatchRequest) => void {
    let batch: HostBatch | null = null;
    let failed = false;
    const fail = (err: unknown) => {
        failed = true;
        post({ type: 'error', message: err instanceof Error ? err.message : String(err) });
    };
    return (request) => {
        if (failed) return post({ type: 'error', message: 'this worker has already failed' });
        try {
            if (request.type === 'init') {
                if (batch) return;
                const made = newBatch(request.worlds, request.capacity, request.ringCapacity);
                batch = made;
                if (request.physics) made.enable_physics();
                const layouts: Uint32Array[] = [];
                for (let i = 0; i < made.world_count(); i++) {
                    const layout = made.world_layout(i);
                    if (!layout) throw new Error(`the batch has no layout for world ${i}`);
                    layouts.push(layout);
                }
                post({ type: 'ready', memory: request.memory, layouts });
            } else if (request.type === 'render') {
                if (!batch) throw new Error('render before init');
                try {
                    batch.render_all(request.width, request.height, request.lens, request.poses, request.ignore);
                } catch (err) {
                    // A refusal is an Error the core returned; only a trap leaves the instance unusable.
                    if (err instanceof WebAssembly.RuntimeError) throw err;
                    return post({ type: 'refused', message: err instanceof Error ? err.message : String(err) });
                }
                post({ type: 'rendered', depthPtr: batch.depth_ptr(), idPtr: batch.id_ptr(), pixels: batch.image_pixels() });
            } else {
                if (!batch) throw new Error('step before init');
                const micros: number[] = [];
                for (let i = 0, n = batch.world_count(); i < n; i++) {
                    const t0 = now();
                    if (!batch.step_world(i)) throw new Error(`world ${i} is not in the batch`);
                    micros.push((now() - t0) * 1000);
                }
                post({ type: 'stepped', micros });
            }
        } catch (err) {
            fail(err);
        }
    };
}
