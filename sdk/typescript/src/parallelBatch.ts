import { World, layoutFromRuntime, type WorldConfig } from './World.js';
import { createMemory, detectMode } from './memory.js';
import { SharedMemoryUnavailableError } from './launch.js';
import type { BatchReply, BatchRequest } from './batchHost.js';

/** What a `Worker` provides, so the same code runs over real workers and over test doubles. */
export interface BatchWorker {
    postMessage(message: BatchRequest): void;
    onmessage: ((event: { data: BatchReply }) => void) | null;
    onerror: ((event: { message: string }) => void) | null;
    terminate(): void;
}

export interface ParallelBatchOptions {
    config: WorldConfig;
    worlds: number;
    workers: number;
    physics?: boolean;
    initialPages?: number;
    maximumPages?: number;
    createWorker: () => BatchWorker;
}

/** How many worlds each worker gets: contiguous, every worker at least one, no two counts more than one apart. */
export function partition(worlds: number, workers: number): number[] {
    if (!Number.isInteger(worlds) || worlds < 1) throw new RangeError(`worlds must be a positive integer, got ${worlds}`);
    if (!Number.isInteger(workers) || workers < 1) throw new RangeError(`workers must be a positive integer, got ${workers}`);
    const n = Math.min(workers, worlds);
    return Array.from({ length: n }, (_, i) => Math.floor(worlds / n) + (i < worlds % n ? 1 : 0));
}

interface Group {
    worker: BatchWorker;
    worlds: World[];
    first: number;
    failed: string | null;
    waiting: ((reply: BatchReply) => void) | null;
}

/**
 * Worlds spread over several workers. Each worker owns its own wasm instance and its own shared memory, so a trap or an
 * out-of-memory in one worker stops only the worlds it holds. The page reads every world straight from shared memory.
 */
export class ParallelBatch {
    readonly worlds: readonly World[];
    private stepping = false;

    private constructor(private readonly groups: Group[]) {
        this.worlds = groups.flatMap((g) => g.worlds);
    }

    get size(): number { return this.worlds.length; }
    get workerCount(): number { return this.groups.length; }

    /** Indices into `worlds` of the worlds whose worker has failed. */
    failedWorlds(): number[] {
        return this.groups.flatMap((g) => (g.failed === null ? [] : g.worlds.map((_, k) => g.first + k)));
    }

    static async launch(opts: ParallelBatchOptions): Promise<ParallelBatch> {
        const counts = partition(opts.worlds, opts.workers);
        const mode = detectMode();
        if (mode === 'headers-missing') throw new SharedMemoryUnavailableError('headers-missing');
        if (mode === 'fallback') throw new Error('a ParallelBatch needs SharedArrayBuffer; use WorldBatch on the calling thread instead');
        const groups: Group[] = [];
        const dispose = () => groups.forEach((g) => g.worker.terminate());
        try {
            let first = 0;
            for (const count of counts) {
                // Memory first: if it throws no worker exists yet to be left running.
                const memory = createMemory(opts.initialPages ?? 256, opts.maximumPages ?? 16384).memory;
                const worker = opts.createWorker();
                const group: Group = { worker, worlds: [], first, failed: null, waiting: null };
                groups.push(group);
                first += count;
                const ready = await new Promise<Extract<BatchReply, { type: 'ready' }>>((resolve, reject) => {
                    worker.onmessage = (e) => (e.data.type === 'ready' ? resolve(e.data) : reject(new Error(e.data.type === 'error' ? e.data.message : `unexpected ${e.data.type} before ready`)));
                    worker.onerror = (e) => reject(new Error(e.message));
                    worker.postMessage({ type: 'init', memory, worlds: count, capacity: opts.config.capacity, ringCapacity: opts.config.ringCapacity, physics: opts.physics ?? false });
                });
                for (const layout of ready.layouts) {
                    if (layout[6] !== opts.config.ringCapacity) throw new Error(`a world has a ring of ${layout[6]} slots but the config says ${opts.config.ringCapacity}`);
                    group.worlds.push(World.attach(opts.config, ready.memory, mode, layoutFromRuntime({
                        header_ptr: () => layout[0], header_words: () => layout[1], ring_slots_ptr: () => layout[2],
                        ring_head_ptr: () => layout[3], ring_tail_ptr: () => layout[4], id_counter_ptr: () => layout[5],
                    })));
                }
            }
        } catch (err) {
            dispose();
            throw err;
        }
        const batch = new ParallelBatch(groups);
        // Between steps nobody waits, so a worker that errors or dies then is failed on the spot instead of being found by the next step.
        for (const g of groups) {
            g.worker.onmessage = (e) => (g.waiting ? g.waiting(e.data) : e.data.type === 'error' && batch.fail(g, e.data.message));
            g.worker.onerror = (e) => (g.waiting ? g.waiting({ type: 'error', message: e.message }) : batch.fail(g, e.message));
        }
        return batch;
    }

    /**
     * Steps every world whose worker is still alive, all workers at once, and resolves when each has answered. A worker
     * that traps or dies is marked failed (its worlds throw on use) and the others carry on; the result lists the
     * indices of every failed world so far, empty when all is well. Rejects when no worker is left, or with the first error
     * raised while finishing a world's step (the rest still finish theirs).
     */
    async step(): Promise<number[]> {
        if (this.stepping) throw new Error('a step is already running');
        const live = this.groups.filter((g) => g.failed === null);
        if (live.length === 0) throw new Error('every worker has failed');
        this.stepping = true;
        let first: unknown;
        let raised = false;
        try {
            await Promise.all(live.map(async (g) => {
                const reply = await new Promise<BatchReply>((resolve) => {
                    g.waiting = resolve;
                    g.worker.postMessage({ type: 'step' });
                });
                g.waiting = null;
                if (reply.type !== 'stepped') {
                    this.fail(g, reply.type === 'error' ? reply.message : `unexpected ${reply.type}`);
                    return;
                }
                if (reply.micros.length !== g.worlds.length) {
                    this.fail(g, `a step reply had ${reply.micros.length} times for ${g.worlds.length} worlds`);
                    return;
                }
                g.worlds.forEach((w, k) => {
                    // A world its owner has already failed or disposed has nothing to finish; its worker is still healthy.
                    if (w.failure !== null) return;
                    try {
                        w.finishStep(reply.micros[k]);
                    } catch (err) {
                        if (!raised) { raised = true; first = err; }
                    }
                });
            }));
        } finally {
            this.stepping = false;
        }
        if (raised) throw first;
        return this.failedWorlds();
    }

    private fail(group: Group, message: string): void {
        if (group.failed !== null) return;
        group.failed = message;
        for (const w of group.worlds) w.markFatal(`worker failed: ${message}`);
        group.worker.terminate();
    }

    /** Terminates every worker and fails every world. A step still waiting on a worker resolves, with every world listed as failed. */
    dispose(): void {
        for (const g of this.groups) {
            const wasFailed = g.failed !== null;
            g.failed ??= 'batch disposed';
            for (const w of g.worlds) w.markFatal('batch disposed');
            if (!wasFailed) g.worker.terminate();
            g.waiting?.({ type: 'error', message: 'batch disposed' });
        }
    }
}
