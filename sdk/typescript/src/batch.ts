import { World, layoutFromRuntime, type RuntimePointers, type WorldConfig } from './World.js';
import type { MemoryMode } from './memory.js';

/** The wasm `Batch`: N independent worlds in one memory, stepped by one call. */
export interface BatchRuntime {
    world_count(): number;
    enable_physics(): void;
    step_all(): void;
    /** `[header, header words, ring slots, ring head, ring tail, id counter, ring capacity]` or nothing past the last world. */
    world_layout(i: number): Uint32Array | undefined;
    free?(): void;
}

function pointersOf(layout: ArrayLike<number>): RuntimePointers {
    return {
        header_ptr: () => layout[0],
        header_words: () => layout[1],
        ring_slots_ptr: () => layout[2],
        ring_head_ptr: () => layout[3],
        ring_tail_ptr: () => layout[4],
        id_counter_ptr: () => layout[5],
    };
}

/**
 * One `World` per world of a batch, all over the same memory. Write to a world with its own methods as usual; `step()` here
 * advances every world together, so do not call `step()` on an individual world.
 */
export class WorldBatch {
    readonly worlds: readonly World[];

    constructor(config: WorldConfig, memory: WebAssembly.Memory, mode: MemoryMode, private readonly batch: BatchRuntime) {
        const worlds: World[] = [];
        for (let i = 0; i < batch.world_count(); i++) {
            const layout = batch.world_layout(i);
            if (!layout) throw new Error(`the batch has no layout for world ${i}`);
            // The ring's mask and length come from the config; a larger one than the real ring would write into the next world's memory.
            if (layout[6] !== config.ringCapacity) throw new Error(`world ${i} has a ring of ${layout[6]} slots but the config says ${config.ringCapacity}`);
            worlds.push(World.attach(config, memory, mode, layoutFromRuntime(pointersOf(layout))));
        }
        this.worlds = worlds;
    }

    get size(): number { return this.worlds.length; }

    /**
     * Steps every world. The time each world records is the batch time divided by the number of worlds: one wasm call
     * steps them all, so per-world cost is not measured. A trap in the wasm step leaves every world unusable (they share
     * one runtime), so each is marked failed and the error is rethrown; an error while one world finishes its step does not
     * stop the others from finishing theirs.
     */
    step(): void {
        const t0 = performance.now();
        try {
            this.batch.step_all();
        } catch (err) {
            this.fail(`the batch trapped: ${String(err)}`);
            throw err;
        }
        const each = ((performance.now() - t0) * 1000) / Math.max(1, this.worlds.length);
        let first: unknown;
        let failed = false;
        for (const w of this.worlds) {
            try {
                w.finishStep(each);
            } catch (err) {
                if (!failed) { failed = true; first = err; }
            }
        }
        if (failed) throw first;
    }

    /** Marks every world unusable and frees the wasm batch. */
    dispose(): void {
        this.fail('batch disposed');
        this.batch.free?.();
    }

    private fail(message: string): void {
        for (const w of this.worlds) w.markFatal(message);
    }
}
