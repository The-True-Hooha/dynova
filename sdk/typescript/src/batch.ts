import { World, layoutFromRuntime, type RuntimePointers, type WorldConfig } from './World.js';
import type { MemoryMode } from './memory.js';
import { lensWords, poseWords, type Camera, type CameraPose, type WorldImage } from './camera.js';
import { NO_ENTITY } from './CommandRing.js';

/** The wasm `Batch`: N independent worlds in one memory, stepped by one call. */
export interface BatchRuntime {
    world_count(): number;
    enable_physics(): void;
    step_all(): void;
    /** `[header, header words, ring slots, ring head, ring tail, id counter, ring capacity]` or nothing past the last world. */
    world_layout(i: number): Uint32Array | undefined;
    render_all(width: number, height: number, lens: Float32Array, poses: Float32Array, ignore: number): void;
    depth_ptr(): number;
    id_ptr(): number;
    /** Pixels of one world's image. */
    image_pixels(): number;
    free?(): void;
}

/** What one `render` produced: every world's image in one buffer each, world `i` at pixels `i * width * height` onward. Valid until the next render or any call that grows the memory. */
export interface BatchImages {
    width: number;
    height: number;
    depth: Float32Array;
    id: Uint32Array;
    world(i: number): WorldImage;
}

/** @internal views of a batch's image buffers over `memory`. */
export function imagesOf(memory: WebAssembly.Memory, batch: Pick<BatchRuntime, 'depth_ptr' | 'id_ptr' | 'image_pixels'>, worlds: number, camera: Pick<Camera, 'width' | 'height'>): BatchImages {
    const per = batch.image_pixels();
    const depth = new Float32Array(memory.buffer, batch.depth_ptr(), per * worlds);
    const id = new Uint32Array(memory.buffer, batch.id_ptr(), per * worlds);
    return {
        width: camera.width,
        height: camera.height,
        depth,
        id,
        world: (i) => {
            if (!Number.isInteger(i) || i < 0 || i >= worlds) throw new RangeError(`world ${i} is not in this batch of ${worlds}`);
            return { depth: depth.subarray(i * per, (i + 1) * per), id: id.subarray(i * per, (i + 1) * per) };
        },
    };
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

    /**
     * One depth and one id image per world, from the poses the last step left, in one wasm call. `poses` has one entry per
     * world. Changes nothing in any world. The images are views into wasm memory and the next render overwrites them, so copy what must outlive it.
     */
    render(camera: Camera, poses: readonly CameraPose[]): BatchImages {
        const failed = this.worlds.find((w) => w.failure !== null);
        if (failed) throw new Error(`world unavailable: ${failed.failure}`);
        if (poses.length !== this.worlds.length) throw new RangeError(`render needs one pose per world: ${this.worlds.length} worlds, ${poses.length} poses`);
        const lens = lensWords(camera);
        const words = poseWords(poses);
        try {
            this.batch.render_all(camera.width, camera.height, lens, words, camera.ignore ?? NO_ENTITY);
        } catch (err) {
            // Only a trap leaves the instance unusable; an Error the core returned declined the render and ran nothing.
            if (err instanceof WebAssembly.RuntimeError) this.fail(`the batch trapped: ${String(err)}`);
            throw err;
        }
        return imagesOf(this.worlds[0].memory, this.batch, this.worlds.length, camera);
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
