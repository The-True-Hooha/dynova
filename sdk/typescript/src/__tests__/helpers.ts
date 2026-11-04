import { World, WorldConfig, WorldLayout } from '../World.js';
import { SLOT_BYTES, TAG_SPAWN, TAG_DESTROY, TAG_GROW } from '../CommandRing.js';
import { FLOATS_PER_TRANSFORM, H_EPOCH, H_CAPACITY, H_BUF0, H_STATE, HEADER_WORDS } from '../TransformView.js';
import { createMemory } from '../memory.js';

const INITIAL_STATE = 1 | (2 << 3);

/** JS port of the Rust writer (triple buffer, header, grow, ring drain). Test double only. */
export class FakeRuntime {
    readonly layout: WorldLayout;
    readonly header: Int32Array;
    capacity: number;
    write = 0;
    private bump: number;
    private bufOffsets: number[] = [];
    readonly bytes: Uint8Array;
    readonly dv: DataView;
    ringCapacity: number;
    growCount = 0;

    constructor(readonly memory: WebAssembly.Memory, config: WorldConfig) {
        this.capacity = config.capacity;
        this.ringCapacity = config.ringCapacity;
        let o = 0;
        const take = (n: number) => { const at = o; o += n; return at; };
        const headerByteOffset = take(HEADER_WORDS * 4);
        const ringSlotsByteOffset = take(config.ringCapacity * SLOT_BYTES);
        const ringHeadByteOffset = take(4);
        const ringTailByteOffset = take(4);
        const idCounterByteOffset = take(4);
        this.layout = { headerByteOffset, ringSlotsByteOffset, ringHeadByteOffset, ringTailByteOffset, idCounterByteOffset };
        this.bump = Math.ceil(o / 16) * 16;
        this.header = new Int32Array(memory.buffer, headerByteOffset, HEADER_WORDS);
        this.bytes = new Uint8Array(memory.buffer);
        this.dv = new DataView(memory.buffer);
        this.allocBuffers(this.capacity);
        Atomics.store(this.header, H_STATE, INITIAL_STATE);
        this.publishHeader();
    }

    private f32(idx: number): Float32Array {
        return new Float32Array(this.memory.buffer, this.bufOffsets[idx], this.capacity * FLOATS_PER_TRANSFORM);
    }

    private allocBuffers(capacity: number) {
        const bytes = capacity * FLOATS_PER_TRANSFORM * 4;
        this.bufOffsets = [0, 1, 2].map(() => { const at = this.bump; this.bump += Math.ceil(bytes / 16) * 16 + 16; return at; });
    }

    private publishHeader() {
        Atomics.store(this.header, H_CAPACITY, this.capacity);
        this.bufOffsets.forEach((o, i) => Atomics.store(this.header, H_BUF0 + i, o));
    }

    writeTransform(id: number, t: number[]) {
        this.f32(this.write).set(t, id * FLOATS_PER_TRANSFORM);
    }

    publish() {
        const published = this.write;
        let cur = Atomics.load(this.header, H_STATE);
        for (;;) {
            const next = (cur & (3 << 3)) | 4 | published;
            if (Atomics.compareExchange(this.header, H_STATE, cur, next) === cur) break;
            cur = Atomics.load(this.header, H_STATE);
        }
        this.write = cur & 3;
        this.f32(this.write).set(this.f32(published));
    }

    latest(): Float32Array {
        return this.f32(Atomics.load(this.header, H_STATE) & 3);
    }

    grow(newCapacity: number) {
        Atomics.add(this.header, H_EPOCH, 1);
        const latest = this.latest().slice();
        this.capacity = newCapacity;
        this.allocBuffers(newCapacity);
        for (let i = 0; i < 3; i++) this.f32(i).set(latest);
        this.write = 0;
        this.publishHeader();
        Atomics.store(this.header, H_STATE, INITIAL_STATE);
        Atomics.add(this.header, H_EPOCH, 1);
        this.growCount++;
    }

    /** Consume every command in the ring as the worker would. */
    drain() {
        const head = new Int32Array(this.memory.buffer, this.layout.ringHeadByteOffset, 1);
        const tail = new Int32Array(this.memory.buffer, this.layout.ringTailByteOffset, 1);
        const mask = this.ringCapacity - 1;
        let n = 0;
        while (Atomics.load(head, 0) !== Atomics.load(tail, 0)) {
            const h = Atomics.load(head, 0);
            const off = this.layout.ringSlotsByteOffset + (h & mask) * SLOT_BYTES;
            const tag = this.dv.getUint8(off);
            const id = this.dv.getUint32(off + 1, true);
            if (tag === TAG_SPAWN) {
                const f = (o: number) => this.dv.getFloat32(off + o, true);
                this.writeTransform(id, [f(6), f(10), f(14), f(18), f(22), f(26), f(30)]);
            } else if (tag === TAG_DESTROY) {
                this.writeTransform(id, [0, 0, 0, 0, 0, 0, 0]);
            } else if (tag === TAG_GROW) {
                this.grow(this.dv.getUint32(off + 6, true));
            }
            Atomics.store(head, 0, h + 1);
            n++;
        }
        this.publish();
        return n;
    }
}

export function makeWorld(config: WorldConfig, opts: { shared?: boolean; pages?: number } = {}) {
    const pages = opts.pages ?? 8;
    const shared = opts.shared ?? true;
    const memory = shared
        ? createMemory(pages, pages * 4).memory
        : new WebAssembly.Memory({ initial: pages, maximum: pages * 4 });
    const rt = new FakeRuntime(memory, config);
    const world = World.attach(config, memory, shared ? 'shared' : 'fallback', rt.layout);
    return { world, memory, layout: rt.layout, rt };
}

export function makeSyncWorld(config: WorldConfig, onStep: (rt: FakeRuntime) => void, pages = 8) {
    const memory = new WebAssembly.Memory({ initial: pages, maximum: pages * 4 });
    const rt = new FakeRuntime(memory, config);
    const world = World.attachSync(config, memory, 'fallback', rt.layout, { step: () => onStep(rt) });
    return { world, memory, rt };
}
