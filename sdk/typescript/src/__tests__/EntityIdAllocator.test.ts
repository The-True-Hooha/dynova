import { describe, it, expect } from 'vitest';
import { EntityIdAllocator } from '../EntityIdAllocator.js';

function makeAlloc(capacity = 16) {
    const mem = new WebAssembly.Memory({ initial: 1 });
    return { mem, alloc: new EntityIdAllocator(mem, 0, capacity) };
}

describe('EntityIdAllocator', () => {
    it('allocates sequential handles with gen=0', () => {
        const { alloc } = makeAlloc();
        expect(alloc.alloc()).toEqual({ id: 0, gen: 0 });
        expect(alloc.alloc()).toEqual({ id: 1, gen: 0 });
        expect(alloc.alloc()).toEqual({ id: 2, gen: 0 });
    });

    it('free never writes shared memory (Rust is the only generation writer)', () => {
        const { mem, alloc } = makeAlloc();
        const before = new Uint8Array(mem.buffer.slice(0)).join();
        const h = alloc.alloc();
        alloc.free(h);
        const after = new Uint8Array(mem.buffer.slice(0));
        expect(after.slice(4).join()).toBe(before.split(',').slice(4).join());
    });

    it('reuses freed id with incremented gen', () => {
        const { alloc } = makeAlloc();
        const h = alloc.alloc();
        alloc.free(h);
        expect(alloc.alloc()).toEqual({ id: h.id, gen: 1 });
    });

    it('gen increments each free cycle', () => {
        const { alloc } = makeAlloc();
        let h = alloc.alloc();
        alloc.free(h);
        h = alloc.alloc();
        alloc.free(h);
        expect(alloc.alloc()).toEqual({ id: 0, gen: 2 });
    });

    it('gen wraps at 256 back to 0', () => {
        const { alloc } = makeAlloc();
        let h = alloc.alloc();
        for (let i = 0; i < 256; i++) {
            expect(alloc.free(h)).toBe(true);
            h = alloc.alloc();
        }
        expect(h.gen).toBe(0);
        expect(alloc.isLive(h)).toBe(true);
    });

    it('double free is rejected and does not duplicate the id', () => {
        const { alloc } = makeAlloc();
        const h = alloc.alloc();
        expect(alloc.free(h)).toBe(true);
        expect(alloc.free(h)).toBe(false);
        const a = alloc.alloc();
        const b = alloc.alloc();
        expect(a.id).not.toBe(b.id);
    });

    it('free of a stale or foreign handle is rejected', () => {
        const { alloc } = makeAlloc();
        const h = alloc.alloc();
        expect(alloc.free({ id: h.id, gen: 9 })).toBe(false);
        expect(alloc.free({ id: 99, gen: 0 })).toBe(false);
        expect(alloc.free({ id: -1, gen: 0 })).toBe(false);
        expect(alloc.isLive(h)).toBe(true);
    });

    it('throws RangeError at capacity and leaves the counter unchanged', () => {
        const { alloc } = makeAlloc(2);
        alloc.alloc(); alloc.alloc();
        expect(() => alloc.alloc()).toThrow(RangeError);
        expect(alloc.allocated).toBe(2);
    });

    it('freed ids are reusable at capacity', () => {
        const { alloc } = makeAlloc(1);
        const h = alloc.alloc();
        alloc.free(h);
        expect(alloc.alloc()).toEqual({ id: 0, gen: 1 });
    });

    it('capacity 0 always throws', () => {
        const { alloc } = makeAlloc(0);
        expect(() => alloc.alloc()).toThrow(RangeError);
    });

    it('allocated count reflects counter, not freelist', () => {
        const { alloc } = makeAlloc();
        alloc.alloc(); alloc.alloc(); alloc.alloc();
        alloc.free({ id: 1, gen: 0 });
        expect(alloc.allocated).toBe(3);
    });

    it('release returns the id with the same generation', () => {
        const { alloc } = makeAlloc();
        const h = alloc.alloc();
        expect(alloc.release(h)).toBe(true);
        expect(alloc.isLive(h)).toBe(false);
        expect(alloc.alloc()).toEqual(h);
    });

    it('release of a non-live handle is rejected', () => {
        const { alloc } = makeAlloc();
        const h = alloc.alloc();
        alloc.free(h);
        expect(alloc.release(h)).toBe(false);
    });

    it('counter overshoot by another allocator is detected after the add', () => {
        const { mem, alloc } = makeAlloc(2);
        const counter = new Uint32Array(mem.buffer, 0, 1);
        const realLoad = Atomics.load;
        let first = true;
        Atomics.load = ((a: Int32Array, i: number) => {
            if (first) { first = false; return 0; }
            return realLoad(a, i);
        }) as typeof Atomics.load;
        try {
            Atomics.store(counter, 0, 2);
            expect(() => alloc.alloc()).toThrow(RangeError);
        } finally {
            Atomics.load = realLoad;
        }
    });
});
