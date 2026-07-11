import { describe, it, expect } from 'vitest';
import { EntityIdAllocator } from '../EntityIdAllocator.js';

function makeAlloc(capacity = 16) {
    const mem = new WebAssembly.Memory({ initial: 1 });
    return new EntityIdAllocator(mem, 0, 4, capacity);
}

describe('EntityIdAllocator', () => {
    it('allocates sequential handles with gen=0', () => {
        const alloc = makeAlloc();
        expect(alloc.alloc()).toEqual({ id: 0, gen: 0 });
        expect(alloc.alloc()).toEqual({ id: 1, gen: 0 });
        expect(alloc.alloc()).toEqual({ id: 2, gen: 0 });
    });

    it('free increments generation in shared memory', () => {
        const alloc = makeAlloc();
        const h = alloc.alloc();
        expect(alloc.currentGen(h.id)).toBe(0);
        alloc.free(h);
        expect(alloc.currentGen(h.id)).toBe(1);
    });

    it('reuses freed id with incremented gen', () => {
        const alloc = makeAlloc();
        const h = alloc.alloc();
        alloc.free(h);
        expect(alloc.alloc()).toEqual({ id: h.id, gen: 1 });
    });

    it('gen increments each free cycle', () => {
        const alloc = makeAlloc();
        let h = alloc.alloc();
        alloc.free(h);
        h = alloc.alloc();
        alloc.free(h);
        expect(alloc.alloc()).toEqual({ id: 0, gen: 2 });
    });

    it('gen wraps at 256 back to 0', () => {
        const alloc = makeAlloc();
        let h = alloc.alloc();
        for (let i = 0; i < 256; i++) {
            alloc.free(h);
            h = alloc.alloc();
        }
        expect(h.gen).toBe(0);
    });

    it('allocated count reflects counter, not freelist', () => {
        const alloc = makeAlloc();
        alloc.alloc(); alloc.alloc(); alloc.alloc();
        alloc.free({ id: 1, gen: 0 });
        expect(alloc.allocated).toBe(3);
    });
});
