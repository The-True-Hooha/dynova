import { describe, it, expect } from 'vitest';
import { EntityIdAllocator } from '../EntityIdAllocator.js';

function makeMemory() {
    return new WebAssembly.Memory({ initial: 1 });
}

describe('EntityIdAllocator', () => {
    it('allocates sequential ids', () => {
        const mem = makeMemory();
        const alloc = new EntityIdAllocator(mem, 0);
        expect(alloc.alloc()).toBe(0);
        expect(alloc.alloc()).toBe(1);
        expect(alloc.alloc()).toBe(2);
    });

    it('reuses freed ids via freelist', () => {
        const mem = makeMemory();
        const alloc = new EntityIdAllocator(mem, 0);
        const a = alloc.alloc();
        const b = alloc.alloc();
        alloc.free(a);
        expect(alloc.alloc()).toBe(a);
        alloc.free(b);
        expect(alloc.alloc()).toBe(b);
    });

    it('allocated count reflects only counter, not freelist', () => {
        const mem = makeMemory();
        const alloc = new EntityIdAllocator(mem, 0);
        alloc.alloc(); alloc.alloc(); alloc.alloc();
        alloc.free(1);
        expect(alloc.allocated).toBe(3);
    });
});
