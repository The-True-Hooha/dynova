import { describe, it, expect, vi, afterEach } from 'vitest';
import { detectMode, createMemory } from '../memory.js';

afterEach(() => vi.restoreAllMocks());

describe('detectMode', () => {
    it('returns shared when SharedArrayBuffer works', () => {
        expect(detectMode()).toBe('shared');
    });

    it('returns fallback when SharedArrayBuffer is undefined', () => {
        const orig = globalThis.SharedArrayBuffer;
        // @ts-expect-error intentional
        globalThis.SharedArrayBuffer = undefined;
        expect(detectMode()).toBe('fallback');
        globalThis.SharedArrayBuffer = orig;
    });

    it('returns headers-missing when SharedArrayBuffer constructor throws', () => {
        const orig = globalThis.SharedArrayBuffer;
        // @ts-expect-error intentional
        globalThis.SharedArrayBuffer = class { constructor() { throw new TypeError('cross-origin isolated'); } };
        expect(detectMode()).toBe('headers-missing');
        globalThis.SharedArrayBuffer = orig;
    });
});

describe('createMemory', () => {
    it('returns shared mode when SAB available', () => {
        const { mode } = createMemory(1, 16);
        expect(mode).toBe('shared');
    });

    it('returns headers-missing mode and unshared memory when SAB throws', () => {
        const orig = globalThis.SharedArrayBuffer;
        // @ts-expect-error intentional
        globalThis.SharedArrayBuffer = class { constructor() { throw new TypeError('cross-origin isolated'); } };
        const { memory, mode } = createMemory(1, 16);
        expect(mode).toBe('headers-missing');
        expect(memory).toBeInstanceOf(WebAssembly.Memory);
        globalThis.SharedArrayBuffer = orig;
    });
});
