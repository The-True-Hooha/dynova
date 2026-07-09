import { describe, it, expect } from 'vitest';
import { TransformView, FLOATS_PER_TRANSFORM } from '../TransformView.js';

function makeView(capacity = 8) {
    const mem = new WebAssembly.Memory({ initial: 1 });
    const len = capacity * FLOATS_PER_TRANSFORM * 4;
    const aOffset    = 0;
    const bOffset    = len;
    const flagOffset = len * 2;
    return { mem, view: new TransformView(mem, aOffset, bOffset, flagOffset, capacity), aOffset, bOffset, flagOffset };
}

describe('TransformView', () => {
    it('reads from buf_b when write_index is 0 (Rust writing to buf_a)', () => {
        const { mem, view, bOffset } = makeView();
        const buf = new Float32Array(mem.buffer, bOffset, 8 * FLOATS_PER_TRANSFORM);
        buf[0] = 99;
        const flag = new Int32Array(mem.buffer, 8 * FLOATS_PER_TRANSFORM * 4 * 2, 1);
        Atomics.store(flag, 0, 0);
        expect(view.read()[0]).toBe(99);
    });

    it('reads from buf_a when write_index is 1 (Rust writing to buf_b)', () => {
        const { mem, view, aOffset, flagOffset } = makeView();
        const buf = new Float32Array(mem.buffer, aOffset, 8 * FLOATS_PER_TRANSFORM);
        buf[7] = 42;
        const flag = new Int32Array(mem.buffer, flagOffset, 1);
        Atomics.store(flag, 0, 1);
        expect(view.read()[7]).toBe(42);
    });

    it('readEntity returns correct fields', () => {
        const { mem, view, bOffset, flagOffset } = makeView();
        const flag = new Int32Array(mem.buffer, flagOffset, 1);
        Atomics.store(flag, 0, 0);
        const buf = new Float32Array(mem.buffer, bOffset, 8 * FLOATS_PER_TRANSFORM);
        const o = 2 * FLOATS_PER_TRANSFORM;
        buf[o] = 1; buf[o+1] = 2; buf[o+2] = 3;
        buf[o+3] = 0.1; buf[o+4] = 0.2; buf[o+5] = 0.3; buf[o+6] = 0.9;
        const e = view.readEntity(2);
        expect(e.x).toBeCloseTo(1);
        expect(e.y).toBeCloseTo(2);
        expect(e.z).toBeCloseTo(3);
        expect(e.qw).toBeCloseTo(0.9);
    });
});
