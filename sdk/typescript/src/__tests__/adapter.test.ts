import { describe, it, expect } from 'vitest';
import { decodeTransforms, assertIdOffset, type InstanceSink } from '../adapter.js';

const F = 7;

function frame(...entities: number[][]): Float32Array {
    const buf = new Float32Array(entities.length * F);
    entities.forEach((e, i) => buf.set(e, i * F));
    return buf;
}

function recorder() {
    const shown: Record<string, number>[] = [];
    const hidden: number[] = [];
    const sink: InstanceSink = {
        show: (slot, id, x, y, z, qx, qy, qz, qw, sx, sy, sz) => { shown.push({ slot, id, x, y, z, qx, qy, qz, qw, sx, sy, sz }); },
        hide: (slot) => { hidden.push(slot); },
    };
    return { sink, shown, hidden };
}

const source = (buf: Float32Array) => ({ readTransforms: () => buf });
const ID = [0, 0, 0, 0, 0, 0, 1];

describe('decodeTransforms', () => {
    it('shows live entities with their pose, unit quaternion and default scale', () => {
        const r = recorder();
        const res = decodeTransforms(source(frame([1, 2, 3, 0, 0, 2, 2])), 4, r.sink);
        expect(res).toEqual({ live: 1, hidden: 0, truncated: 0 });
        expect(r.shown[0]).toMatchObject({ slot: 0, id: 0, x: 1, y: 2, z: 3, sx: 1, sy: 1, sz: 1 });
        expect(r.shown[0].qz).toBeCloseTo(Math.SQRT1_2, 6);
        expect(r.shown[0].qw).toBeCloseTo(Math.SQRT1_2, 6);
    });

    it('hides destroyed (zero quaternion) and non-finite entities and counts them', () => {
        const r = recorder();
        const res = decodeTransforms(source(frame([0, 0, 0, 0, 0, 0, 0], [NaN, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, Infinity], ID)), 4, r.sink);
        expect(res).toEqual({ live: 1, hidden: 3, truncated: 0 });
        expect(r.hidden.slice(0, 3)).toEqual([0, 1, 2]);
        expect(r.shown.map((s) => s.id)).toEqual([3]);
    });

    it('a non-finite scale hides the entity and scales are indexed by entity id', () => {
        const r = recorder();
        const res = decodeTransforms(source(frame(ID, ID)), 2, r.sink, { idOffset: 1, scales: [9, 9, 9, 2, 3, 4] });
        expect(res.live).toBe(1);
        expect(r.shown[0]).toMatchObject({ slot: 0, id: 1, sx: 2, sy: 3, sz: 4 });
        const bad = recorder();
        expect(decodeTransforms(source(frame(ID)), 1, bad.sink, { scales: [NaN, 1, 1] })).toEqual({ live: 0, hidden: 1, truncated: 0 });
    });

    it('reports truncation when capacity is smaller than the entity count and never shows past it', () => {
        const r = recorder();
        const res = decodeTransforms(source(frame(ID, ID, ID, ID)), 2, r.sink);
        expect(res).toEqual({ live: 2, hidden: 0, truncated: 2 });
        expect(r.shown.map((s) => s.slot)).toEqual([0, 1]);
    });

    it('hides leftover slots when the buffer has fewer entities than capacity, but not for unbounded capacity', () => {
        const r = recorder();
        decodeTransforms(source(frame(ID)), 4, r.sink);
        expect(r.hidden).toEqual([1, 2, 3]);
        const open = recorder();
        decodeTransforms(source(frame(ID, ID)), Infinity, open.sink);
        expect(open.hidden).toEqual([]);
        expect(open.shown).toHaveLength(2);
    });

    it('idOffset beyond the entity count yields nothing and an empty buffer is harmless', () => {
        const r = recorder();
        expect(decodeTransforms(source(frame(ID)), 3, r.sink, { idOffset: 5 })).toEqual({ live: 0, hidden: 0, truncated: 0 });
        expect(r.hidden).toEqual([0, 1, 2]);
        expect(decodeTransforms(source(new Float32Array(0)), 2, recorder().sink)).toEqual({ live: 0, hidden: 0, truncated: 0 });
    });

    it.each([[-1], [0.5], [NaN], [Infinity]])('rejects idOffset %s', (bad) => {
        expect(() => assertIdOffset(bad)).toThrow(RangeError);
        expect(() => decodeTransforms(source(frame(ID)), 1, recorder().sink, { idOffset: bad })).toThrow(RangeError);
    });

    it('reads the source once per call and is stateless between calls', () => {
        let reads = 0;
        const buf = frame([1, 0, 0, 0, 0, 0, 1]);
        const src = { readTransforms: () => { reads++; return buf; } };
        const r = recorder();
        decodeTransforms(src, 1, r.sink);
        decodeTransforms(src, 1, r.sink);
        expect(reads).toBe(2);
        expect(r.shown).toHaveLength(2);
    });

    it.each([[NaN], [-1], [-Infinity]])('rejects capacity %s', (bad) => {
        expect(() => decodeTransforms(source(frame(ID)), bad, recorder().sink)).toThrow(RangeError);
    });

    it('rounds a fractional capacity down and accepts zero', () => {
        const r = recorder();
        expect(decodeTransforms(source(frame(ID, ID, ID)), 1.9, r.sink)).toEqual({ live: 1, hidden: 0, truncated: 2 });
        expect(decodeTransforms(source(frame(ID)), 0, recorder().sink)).toEqual({ live: 0, hidden: 0, truncated: 1 });
    });

    it('checks every value for finiteness individually: huge finite scales stay visible, any single bad value hides', () => {
        expect(decodeTransforms(source(frame(ID)), 1, recorder().sink, { scales: [1e308, 1e308, 1] }).live).toBe(1);
        for (const bad of [[0, NaN, 0], [0, 0, Infinity]]) {
            expect(decodeTransforms(source(frame([...bad, 0, 0, 0, 1])), 1, recorder().sink).live).toBe(0);
        }
        expect(decodeTransforms(source(frame(ID)), 1, recorder().sink, { scales: [1, NaN, 1] }).live).toBe(0);
        expect(decodeTransforms(source(frame(ID)), 1, recorder().sink, { scales: [1, 1, Infinity] }).live).toBe(0);
        expect(decodeTransforms(source(frame([0, 0, 0, NaN, 0, 0, 1])), 1, recorder().sink).live).toBe(0);
    });

    it('a half turn quaternion (qw = 0, xyz not all zero) is a live entity, not an empty slot', () => {
        const r = recorder();
        expect(decodeTransforms(source(frame([0, 0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 1, 0])), 3, r.sink).live).toBe(3);
    });

    it('normalises every quaternion component and ignores a partial trailing entity', () => {
        const r = recorder();
        decodeTransforms(source(frame([0, 0, 0, 2, 2, 2, 2])), 1, r.sink);
        expect([r.shown[0].qx, r.shown[0].qy, r.shown[0].qz, r.shown[0].qw].every((q) => Math.abs(q - 0.5) < 1e-6)).toBe(true);
        const ragged = new Float32Array(10);
        ragged.set(ID, 0);
        expect(decodeTransforms({ readTransforms: () => ragged }, 4, recorder().sink)).toEqual({ live: 1, hidden: 0, truncated: 0 });
    });
});
