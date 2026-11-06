import { describe, it, expect } from 'vitest';
import { canvas2dAdapter, type Canvas2DContext } from '../canvas2dAdapter.js';

type Call = [string, ...unknown[]];

function fakeContext() {
    const calls: Call[] = [];
    const ctx: Canvas2DContext = {
        fillStyle: '',
        save: () => calls.push(['save']),
        restore: () => calls.push(['restore']),
        translate: (x, y) => calls.push(['translate', x, y]),
        rotate: (a) => calls.push(['rotate', a]),
        fillRect: (x, y, w, h) => calls.push(['fillRect', x, y, w, h]),
        beginPath: () => calls.push(['beginPath']),
        arc: (x, y, r) => calls.push(['arc', x, y, r]),
        fill: () => calls.push(['fill']),
    };
    const styles: unknown[] = [];
    Object.defineProperty(ctx, 'fillStyle', { get: () => styles[styles.length - 1], set: (v) => { styles.push(v); } });
    return { ctx, calls, styles };
}

const frame = (...entities: number[][]) => {
    const buf = new Float32Array(entities.length * 7);
    entities.forEach((e, i) => buf.set(e, i * 7));
    return { readTransforms: () => buf };
};
const ID = [0, 0, 0, 0, 0, 0, 1];
const names = (calls: Call[]) => calls.map((c) => c[0]);

describe('canvas2dAdapter', () => {
    it('maps world +y up the screen: translate to origin + (x, -y) * pixelsPerUnit', () => {
        const { ctx, calls } = fakeContext();
        canvas2dAdapter(frame([2, 3, 0, 0, 0, 0, 1]), ctx, { view: { pixelsPerUnit: 10, originX: 100, originY: 200 } }).update();
        expect(calls.find((c) => c[0] === 'translate')).toEqual(['translate', 120, 170]);
    });

    it('rotation about z turns the drawn shape the other way on screen', () => {
        const { ctx, calls } = fakeContext();
        const h = Math.SQRT1_2;
        canvas2dAdapter(frame([0, 0, 0, 0, 0, h, h]), ctx).update();
        const rotate = calls.find((c) => c[0] === 'rotate')!;
        expect(rotate[1] as number).toBeCloseTo(-Math.PI / 2, 6);
    });

    it('draws a rectangle of scale * pixelsPerUnit centred on the entity, and a circle when styled', () => {
        const { ctx, calls } = fakeContext();
        canvas2dAdapter(frame(ID, ID), ctx, {
            view: { pixelsPerUnit: 10 },
            scales: [2, 4, 1, 6, 6, 1],
            style: (id) => (id === 1 ? { shape: 'circle' } : {}),
        }).update();
        expect(calls.find((c) => c[0] === 'fillRect')).toEqual(['fillRect', -10, -20, 20, 40]);
        expect(calls.find((c) => c[0] === 'arc')).toEqual(['arc', 0, 0, 30]);
        expect(names(calls).filter((n) => n === 'beginPath')).toHaveLength(1);
    });

    it('sets the style colour per entity and defaults to grey', () => {
        const { ctx, styles } = fakeContext();
        canvas2dAdapter(frame(ID, ID), ctx, { style: (id) => (id === 0 ? { color: '#ff0000' } : {}) }).update();
        expect(styles).toEqual(['#ff0000', '#888888']);
    });

    it('skips destroyed and non-finite entities, balances save/restore, and returns the shared counts', () => {
        const { ctx, calls } = fakeContext();
        const res = canvas2dAdapter(frame([0, 0, 0, 0, 0, 0, 0], [NaN, 0, 0, 0, 0, 0, 1], ID), ctx).update();
        expect(res).toEqual({ live: 1, hidden: 2, truncated: 0 });
        expect(names(calls).filter((n) => n === 'save')).toHaveLength(1);
        expect(names(calls).filter((n) => n === 'restore')).toHaveLength(1);
        expect(names(calls).filter((n) => n === 'fillRect')).toHaveLength(1);
    });

    it('idOffset skips leading entities; scales and style receive the entity id', () => {
        const { ctx, calls } = fakeContext();
        const seen: number[] = [];
        canvas2dAdapter(frame(ID, ID, ID), ctx, { idOffset: 2, style: (id) => { seen.push(id); return {}; }, scales: [1, 1, 1, 1, 1, 1, 3, 5, 1] }).update();
        expect(seen).toEqual([2]);
        expect(calls.find((c) => c[0] === 'fillRect')).toEqual(['fillRect', -30, -50, 60, 100]);
    });

    it('draws every entity with no fixed slot limit', () => {
        const { ctx } = fakeContext();
        const many = Array.from({ length: 500 }, () => ID);
        expect(canvas2dAdapter(frame(...many), ctx).update()).toEqual({ live: 500, hidden: 0, truncated: 0 });
    });

    it('keeps no state between updates: a later frame draws the new poses', () => {
        const { ctx, calls } = fakeContext();
        let buf = frame([1, 0, 0, 0, 0, 0, 1]);
        const adapter = canvas2dAdapter({ readTransforms: () => buf.readTransforms() }, ctx, { view: { pixelsPerUnit: 10 } });
        adapter.update();
        buf = frame([5, 0, 0, 0, 0, 0, 1]);
        adapter.update();
        expect(calls.filter((c) => c[0] === 'translate').map((c) => c[1])).toEqual([10, 50]);
    });

    it.each([[0], [-5], [NaN], [Infinity]])('rejects pixelsPerUnit %s', (bad) => {
        expect(() => canvas2dAdapter(frame(ID), fakeContext().ctx, { view: { pixelsPerUnit: bad } })).toThrow(RangeError);
    });

    it('rejects a non-finite origin and a bad idOffset at construction', () => {
        expect(() => canvas2dAdapter(frame(ID), fakeContext().ctx, { view: { originX: NaN } })).toThrow(RangeError);
        expect(() => canvas2dAdapter(frame(ID), fakeContext().ctx, { idOffset: -1 })).toThrow(RangeError);
    });

    it('a circle takes its diameter from scale x only and survives a negative scale', () => {
        const { ctx, calls } = fakeContext();
        canvas2dAdapter(frame(ID, ID), ctx, { view: { pixelsPerUnit: 10 }, scales: [2, 6, 1, -4, 1, 1], style: () => ({ shape: 'circle' }) }).update();
        expect(calls.filter((c) => c[0] === 'arc')).toEqual([['arc', 0, 0, 10], ['arc', 0, 0, 20]]);
    });

    it('restores the context even when drawing throws', () => {
        const { ctx, calls } = fakeContext();
        ctx.fillRect = () => { throw new Error('boom'); };
        expect(() => canvas2dAdapter(frame(ID), ctx).update()).toThrow('boom');
        expect(names(calls).filter((n) => n === 'save')).toHaveLength(1);
        expect(names(calls).filter((n) => n === 'restore')).toHaveLength(1);
    });

    it('draws the projected x axis angle for a general rotation, not just rotation about z', () => {
        const q = [0.3, -0.2, 0.5, 0.78];
        const l = Math.hypot(...q);
        const [x, y, z, w] = q.map((v) => v / l);
        const vx = 1 - 2 * (y * y + z * z), vy = 2 * (x * y + w * z);
        const { ctx, calls } = fakeContext();
        canvas2dAdapter(frame([0, 0, 0, ...q]), ctx).update();
        expect(calls.find((c) => c[0] === 'rotate')![1] as number).toBeCloseTo(-Math.atan2(vy, vx), 5);
    });

    it('defaults to 20 pixels per unit with the origin at the canvas corner', () => {
        const { ctx, calls } = fakeContext();
        canvas2dAdapter(frame([1, 1, 0, 0, 0, 0, 1]), ctx).update();
        expect(calls.find((c) => c[0] === 'translate')).toEqual(['translate', 20, -20]);
        expect(calls.find((c) => c[0] === 'fillRect')).toEqual(['fillRect', -10, -10, 20, 20]);
    });
});
