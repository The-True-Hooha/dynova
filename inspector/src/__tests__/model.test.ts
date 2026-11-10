import { describe, expect, it } from 'vitest';
import type { InspectedBody, InspectedContact, InspectedJoint, InspectorSnapshot, ProfilerSample, RecordingState } from '@dynova/sdk';
import { ProfilerWindow, attempt, parseCoordinate, stepTarget, timelineView, contactsOf, jointGapLabel, jointKindName, jointsOf, fmt, fmtBytes, fmtVec, listEntities, speed, stateCounts, stateName } from '../model.js';

const sample = (ticks: number, totalMicros: number, extra: Partial<ProfilerSample> = {}): ProfilerSample => ({
    ticks, stepLastMicros: 0, stepTotalMicros: totalMicros, ringPending: 0, contacts: 0, droppedContacts: 0, workerOverruns: 0,
    memoryBytes: 0, growths: 0, snapshotAgeTicks: 0, ...extra,
});

function snapshot(states: number[], contacts: InspectedContact[] = [], joints: InspectedJoint[] = []): InspectorSnapshot {
    const bodies: InspectedBody[] = states.map((state, id) => ({
        id, state, position: [id, 0, 0], rotation: [0, 0, 0, 1], velocity: [3, 4, 0], angularVelocity: [0, 0, 0],
    }));
    return {
        tick: 1,
        capacity: states.length,
        contactCount: contacts.length,
        body: (id) => bodies[id] ?? null,
        contact: (k) => contacts[k],
        jointCount: joints.length,
        joint: (k) => joints[k],
        idsInState: (s) => bodies.filter((b) => b.state === s).map((b) => b.id),
    };
}

describe('ProfilerWindow', () => {
    it('needs two samples before it reports a rate', () => {
        const w = new ProfilerWindow();
        expect(w.rates()).toEqual({ ticksPerSecond: null, avgStepMs: null });
        w.push(sample(10, 0), 0);
        expect(w.rates()).toEqual({ ticksPerSecond: null, avgStepMs: null });
    });

    it('derives ticks per second and the mean step time from the window ends', () => {
        const w = new ProfilerWindow();
        w.push(sample(100, 1_000_000), 0);
        w.push(sample(130, 1_000_000 + 30 * 2500), 500);
        expect(w.rates()).toEqual({ ticksPerSecond: 60, avgStepMs: 2.5 });
    });

    it('reports nothing for a paused simulation instead of dividing by zero', () => {
        const w = new ProfilerWindow();
        w.push(sample(100, 5), 0);
        w.push(sample(100, 5), 1000);
        expect(w.rates()).toEqual({ ticksPerSecond: null, avgStepMs: null });
        const same = new ProfilerWindow();
        same.push(sample(1, 0), 10);
        same.push(sample(2, 1000), 10);
        expect(same.rates().ticksPerSecond).toBeNull();
        expect(same.rates().avgStepMs).toBe(1);
    });

    it('handles the wrapping 32-bit tick and step-total counters', () => {
        const w = new ProfilerWindow();
        w.push(sample(0xfffffff0, 0xfffffc00), 0);
        w.push(sample(0x00000010, 0x00000c00 + 0x400), 1000);
        const r = w.rates();
        expect(r.ticksPerSecond).toBe(32);
        expect(r.avgStepMs).toBeCloseTo(((0x400 + 0xc00 + 0x400) / 32) / 1000, 6);
    });

    it('forgets history when the tick counter moves backwards (a different world)', () => {
        const w = new ProfilerWindow();
        w.push(sample(5000, 1_000_000), 0);
        w.push(sample(5060, 1_100_000), 1000);
        w.push(sample(3, 100), 2000);
        expect(w.rates()).toEqual({ ticksPerSecond: null, avgStepMs: null });
        w.push(sample(63, 150_100), 3000);
        expect(w.rates().ticksPerSecond).toBe(60);
        expect(w.rates().avgStepMs).toBe(2.5);
    });

    it('keeps only the newest samples so old behaviour ages out', () => {
        const w = new ProfilerWindow(3);
        w.push(sample(0, 0), 0);
        w.push(sample(60, 0), 1000);
        w.push(sample(70, 0), 2000);
        w.push(sample(80, 0), 3000);
        expect(w.rates().ticksPerSecond).toBe(10);
    });

    it('rejects windows that cannot produce a rate', () => {
        for (const n of [0, 1, 2.5, NaN]) expect(() => new ProfilerWindow(n)).toThrow(RangeError);
    });
});

describe('entity queries', () => {
    const snap = snapshot([3, 1, 2, 0, 1, 1]);

    it('lists existing entities only and counts the total before the limit', () => {
        const all = listEntities(snap, { limit: 100 });
        expect(all.rows.map((r) => r.id)).toEqual([0, 1, 2, 4, 5]);
        expect(all.total).toBe(5);
        const limited = listEntities(snap, { limit: 2 });
        expect(limited.rows.map((r) => r.id)).toEqual([0, 1]);
        expect(limited.total).toBe(5);
        expect(listEntities(snap, { limit: 0 })).toEqual({ rows: [], total: 5 });
    });

    it('filters by state', () => {
        expect(listEntities(snap, { state: 1, limit: 10 }).rows.map((r) => r.id)).toEqual([1, 4, 5]);
        expect(listEntities(snap, { state: 2, limit: 10 }).total).toBe(1);
        expect(listEntities(snap, { state: null, limit: 10 }).total).toBe(5);
        expect(listEntities(snapshot([]), { limit: 10 })).toEqual({ rows: [], total: 0 });
    });

    it('counts awake, sleeping and static bodies, ignoring unused slots', () => {
        expect(stateCounts(snap)).toEqual({ awake: 3, sleeping: 1, static: 1 });
        expect(stateCounts(snapshot([0, 0]))).toEqual({ awake: 0, sleeping: 0, static: 0 });
    });

    it('names states and survives an unknown one', () => {
        expect([0, 1, 2, 3].map(stateName)).toEqual(['unused', 'awake', 'sleeping', 'static']);
        expect(stateName(9)).toBe('state 9');
    });

    it('speed is the length of the velocity', () => {
        expect(speed(snap.body(1)!)).toBe(5);
    });
});

describe('contactsOf', () => {
    const contacts: InspectedContact[] = [
        { point: [0, 0, 0], normal: [0, 1, 0], depth: 0.1, a: 4, b: 0 },
        { point: [1, 0, 0], normal: [1, 0, 0], depth: 0.2, a: 2, b: 4 },
        { point: [9, 0, 0], normal: [0, 0, 1], depth: 0.3, a: 7, b: 8 },
    ];
    const snap = snapshot([3, 1, 1, 0, 1, 0, 0, 1, 1], contacts);

    it('finds contacts on either side and points the normal in the direction that pushes the entity', () => {
        expect(contactsOf(snap, 4)).toEqual([
            { other: 0, point: [0, 0, 0], normal: [0, 1, 0], depth: 0.1 },
            { other: 2, point: [1, 0, 0], normal: [-1, -0, -0], depth: 0.2 },
        ]);
        expect(contactsOf(snap, 0)[0].normal).toEqual([-0, -1, -0]);
    });

    it('returns nothing for an entity without contacts or an unknown id', () => {
        expect(contactsOf(snap, 1)).toEqual([]);
        expect(contactsOf(snap, 99)).toEqual([]);
    });
});

describe('jointsOf', () => {
    const joints: InspectedJoint[] = [
        { id: 1, kind: 1, a: 4, b: 0, anchorA: [0, 0, 0], anchorB: [0, 0, 0] },
        { id: 2, kind: 3, a: 2, b: 4, anchorA: [1, 2, 2], anchorB: [0, 0, 0] },
        { id: 3, kind: 0, a: 7, b: 8, anchorA: [0, 0, 0], anchorB: [0, 0, 0] },
    ];
    const snap = snapshot([3, 1, 1, 0, 1, 0, 0, 1, 1], [], joints);

    it('finds joints on either side, names the other body and measures the anchor gap', () => {
        expect(jointsOf(snap, 4)).toEqual([
            { id: 1, kind: 1, other: 0, gap: 0 },
            { id: 2, kind: 3, other: 2, gap: 3 },
        ]);
        expect(jointsOf(snap, 0)).toEqual([{ id: 1, kind: 1, other: 4, gap: 0 }]);
    });

    it('returns nothing for an entity without joints, an unknown id or a snapshot with none', () => {
        expect(jointsOf(snap, 1)).toEqual([]);
        expect(jointsOf(snap, 99)).toEqual([]);
        expect(jointsOf(snapshot([1, 1]), 0)).toEqual([]);
    });

    it('calls a slider\'s anchor distance travel and every other joint\'s gap', () => {
        expect([0, 1, 2, 3].map(jointGapLabel)).toEqual(['gap', 'gap', 'gap', 'travel']);
    });

    it('names the four kinds and falls back for an unknown one', () => {
        expect([0, 1, 2, 3].map(jointKindName)).toEqual(['ball', 'hinge', 'fixed', 'slider']);
        expect(jointKindName(9)).toBe('kind 9');
    });
});

describe('formatting', () => {
    it('shows a dash for non-finite numbers and never prints negative zero', () => {
        expect(fmt(NaN)).toBe('—');
        expect(fmt(Infinity)).toBe('—');
        expect(fmt(-0)).toBe('0.000');
        expect(fmt(-0.0004)).toBe('0.000');
        expect(fmt(-0.0006)).toBe('-0.001');
        expect(fmt(1.23456, 2)).toBe('1.23');
        expect(fmt(-5, 0)).toBe('-5');
        expect(fmt(-0.4, 0)).toBe('0');
    });

    it('joins vectors and formats memory sizes', () => {
        expect(fmtVec([1, -0, 2.5], 1)).toBe('1.0, 0.0, 2.5');
        expect(fmtBytes(16 * 1024 * 1024)).toBe('16.0 MiB');
    });
});

const rec = (extra: Partial<RecordingState> = {}): RecordingState => ({
    on: true, paused: false, tick: 50, start: 10, end: 90, keyframes: 2, hash: 0xabcn, seeks: 0, ...extra,
});

describe('timelineView', () => {
    it('shows a scrubber over the recorded range with the hash in hex', () => {
        expect(timelineView(rec())).toEqual({
            scrubbable: true, min: 10, max: 90, value: 50,
            label: 'tick 50 of 10–90, 2 keyframes, hash 0000000000000abc',
        });
    });

    it('says when the clock is paused', () => {
        expect(timelineView(rec({ paused: true })).label).toMatch(/, paused$/);
    });

    it('pads a full 64-bit hash without losing bits', () => {
        expect(timelineView(rec({ hash: 0xfedcba9876543210n })).label).toMatch(/hash fedcba9876543210/);
    });

    it('disables the scrubber when nothing is recorded, the read failed, or the range is a single tick', () => {
        expect(timelineView(null)).toMatchObject({ scrubbable: false, label: 'recording state unavailable' });
        expect(timelineView(rec({ on: false }))).toMatchObject({ scrubbable: false, label: 'not recording' });
        expect(timelineView(rec({ start: 20, end: 20, tick: 20 })).scrubbable).toBe(false);
    });

    it('keeps the thumb inside the range even if the tick is momentarily outside it', () => {
        expect(timelineView(rec({ tick: 5 })).value).toBe(10);
        expect(timelineView(rec({ tick: 500 })).value).toBe(90);
    });
});

describe('stepTarget', () => {
    it('moves one tick inside the range and refuses to leave it', () => {
        expect(stepTarget(rec(), -1)).toBe(49);
        expect(stepTarget(rec(), 1)).toBe(51);
        expect(stepTarget(rec({ tick: 10 }), -1)).toBeNull();
        expect(stepTarget(rec({ tick: 90 }), 1)).toBeNull();
        expect(stepTarget(rec({ tick: 10 }), 1)).toBe(11);
    });

    it('reaches the first and last recorded tick but not beyond', () => {
        expect(stepTarget(rec({ tick: 11 }), -1)).toBe(10);
        expect(stepTarget(rec({ tick: 89 }), 1)).toBe(90);
    });

    it('has no target without a recording', () => {
        expect(stepTarget(null, 1)).toBeNull();
        expect(stepTarget(rec({ on: false }), 1)).toBeNull();
    });
});

describe('parseCoordinate', () => {
    it('accepts plain, signed, decimal and exponent numbers with surrounding spaces', () => {
        expect(parseCoordinate(' 3.5 ')).toBe(3.5);
        expect(parseCoordinate('-2')).toBe(-2);
        expect(parseCoordinate('1e3')).toBe(1000);
        expect(parseCoordinate('0')).toBe(0);
    });

    it('rejects empty, non-numeric and non-finite text instead of turning it into 0 or NaN', () => {
        for (const bad of ['', '   ', 'abc', '1,5', 'Infinity', '-Infinity', 'NaN', '1 2']) expect(parseCoordinate(bad)).toBeNull();
    });
});

describe('attempt', () => {
    it('returns null when the callback succeeds and the message when it throws', () => {
        expect(attempt(() => undefined)).toBeNull();
        expect(attempt(() => { throw new Error('CommandRing full'); })).toBe('CommandRing full');
        expect(attempt(() => { throw new RangeError('bad tick'); })).toBe('bad tick');
        expect(attempt(() => { throw 'plain'; })).toBe('plain');
    });
});
