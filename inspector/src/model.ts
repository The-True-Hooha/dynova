import {
    BODY_STATE_AWAKE, BODY_STATE_INACTIVE, BODY_STATE_SLEEPING, BODY_STATE_STATIC,
    type InspectedBody, type InspectorSnapshot, type ProfilerSample,
} from '@dynova/sdk';

export type Vec3 = [number, number, number];

export interface ProfilerRates {
    /** Simulation steps per second over the sample window; `null` while no time has passed or no step ran. */
    ticksPerSecond: number | null;
    /** Mean step duration over the window in milliseconds; `null` when no step ran in it. */
    avgStepMs: number | null;
}

interface Timed {
    sample: ProfilerSample;
    atMs: number;
}

/** Sliding window of profiler samples; rates come from the oldest and newest sample, so they are means, not spikes. */
export class ProfilerWindow {
    private samples: Timed[] = [];

    constructor(private readonly maxSamples = 8) {
        if (!Number.isInteger(maxSamples) || maxSamples < 2) throw new RangeError('a profiler window needs at least 2 samples');
    }

    push(sample: ProfilerSample, atMs: number): void {
        const last = this.samples[this.samples.length - 1];
        // A tick counter that moves backwards is a different world (restart): its history says nothing about this one.
        if (last && ((sample.ticks - last.sample.ticks) >>> 0) > 0x80000000) this.samples = [];
        this.samples.push({ sample, atMs });
        if (this.samples.length > this.maxSamples) this.samples.shift();
    }

    rates(): ProfilerRates {
        const first = this.samples[0];
        const last = this.samples[this.samples.length - 1];
        if (!first || first === last) return { ticksPerSecond: null, avgStepMs: null };
        const ticks = (last.sample.ticks - first.sample.ticks) >>> 0;
        const seconds = (last.atMs - first.atMs) / 1000;
        const micros = (last.sample.stepTotalMicros - first.sample.stepTotalMicros) >>> 0;
        return {
            ticksPerSecond: ticks > 0 && seconds > 0 ? ticks / seconds : null,
            avgStepMs: ticks > 0 ? micros / ticks / 1000 : null,
        };
    }
}

export const STATE_NAMES = ['unused', 'awake', 'sleeping', 'static'] as const;

export function stateName(state: number): string {
    return STATE_NAMES[state] ?? `state ${state}`;
}

export interface EntityRow {
    id: number;
    state: number;
    position: Vec3;
}

export interface EntityList {
    rows: EntityRow[];
    /** Entities matching the filter, before the row limit. */
    total: number;
}

/** Entities that exist (not unused slots), optionally only one state, first `limit` by id. */
export function listEntities(snapshot: InspectorSnapshot, options: { state?: number | null; limit: number }): EntityList {
    const wanted = options.state ?? null;
    const rows: EntityRow[] = [];
    let total = 0;
    for (let id = 0; id < snapshot.capacity; id++) {
        const body = snapshot.body(id);
        if (!body || body.state === BODY_STATE_INACTIVE) continue;
        if (wanted !== null && body.state !== wanted) continue;
        total++;
        if (rows.length < options.limit) rows.push({ id, state: body.state, position: body.position });
    }
    return { rows, total };
}

export function stateCounts(snapshot: InspectorSnapshot): Record<'awake' | 'sleeping' | 'static', number> {
    const counts = { awake: 0, sleeping: 0, static: 0 };
    for (let id = 0; id < snapshot.capacity; id++) {
        const state = snapshot.body(id)!.state;
        if (state === BODY_STATE_AWAKE) counts.awake++;
        else if (state === BODY_STATE_SLEEPING) counts.sleeping++;
        else if (state === BODY_STATE_STATIC) counts.static++;
    }
    return counts;
}

export interface EntityContact {
    other: number;
    point: Vec3;
    normal: Vec3;
    depth: number;
}

/** Contacts touching `id`; `normal` is the direction the contact pushes `id` (the solver's normal points from b to a). */
export function contactsOf(snapshot: InspectorSnapshot, id: number): EntityContact[] {
    const out: EntityContact[] = [];
    for (let k = 0; k < snapshot.contactCount; k++) {
        const c = snapshot.contact(k);
        if (c.a === id) out.push({ other: c.b, point: c.point, normal: c.normal, depth: c.depth });
        else if (c.b === id) out.push({ other: c.a, point: c.point, normal: [-c.normal[0], -c.normal[1], -c.normal[2]], depth: c.depth });
    }
    return out;
}

export function speed(body: InspectedBody): number {
    return Math.hypot(...body.velocity);
}

export function fmt(value: number, digits = 3): string {
    if (!Number.isFinite(value)) return '—';
    const text = value.toFixed(digits);
    return /^-0\.?0*$/.test(text) ? text.slice(1) : text;
}

export function fmtVec(v: readonly number[], digits = 3): string {
    return v.map((x) => fmt(x, digits)).join(', ');
}

export function fmtBytes(bytes: number): string {
    return `${(bytes / (1024 * 1024)).toFixed(1)} MiB`;
}
