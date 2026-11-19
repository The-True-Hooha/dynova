/** Joint targets for a four-legged robot whose joints are named `<leg>_abduct`, `<leg>_hip_pitch` and `<leg>_knee` for the legs `fl`, `fr`, `hl` and `hr`. Pure, so a gait can be tested without a world. */

export const LEGS = ['fl', 'fr', 'hl', 'hr'] as const;
export type Leg = (typeof LEGS)[number];

export interface TrotOptions {
    /** Swing of the thigh either side of standing, radians. */
    stride: number;
    /** How far the knee folds to lift a foot, radians. */
    lift: number;
    /** Steps per second of each leg. */
    frequency: number;
    /** Seconds over which the gait is eased in from standing. */
    ramp: number;
}

export const DEFAULT_TROT: TrotOptions = { stride: 0.3, lift: 0.6, frequency: 1.5, ramp: 1 };

/** The leg pairs that move together: a trot lifts the diagonal pairs in turn. */
const PHASE: Record<Leg, number> = { fl: 0, hr: 0, fr: 0.5, hl: 0.5 };

/** All joints at 0: the standing pose of a description whose zero pose is standing. */
export function standing(joints: readonly string[]): Record<string, number> {
    return Object.fromEntries(joints.map((j) => [j, 0]));
}

/**
 * Targets at `time` seconds into a trot. Each leg runs a cycle: the thigh sweeps back while the foot is down, then forward while the knee folds to
 * carry the foot through the air. The gait starts from standing and reaches full amplitude after `ramp` seconds. A joint that is not one of the twelve
 * leg joints gets 0 (the abduction joints are never driven), and a list with no leg joint at all is an error.
 */
export function trot(joints: readonly string[], time: number, options: Partial<TrotOptions> = {}): Record<string, number> {
    const o = { ...DEFAULT_TROT, ...options };
    if (![time, o.stride, o.lift, o.frequency, o.ramp].every(Number.isFinite) || time < 0 || o.ramp < 0 || o.frequency <= 0) throw new RangeError('a trot needs a finite, non-negative time, a positive frequency and finite options');
    const ease = o.ramp === 0 ? 1 : Math.min(1, time / o.ramp);
    const out: Record<string, number> = {};
    let matched = false;
    for (const name of joints) {
        out[name] = 0;
        const m = /^(fl|fr|hl|hr)_(abduct|hip_pitch|knee)$/.exec(name);
        if (!m) continue;
        matched = true;
        const leg = m[1] as Leg;
        const p = (o.frequency * time + PHASE[leg]) % 1;
        if (m[2] === 'hip_pitch') out[name] = -o.stride * Math.cos(2 * Math.PI * p) * ease;
        else if (m[2] === 'knee' && p > 0.5) out[name] = -o.lift * Math.sin(2 * Math.PI * (p - 0.5)) * ease;
    }
    if (!matched) throw new RangeError('no joint is named <fl|fr|hl|hr>_<abduct|hip_pitch|knee>, so there is no leg to move');
    return out;
}
