import { JOINT_BALL, JOINT_DISTANCE, JOINT_FIXED, JOINT_HINGE, JOINT_SLIDER } from './CommandRing.js';
import type { PositionGains } from './control.js';
import type { EntityHandle } from './EntityIdAllocator.js';
import type { JointHandle, JointState, SpawnOptions, World } from './World.js';

export type V3 = readonly [number, number, number];

/** Where two parts meet, in the assembly's own coordinates (the same ones the parts were added in). */
export type MateSpec =
    | { type: 'fixed'; a: string; b: string; at: V3 }
    | { type: 'ball'; a: string; b: string; at: V3 }
    | { type: 'revolute'; a: string; b: string; at: V3; axis: V3; range?: readonly [number, number] }
    | { type: 'slider'; a: string; b: string; at: V3; axis: V3; range?: readonly [number, number] }
    | { type: 'distance'; a: string; b: string; pointA: V3; pointB: V3; range?: readonly [number, number] };

export interface MateState {
    /** Hinge angle in radians, slider offset in metres, or the stretch of a distance mate; zero in the pose the mate was made in. */
    position: number;
    velocity: number;
}

function sub3(p: V3, o: V3): [number, number, number] {
    return [p[0] - o[0], p[1] - o[1], p[2] - o[2]];
}

/**
 * Named parts and mates over a world, in coordinates of the caller's choosing. The simulation stores 32-bit floats, which resolve about
 * 8 mm at 100 km from the origin and only 3 cm at 500 km, so a model that sits far from the origin loses its small motions and its
 * mates drift. An assembly keeps an `origin` in 64-bit floats and subtracts it before anything reaches the runtime: pick the centre of
 * the model and the parts are simulated near zero whatever the original coordinates were. The origin is fixed for the life of the
 * assembly (re-basing a running world would have to move every body and keep their velocities, which no command does).
 */
export class Assembly {
    private readonly parts = new Map<string, EntityHandle>();
    private readonly mates = new Map<string, JointHandle>();
    readonly origin: [number, number, number];

    constructor(private readonly world: World, origin: V3 = [0, 0, 0]) {
        if (origin.length !== 3 || !origin.every(Number.isFinite)) throw new RangeError('assembly origin must be three finite numbers');
        this.origin = [origin[0], origin[1], origin[2]];
    }

    /** A point of the assembly's coordinates as the runtime stores it: the origin removed in 64 bits, then rounded to 32. The ring rounds to 32 bits anyway; doing it here keeps the value the caller sees equal to the value the world holds. */
    toLocal(p: V3): [number, number, number] {
        const d = sub3(p, this.origin);
        return [Math.fround(d[0]), Math.fround(d[1]), Math.fround(d[2])];
    }

    toGlobal(local: V3): [number, number, number] {
        return [local[0] + this.origin[0], local[1] + this.origin[1], local[2] + this.origin[2]];
    }

    part(name: string): EntityHandle {
        const h = this.parts.get(name);
        if (!h) throw new Error(`no part named '${name}'`);
        return h;
    }

    mate(name: string): JointHandle {
        const h = this.mates.get(name);
        if (!h) throw new Error(`no mate named '${name}'`);
        return h;
    }

    get partNames(): string[] { return [...this.parts.keys()]; }
    get mateNames(): string[] { return [...this.mates.keys()]; }

    /** `options.x/y/z` are in the assembly's coordinates; everything else (size, mass, orientation) is passed through. */
    addPart(name: string, options: SpawnOptions = {}): EntityHandle {
        if (this.parts.has(name)) throw new Error(`part '${name}' already exists`);
        const [x, y, z] = this.toLocal([options.x ?? 0, options.y ?? 0, options.z ?? 0]);
        const handle = this.world.spawn({ ...options, x, y, z });
        this.parts.set(name, handle);
        return handle;
    }

    /** Joins two parts in the pose they are in now; ranges are measured from that pose. */
    addMate(name: string, spec: MateSpec): JointHandle {
        if (this.mates.has(name)) throw new Error(`mate '${name}' already exists`);
        const a = this.part(spec.a), b = this.part(spec.b);
        let joint: JointHandle;
        switch (spec.type) {
            case 'fixed': joint = this.world.createJoint({ kind: JOINT_FIXED, a, b, anchor: this.toLocal(spec.at) }); break;
            case 'ball': joint = this.world.createJoint({ kind: JOINT_BALL, a, b, anchor: this.toLocal(spec.at) }); break;
            case 'revolute': joint = this.world.createJoint({ kind: JOINT_HINGE, a, b, anchor: this.toLocal(spec.at), axis: spec.axis, limits: spec.range }); break;
            case 'slider': joint = this.world.createJoint({ kind: JOINT_SLIDER, a, b, anchor: this.toLocal(spec.at), axis: spec.axis, limits: spec.range }); break;
            case 'distance': joint = this.world.createJoint({ kind: JOINT_DISTANCE, a, b, anchor: this.toLocal(spec.pointA), anchorB: this.toLocal(spec.pointB), limits: spec.range }); break;
            default: throw new RangeError(`unknown mate type '${(spec as { type: string }).type}'`);
        }
        this.mates.set(name, joint);
        return joint;
    }

    /** Position and rate of every mate that has a free coordinate (revolute, slider, distance), by name, as of the last step. */
    states(): Record<string, MateState> {
        const byJoint = new Map(this.world.readJointStates().map((s: JointState) => [s.joint, s]));
        const out: Record<string, MateState> = {};
        for (const [name, joint] of this.mates) {
            const s = byJoint.get(joint.id);
            if (s) out[name] = { position: s.position, velocity: s.velocity };
        }
        return out;
    }

    /** One control tick towards `targets` (mate name to position) with the same gains for each; returns the names it had no state for. */
    drive(targets: Readonly<Record<string, number>>, gains: PositionGains): string[] {
        const names = Object.keys(targets);
        const skipped = this.world.driveJoints(names.map((n) => ({ joint: this.mate(n), target: targets[n], gains })));
        return names.filter((n) => skipped.includes(this.mate(n).id));
    }
}
