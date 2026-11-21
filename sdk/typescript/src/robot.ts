import { parseXml, type XmlElement } from './xml.js';
import { SHAPE_BOX, SHAPE_CAPSULE, SHAPE_SPHERE, JOINT_FIXED, JOINT_HINGE, JOINT_SLIDER, MAX_ANGLE_LIMIT, MAX_LINEAR_LIMIT } from './CommandRing.js';
import type { World, JointHandle } from './World.js';
import type { EntityHandle } from './EntityIdAllocator.js';

export type V3 = [number, number, number];
/** Unit quaternion (x, y, z, w). */
export type Quat = [number, number, number, number];
export interface Frame { p: V3; q: Quat }

/** One collision shape of a link, in the link's frame. A box has `half`; a sphere has `radius`; a capsule runs along its local Y with `radius` and `halfLength` of the straight part. */
export interface ShapeDesc {
    kind: 'box' | 'sphere' | 'capsule';
    half: V3;
    radius: number;
    halfLength: number;
    frame: Frame;
}

export interface LinkDesc {
    name: string;
    /** Kilograms, or `null` when the description gives none. */
    mass: number | null;
    shape: ShapeDesc | null;
    /** A body that never moves, whatever `fixedBase` says (MJCF scenery). */
    static?: boolean;
}

export type JointType = 'revolute' | 'continuous' | 'prismatic' | 'fixed';

export interface RobotJointDesc {
    name: string;
    type: JointType;
    parent: string;
    child: string;
    /** The joint frame in the parent link's frame, at joint value 0. */
    origin: Frame;
    /** In the joint frame. */
    axis: V3;
    limit: { lower: number; upper: number } | null;
}

/** A robot as the description states it. `joints` are in the file's order; `order` lists them parent before child. */
export interface RobotDesc {
    name: string;
    root: string;
    links: LinkDesc[];
    joints: RobotJointDesc[];
    order: RobotJointDesc[];
    /** What the importer could not honour: dropped shapes, clamped limits, defaulted masses. */
    warnings: string[];
}

export class RobotError extends Error {}

// ---- small exact-enough maths (host side, f64) ----
const IDENTITY: Frame = { p: [0, 0, 0], q: [0, 0, 0, 1] };
export function qmul(a: Quat, b: Quat): Quat {
    return [
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    ];
}
export function qrot(q: Quat, v: readonly number[]): V3 {
    const [x, y, z, w] = q;
    const t = [2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])];
    return [v[0] + w * t[0] + (y * t[2] - z * t[1]), v[1] + w * t[1] + (z * t[0] - x * t[2]), v[2] + w * t[2] + (x * t[1] - y * t[0])];
}
export function compose(a: Frame, b: Frame): Frame {
    const r = qrot(a.q, b.p);
    return { p: [a.p[0] + r[0], a.p[1] + r[1], a.p[2] + r[2]], q: qmul(a.q, b.q) };
}
export function axisAngle(axis: readonly number[], angle: number): Quat {
    const s = Math.sin(angle / 2) / Math.hypot(axis[0], axis[1], axis[2]);
    return [axis[0] * s, axis[1] * s, axis[2] * s, Math.cos(angle / 2)];
}
/** URDF roll-pitch-yaw: rotate about fixed X, then Y, then Z. */
export function rpyToQuat(r: number, p: number, y: number): Quat {
    return qmul(qmul(axisAngle([0, 0, 1], y), axisAngle([0, 1, 0], p)), axisAngle([1, 0, 0], r));
}

// ---- parsing ----
function numbers(text: string | undefined, count: number, what: string, fallback?: number[]): number[] {
    if (text === undefined) {
        if (fallback) return fallback;
        throw new RobotError(`${what} is missing`);
    }
    const parts = text.trim().split(/\s+/);
    // Number() would also take 0x10, 0b1 and the like, which no description means.
    const values = parts.map((s) => (/^[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?$/.test(s) ? Number(s) : NaN));
    if (parts.length !== count || values.some((v) => !Number.isFinite(v))) throw new RobotError(`${what} needs ${count} finite numbers, got "${text}"`);
    return values;
}

function number1(text: string | undefined, what: string, positive = false): number {
    const [v] = numbers(text, 1, what);
    if (positive && !(v > 0)) throw new RobotError(`${what} must be positive, got ${v}`);
    return v;
}

function frameOf(el: XmlElement | undefined, what: string): Frame {
    if (!el) return IDENTITY;
    const [x, y, z] = numbers(el.attrs.xyz, 3, `${what} xyz`, [0, 0, 0]);
    const [r, p, w] = numbers(el.attrs.rpy, 3, `${what} rpy`, [0, 0, 0]);
    return { p: [x, y, z], q: rpyToQuat(r, p, w) };
}

const child = (el: XmlElement, name: string) => el.children.find((c) => c.name === name);

function shapeOf(link: string, collision: XmlElement, warnings: string[]): ShapeDesc | null {
    const frame = frameOf(child(collision, 'origin'), `link ${link} collision origin`);
    const geometry = child(collision, 'geometry');
    const g = geometry?.children[0];
    if (!g) throw new RobotError(`link ${link} has a collision with no geometry`);
    const blank = { half: [0, 0, 0] as V3, radius: 0, halfLength: 0 };
    switch (g.name) {
        case 'box': {
            const size = numbers(g.attrs.size, 3, `link ${link} box size`);
            if (size.some((s) => !(s > 0))) throw new RobotError(`link ${link} box size must be positive`);
            return { ...blank, kind: 'box', half: [size[0] / 2, size[1] / 2, size[2] / 2], frame };
        }
        case 'sphere':
            return { ...blank, kind: 'sphere', radius: number1(g.attrs.radius, `link ${link} sphere radius`, true), frame };
        case 'cylinder':
        case 'capsule': {
            const radius = number1(g.attrs.radius, `link ${link} ${g.name} radius`, true);
            const length = number1(g.attrs.length, `link ${link} ${g.name} length`, true);
            // A cylinder has no engine shape: it becomes the capsule of the same radius and overall length, whose ends are rounded. URDF's axis is Z, a capsule's is Y.
            if (g.name === 'cylinder') warnings.push(`link ${link}: the cylinder is a capsule with rounded ends`);
            const halfLength = Math.max(length / 2 - radius, 0);
            return { ...blank, kind: 'capsule', radius, halfLength, frame: compose(frame, { p: [0, 0, 0], q: axisAngle([1, 0, 0], Math.PI / 2) }) };
        }
        default:
            warnings.push(`link ${link}: a ${g.name} collision is not supported and was dropped`);
            return null;
    }
}

export const MAX_LINKS = 512;

export function parseUrdf(text: string): RobotDesc {
    const robot = parseXml(text);
    if (robot.name !== 'robot') throw new RobotError(`the root element is <${robot.name}>, not <robot>`);
    const warnings: string[] = [];
    const links: LinkDesc[] = [];
    const seen = new Set<string>();
    for (const l of robot.children.filter((c) => c.name === 'link')) {
        const name = l.attrs.name;
        if (!name) throw new RobotError('a link has no name');
        if (seen.has(name)) throw new RobotError(`link ${name} is defined twice`);
        seen.add(name);
        if (links.length >= MAX_LINKS) throw new RobotError(`more than ${MAX_LINKS} links`);
        const massEl = child(child(l, 'inertial') ?? { name: '', attrs: {}, children: [], text: '' }, 'mass');
        const mass = massEl ? number1(massEl.attrs.value, `link ${name} mass`, true) : null;
        const collisions = l.children.filter((c) => c.name === 'collision');
        const shapes = collisions.map((c) => shapeOf(name, c, warnings)).filter((s): s is ShapeDesc => s !== null);
        if (shapes.length > 1) warnings.push(`link ${name}: only the first of ${shapes.length} collision shapes is used (a body has one shape)`);
        links.push({ name, mass, shape: shapes[0] ?? null });
    }
    if (links.length === 0) throw new RobotError('the robot has no links');
    const joints: RobotJointDesc[] = [];
    const jointNames = new Set<string>();
    for (const j of robot.children.filter((c) => c.name === 'joint')) {
        const name = j.attrs.name;
        if (!name) throw new RobotError('a joint has no name');
        if (jointNames.has(name)) throw new RobotError(`joint ${name} is defined twice`);
        jointNames.add(name);
        const type = j.attrs.type;
        if (type !== 'revolute' && type !== 'continuous' && type !== 'prismatic' && type !== 'fixed') throw new RobotError(`joint ${name} has type "${type}", which is not supported (revolute, continuous, prismatic and fixed are)`);
        const parent = child(j, 'parent')?.attrs.link;
        const kid = child(j, 'child')?.attrs.link;
        if (!parent || !seen.has(parent)) throw new RobotError(`joint ${name} has an unknown parent link "${parent}"`);
        if (!kid || !seen.has(kid)) throw new RobotError(`joint ${name} has an unknown child link "${kid}"`);
        const axisText = child(j, 'axis')?.attrs.xyz;
        const axis = type === 'fixed' ? [1, 0, 0] : numbers(axisText, 3, `joint ${name} axis`, [1, 0, 0]);
        const len = Math.hypot(axis[0], axis[1], axis[2]);
        if (!(len > 1e-6)) throw new RobotError(`joint ${name} has a zero axis`);
        let limit: RobotJointDesc['limit'] = null;
        const lim = child(j, 'limit');
        if (type === 'revolute' || type === 'prismatic') {
            if (!lim) throw new RobotError(`joint ${name} is ${type} and needs a <limit>`);
            const lower = number1(lim.attrs.lower ?? '0', `joint ${name} lower`);
            const upper = number1(lim.attrs.upper ?? '0', `joint ${name} upper`);
            if (lower > upper) throw new RobotError(`joint ${name} has lower ${lower} above upper ${upper}`);
            limit = { lower, upper };
        }
        joints.push({ name, type, parent, child: kid, origin: frameOf(child(j, 'origin'), `joint ${name} origin`), axis: [axis[0] / len, axis[1] / len, axis[2] / len], limit });
    }
    const parentOf = new Map<string, RobotJointDesc>();
    for (const j of joints) {
        if (parentOf.has(j.child)) throw new RobotError(`link ${j.child} has two parent joints (${parentOf.get(j.child)?.name} and ${j.name}); a loop is not a tree`);
        parentOf.set(j.child, j);
    }
    const roots = links.filter((l) => !parentOf.has(l.name));
    if (roots.length !== 1) throw new RobotError(roots.length === 0 ? 'every link has a parent: the joints form a loop' : `the links ${roots.map((r) => r.name).join(', ')} have no parent; a robot has one root`);
    const order: RobotJointDesc[] = [];
    const placed = new Set([roots[0].name]);
    for (let progress = true; progress && order.length < joints.length;) {
        progress = false;
        for (const j of joints) {
            if (!placed.has(j.child) && placed.has(j.parent)) { order.push(j); placed.add(j.child); progress = true; }
        }
    }
    if (order.length !== joints.length) throw new RobotError('some links are not connected to the root');
    return { name: robot.attrs.name ?? 'robot', root: roots[0].name, links, joints, order, warnings };
}

// ---- forward kinematics ----
/** The frame of every link in the robot's own frame (the root's frame is the identity) for the given joint values: radians for revolute and continuous joints, metres for prismatic. A joint not named is at 0. */
export function forwardKinematics(desc: RobotDesc, values: Readonly<Record<string, number>> = {}): Map<string, Frame> {
    const frames = new Map<string, Frame>([[desc.root, IDENTITY]]);
    for (const j of desc.order) {
        const v = Object.hasOwn(values, j.name) ? values[j.name] : 0;
        if (!Number.isFinite(v)) throw new RobotError(`value of joint ${j.name} is not finite`);
        const motion: Frame = j.type === 'prismatic' ? { p: [j.axis[0] * v, j.axis[1] * v, j.axis[2] * v], q: [0, 0, 0, 1] } : j.type === 'fixed' ? IDENTITY : { p: [0, 0, 0], q: axisAngle(j.axis, v) };
        const parent = frames.get(j.parent) as Frame;
        frames.set(j.child, compose(compose(parent, j.origin), motion));
    }
    return frames;
}

// ---- spawning ----
export interface SpawnRobotOptions {
    /** Where the root link's frame is placed. */
    position?: V3;
    rotation?: Quat;
    /** The root link is static (a bolted arm) when true; free (a walking robot) when false. Default false. */
    fixedBase?: boolean;
    /** Mass in kilograms for a link that states none. Default 1. */
    defaultMass?: number;
    friction?: number;
    /** Which engine axis a description's +Z points along: 'y' (default, the engine's gravity is along -Y) turns the robot upright; 'z' leaves its frames as written. */
    upAxis?: 'y' | 'z';
}

export interface SpawnedRobot {
    desc: RobotDesc;
    links: Map<string, EntityHandle>;
    joints: Map<string, JointHandle>;
    /** Static bodies made to hold a joint whose parent has no body. */
    anchors: EntityHandle[];
    /** Link frames at spawn, in the world. */
    frames: Map<string, Frame>;
    warnings: string[];
}

const MIN_HALF = 1e-3;
const MAX_HALF = 1e4;
const MIN_MASS = 1e-6;
const MAX_MASS = 1e30;
const ANCHOR_RADIUS = 0.002;

/**
 * Puts the robot into `world` at zero joint values: one body per link that has a collision shape, one joint per description joint.
 * A link without a shape is a frame only: it gets no body, its mass is dropped, and a joint into it cannot move anything (each is a warning).
 * A body whose ancestors have no body is a base: static when `fixedBase`, free otherwise. Bodies of linked links do not collide with each other; other pairs collide as usual.
 * Sizes and masses the engine cannot take are clamped to its bounds with a warning. If a command cannot be queued part way, the bodies made so far are destroyed and the error is rethrown.
 */
export function spawnRobot(world: World, desc: RobotDesc, opts: SpawnRobotOptions = {}): SpawnedRobot {
    const warnings = [...desc.warnings];
    const upright: Quat = opts.upAxis === 'z' ? [0, 0, 0, 1] : axisAngle([1, 0, 0], -Math.PI / 2);
    const base: Frame = compose({ p: opts.position ?? [0, 0, 0], q: opts.rotation ?? [0, 0, 0, 1] }, { p: [0, 0, 0], q: upright });
    const frames = new Map<string, Frame>();
    for (const [name, f] of forwardKinematics(desc)) frames.set(name, compose(base, f));
    const links = new Map<string, EntityHandle>();
    const joints = new Map<string, JointHandle>();
    const anchors: EntityHandle[] = [];
    const defaultMass = opts.defaultMass ?? 1;
    const parentJoint = new Map(desc.joints.map((j) => [j.child, j]));
    const carrier = (name: string): string | null => {
        for (let n: string | undefined = name; n !== undefined; n = parentJoint.get(n)?.parent) if (links.has(n)) return n;
        return null;
    };
    const fit = (value: number, lo: number, hi: number, what: string): number => {
        const v = Math.min(hi, Math.max(lo, value));
        if (v !== value) warnings.push(`${what} ${value} was clamped to ${v}, the engine's bound`);
        return v;
    };
    try {
        for (const l of desc.links) {
            if (!l.shape) {
                if (l.mass !== null) warnings.push(`link ${l.name}: it has a mass but no collision shape, so the mass is dropped`);
                continue;
            }
            const s = l.shape;
            const f = compose(frames.get(l.name) as Frame, s.frame);
            const isBase = !parentJoint.has(l.name) || carrier(parentJoint.get(l.name)!.parent) === null;
            const pj = parentJoint.get(l.name);
            // A body hinged or slid on a frame is not the bolted base: it moves on an anchor made for it below.
            const isStatic = l.static === true || (opts.fixedBase === true && isBase && (!pj || pj.type === 'fixed'));
            let mass = l.mass;
            if (mass === null && !isStatic) { warnings.push(`link ${l.name}: no mass given, using ${defaultMass} kg`); mass = defaultMass; }
            const what = `link ${l.name}`;
            const half = (v: number, name: string) => fit(v, MIN_HALF, MAX_HALF, `${what} ${name}`);
            const shape = s.kind === 'box' ? { shape: SHAPE_BOX, hx: half(s.half[0], 'half x'), hy: half(s.half[1], 'half y'), hz: half(s.half[2], 'half z') }
                : s.kind === 'sphere' ? { shape: SHAPE_SPHERE, hx: half(s.radius, 'radius'), hy: half(s.radius, 'radius'), hz: half(s.radius, 'radius') }
                : { shape: SHAPE_CAPSULE, hx: half(s.radius, 'radius'), hy: half(s.halfLength, 'half length'), hz: half(s.radius, 'radius') };
            const invMass = isStatic ? 0 : 1 / fit(mass as number, MIN_MASS, MAX_MASS, `${what} mass`);
            links.set(l.name, world.spawn({ x: f.p[0], y: f.p[1], z: f.p[2], qx: f.q[0], qy: f.q[1], qz: f.q[2], qw: f.q[3], ...shape, invMass, friction: opts.friction }));
        }
        if (links.size === 0) throw new RobotError('the robot has no link with a collision shape, so there is nothing to spawn');
        for (const j of desc.order) {
            if (!links.has(j.child)) {
                if (j.type !== 'fixed') warnings.push(`joint ${j.name}: its child link ${j.child} has no body, so the motion of this ${j.type} joint is lost`);
                continue;
            }
            const a = carrier(j.parent);
            const jf = compose(frames.get(j.parent) as Frame, j.origin);
            let carrierHandle: EntityHandle;
            if (a === null) {
                if (j.type === 'fixed') continue;
                // Nothing above has a body to hinge on: a tiny static sphere at the joint stands in for the world.
                carrierHandle = world.spawn({ x: jf.p[0], y: jf.p[1], z: jf.p[2], shape: SHAPE_SPHERE, hx: ANCHOR_RADIUS, hy: ANCHOR_RADIUS, hz: ANCHOR_RADIUS, invMass: 0 });
                anchors.push(carrierHandle);
                warnings.push(`joint ${j.name}: nothing above it has a body, so a ${ANCHOR_RADIUS * 2} m static anchor was made at the joint`);
            } else {
                carrierHandle = links.get(a) as EntityHandle;
            }
            const kind = j.type === 'fixed' ? JOINT_FIXED : j.type === 'prismatic' ? JOINT_SLIDER : JOINT_HINGE;
            let limits: [number, number] | undefined;
            if (j.limit) {
                const bound = kind === JOINT_HINGE ? MAX_ANGLE_LIMIT : MAX_LINEAR_LIMIT;
                // The engine measures a limit from the pose at creation, so the zero pose must be inside it.
                const lo = Math.min(0, Math.max(j.limit.lower, -bound));
                const hi = Math.max(0, Math.min(j.limit.upper, bound));
                if (lo !== j.limit.lower || hi !== j.limit.upper) warnings.push(`joint ${j.name}: limits [${j.limit.lower}, ${j.limit.upper}] were clamped to [${lo}, ${hi}] (zero must lie inside, and the engine's bound is ${bound})`);
                limits = [lo, hi];
            }
            joints.set(j.name, world.createJoint({ kind, a: carrierHandle, b: links.get(j.child) as EntityHandle, anchor: jf.p, axis: kind === JOINT_FIXED ? undefined : qrot(jf.q, j.axis), limits }));
        }
    } catch (err) {
        for (const h of [...links.values(), ...anchors]) {
            try { world.destroy(h); } catch { /* the ring is full: nothing more can be undone from here */ }
        }
        throw err;
    }
    return { desc, links, joints, anchors, frames, warnings };
}
