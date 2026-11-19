import { parseXml, type XmlElement } from './xml.js';
import { RobotError, compose, axisAngle, qmul, MAX_LINKS, type Frame, type LinkDesc, type Quat, type RobotDesc, type RobotJointDesc, type ShapeDesc, type V3 } from './robot.js';

const ZERO: V3 = [0, 0, 0];
const NUMBER = /^[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?$/;

function nums(text: string | undefined, min: number, max: number, what: string, fallback?: number[]): number[] {
    if (text === undefined) {
        if (fallback) return fallback;
        throw new RobotError(`${what} is missing`);
    }
    const parts = text.trim().split(/\s+/);
    const values = parts.map((s) => (NUMBER.test(s) ? Number(s) : NaN));
    if (parts.length < min || parts.length > max || values.some((v) => !Number.isFinite(v))) throw new RobotError(`${what} needs ${min === max ? min : `${min} to ${max}`} finite numbers, got "${text}"`);
    return values;
}

interface Ctx {
    degrees: boolean;
    autolimits: boolean;
    warnings: string[];
    links: LinkDesc[];
    joints: RobotJointDesc[];
    names: Set<string>;
    /** Every name the document states, so a made-up name never takes one of them. */
    reserved: Set<string>;
    jointNames: Set<string>;
}

function unique(base: string, used: Set<string>, reserved: Set<string>): string {
    let name = base;
    for (let n = 2; used.has(name) || reserved.has(name); n++) name = `${base}_${n}`;
    used.add(name);
    return name;
}

function explicitNames(el: XmlElement, out: Set<string>): void {
    for (const k of el.children) {
        if ((k.name === 'body' || k.name === 'joint') && k.attrs.name) out.add(k.attrs.name);
        explicitNames(k, out);
    }
}

const angle = (c: Ctx, v: number) => (c.degrees ? (v * Math.PI) / 180 : v);

/** MJCF orientation attributes of an element: `quat` (w x y z), `axisangle`, or `euler` in the default intrinsic xyz sequence. */
function orientation(c: Ctx, el: XmlElement, what: string): Quat {
    const set = ['quat', 'axisangle', 'euler', 'xyaxes', 'zaxis'].filter((k) => el.attrs[k] !== undefined);
    if (set.length > 1) throw new RobotError(`${what} gives its orientation twice (${set.join(', ')})`);
    if (set.length === 0) return [0, 0, 0, 1];
    switch (set[0]) {
        case 'quat': {
            const [w, x, y, z] = nums(el.attrs.quat, 4, 4, `${what} quat`);
            const l = Math.hypot(w, x, y, z);
            if (!(l > 1e-9)) throw new RobotError(`${what} quat is zero`);
            return [x / l, y / l, z / l, w / l];
        }
        case 'axisangle': {
            const [x, y, z, a] = nums(el.attrs.axisangle, 4, 4, `${what} axisangle`);
            if (!(Math.hypot(x, y, z) > 1e-9)) throw new RobotError(`${what} axisangle has a zero axis`);
            return axisAngle([x, y, z], angle(c, a));
        }
        case 'euler': {
            const [a, b, g] = nums(el.attrs.euler, 3, 3, `${what} euler`).map((v) => angle(c, v));
            return qmul(qmul(axisAngle([1, 0, 0], a), axisAngle([0, 1, 0], b)), axisAngle([0, 0, 1], g));
        }
        default:
            throw new RobotError(`${what} uses ${set[0]}, which is not supported (quat, axisangle and euler are)`);
    }
}

function volume(type: string, s: { half: V3; radius: number; halfLength: number }, cylinderHalf: number): number {
    if (type === 'cylinder') return Math.PI * s.radius ** 2 * 2 * cylinderHalf;
    if (type === 'box' || type === 'plane') return 8 * s.half[0] * s.half[1] * s.half[2];
    if (type === 'sphere') return (4 / 3) * Math.PI * s.radius ** 3;
    return Math.PI * s.radius ** 2 * 2 * s.halfLength + (4 / 3) * Math.PI * s.radius ** 3;
}

/** The engine's capsule lies along Y; an MJCF capsule or cylinder lies along its geom frame's Z. */
const Z_TO_Y: Quat = axisAngle([1, 0, 0], Math.PI / 2);

interface Geom { shape: ShapeDesc; mass: number | null }

function geomOf(c: Ctx, el: XmlElement, owner: string, shift: V3): Geom | null {
    const what = `${owner} geom${el.attrs.name ? ` ${el.attrs.name}` : ''}`;
    const type = el.attrs.type ?? 'sphere';
    let frame: Frame = { p: [0, 0, 0], q: [0, 0, 0, 1] };
    const blank = { half: [0, 0, 0] as V3, radius: 0, halfLength: 0 };
    let shape: ShapeDesc;
    // A cylinder's mass is that of the cylinder, not of the rounded capsule that stands in for it.
    let cylinderHalf = 0;
    if (el.attrs.fromto !== undefined) {
        if (type !== 'capsule' && type !== 'cylinder') throw new RobotError(`${what}: fromto is only for capsule and cylinder`);
        const [x0, y0, z0, x1, y1, z1] = nums(el.attrs.fromto, 6, 6, `${what} fromto`);
        const d = [x1 - x0, y1 - y0, z1 - z0];
        const len = Math.hypot(d[0], d[1], d[2]);
        if (!(len > 1e-9)) throw new RobotError(`${what}: fromto has no length`);
        const [radius] = nums(el.attrs.size, 1, 3, `${what} size`);
        // Turn +Z onto the segment: about the axis Z x d, by the angle between them (a half turn about X when they are opposite).
        const cross = [-d[1], d[0], 0];
        const cl = Math.hypot(cross[0], cross[1]);
        const toSegment: Quat = cl < 1e-12 * len ? (d[2] > 0 ? [0, 0, 0, 1] : [1, 0, 0, 0]) : axisAngle(cross, Math.atan2(cl, d[2]));
        frame = { p: [(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2], q: toSegment };
        shape = { ...blank, kind: 'capsule', radius, halfLength: type === 'capsule' ? len / 2 : Math.max(len / 2 - radius, 0), frame };
        if (type === 'cylinder') { cylinderHalf = len / 2; c.warnings.push(`${what}: the cylinder is a capsule with rounded ends`); }
    } else {
        const pos = nums(el.attrs.pos, 3, 3, `${what} pos`, [0, 0, 0]) as V3;
        frame = { p: pos, q: orientation(c, el, what) };
        switch (type) {
            case 'sphere': shape = { ...blank, kind: 'sphere', radius: nums(el.attrs.size, 1, 3, `${what} size`)[0], frame }; break;
            case 'box': {
                const [hx, hy, hz] = nums(el.attrs.size, 3, 3, `${what} size`);
                shape = { ...blank, kind: 'box', half: [hx, hy, hz], frame };
                break;
            }
            case 'capsule':
            case 'cylinder': {
                const [radius, half] = nums(el.attrs.size, 2, 3, `${what} size`);
                if (type === 'cylinder') { cylinderHalf = half; c.warnings.push(`${what}: the cylinder is a capsule with rounded ends`); }
                shape = { ...blank, kind: 'capsule', radius, halfLength: type === 'capsule' ? half : Math.max(half - radius, 0), frame };
                break;
            }
            case 'plane': {
                // An infinite plane becomes a wide, thin slab whose top face is the plane.
                const [sx, sy] = nums(el.attrs.size, 2, 3, `${what} size`);
                const hx = sx > 0 ? sx : 50, hy = sy > 0 ? sy : 50;
                shape = { ...blank, kind: 'box', half: [hx, hy, 0.5], frame: compose(frame, { p: [0, 0, -0.5], q: [0, 0, 0, 1] }) };
                c.warnings.push(`${what}: MuJoCo's plane is infinite; here it is a ${2 * hx} by ${2 * hy} m slab (size 0 means ${50 * 2} m)`);
                break;
            }
            default:
                c.warnings.push(`${what}: a ${type} geom is not supported and was dropped`);
                return null;
        }
    }
    if (shape.kind === 'capsule') shape.frame = compose(frame, { p: [0, 0, 0], q: Z_TO_Y });
    for (const dim of [...shape.half, shape.radius, shape.halfLength]) if (!(dim >= 0)) throw new RobotError(`${what} has a negative size`);
    if (shape.kind === 'box' ? shape.half.some((h) => h <= 0) : shape.radius <= 0) throw new RobotError(`${what} needs positive sizes`);
    shape.frame = { p: [shape.frame.p[0] - shift[0], shape.frame.p[1] - shift[1], shape.frame.p[2] - shift[2]], q: shape.frame.q };
    let mass: number | null = null;
    if (el.attrs.mass !== undefined) {
        [mass] = nums(el.attrs.mass, 1, 1, `${what} mass`);
        if (!(mass > 0)) throw new RobotError(`${what} mass must be positive`);
    } else if (type !== 'plane') {
        const [density] = nums(el.attrs.density, 1, 1, `${what} density`, [1000]);
        mass = density * volume(type, shape, type === 'cylinder' ? cylinderHalf : 0);
    }
    return { shape, mass };
}

function addBody(c: Ctx, el: XmlElement, parent: string, parentFrameOffset: V3, top: boolean): void {
    if (c.links.length >= MAX_LINKS) throw new RobotError(`more than ${MAX_LINKS} bodies`);
    if (el.attrs.name !== undefined && c.names.has(el.attrs.name)) throw new RobotError(`body ${el.attrs.name} is defined twice`);
    const name = el.attrs.name ?? unique(`body${c.links.length}`, c.names, c.reserved);
    c.names.add(name);
    const pos = nums(el.attrs.pos, 3, 3, `body ${name} pos`, [0, 0, 0]);
    const q = orientation(c, el, `body ${name}`);
    const joints = el.children.filter((k) => k.name === 'joint' || k.name === 'freejoint');
    if (joints.length > 1) throw new RobotError(`body ${name} has ${joints.length} joints; one joint per body is supported (a body has one shape, so a second degree of freedom would need a body with no mass in between)`);
    const j = joints[0];
    // The link frame sits at the joint: its origin is the body's pos plus the joint's own pos, and everything in the body is shifted back by that pos.
    let shift: V3 = ZERO;
    let type: RobotJointDesc['type'] = 'fixed';
    let axis: V3 = [1, 0, 0];
    let limit: RobotJointDesc['limit'] = null;
    const free = j?.name === 'freejoint' || j?.attrs.type === 'free';
    if (free && !top) throw new RobotError(`body ${name} has a free joint, which is only supported on a top-level body`);
    if (j && !free) {
        const kind = j.attrs.type ?? 'hinge';
        if (kind !== 'hinge' && kind !== 'slide') throw new RobotError(`joint ${j.attrs.name ?? name} has type "${kind}", which is not supported (hinge and slide are)`);
        shift = nums(j.attrs.pos, 3, 3, `joint ${j.attrs.name ?? name} pos`, [0, 0, 0]) as V3;
        axis = nums(j.attrs.axis, 3, 3, `joint ${j.attrs.name ?? name} axis`, [0, 0, 1]) as V3;
        if (!(Math.hypot(...axis) > 1e-9)) throw new RobotError(`joint ${j.attrs.name ?? name} has a zero axis`);
        const limited = j.attrs.limited ?? (c.autolimits ? 'auto' : 'false');
        if (limited !== 'true' && limited !== 'false' && limited !== 'auto') throw new RobotError(`joint ${j.attrs.name ?? name} has limited "${limited}"`);
        if (limited === 'true' && j.attrs.range === undefined) throw new RobotError(`joint ${j.attrs.name ?? name} is limited and has no range`);
        const range = j.attrs.range !== undefined && limited !== 'false' ? nums(j.attrs.range, 2, 2, `joint ${j.attrs.name ?? name} range`) : null;
        if (range) {
            const [lo, hi] = kind === 'hinge' ? range.map((v) => angle(c, v)) : range;
            if (lo > hi) throw new RobotError(`joint ${j.attrs.name ?? name} has a range above its upper bound`);
            limit = { lower: lo, upper: hi };
        }
        type = kind === 'slide' ? 'prismatic' : limit ? 'revolute' : 'continuous';
    }
    if (j?.attrs.name !== undefined && c.jointNames.has(j.attrs.name)) throw new RobotError(`joint ${j.attrs.name} is defined twice`);
    const jointName = j?.attrs.name ?? unique(`${name}_joint`, c.jointNames, c.reserved);
    c.jointNames.add(jointName);
    const origin = compose({ p: [pos[0] - parentFrameOffset[0], pos[1] - parentFrameOffset[1], pos[2] - parentFrameOffset[2]], q }, { p: shift, q: [0, 0, 0, 1] });
    const geoms = el.children.filter((k) => k.name === 'geom').map((g) => geomOf(c, g, `body ${name}`, shift)).filter((g): g is Geom => g !== null);
    if (geoms.length > 1) c.warnings.push(`body ${name}: only the first of ${geoms.length} geoms is the shape (a body has one); their masses are summed`);
    const inertial = el.children.find((k) => k.name === 'inertial');
    if (inertial && (inertial.attrs.diaginertia ?? inertial.attrs.fullinertia ?? inertial.attrs.pos) !== undefined) c.warnings.push(`body ${name}: the inertial's position and inertia are ignored; the engine derives inertia from the shape`);
    const geomMass = geoms.reduce<number | null>((sum, g) => (g.mass === null ? sum : (sum ?? 0) + g.mass), null);
    const mass = inertial ? nums(inertial.attrs.mass, 1, 1, `body ${name} inertial mass`)[0] : geomMass;
    if (mass !== null && !(mass > 0)) throw new RobotError(`body ${name} mass must be positive`);
    // MJCF welds a top-level body with no joint to the world; a free joint leaves it free. A joint-less child is held by its fixed joint to the body above.
    c.links.push({ name, mass, shape: geoms[0]?.shape ?? null, ...(top && !j ? { static: true } : {}) });
    c.joints.push({ name: jointName, type, parent, child: name, origin, axis: axis.map((v) => v / Math.hypot(...axis)) as V3, limit });
    for (const child of el.children.filter((k) => k.name === 'body')) addBody(c, child, name, shift, false);
    for (const other of el.children) {
        if (!['body', 'geom', 'joint', 'freejoint', 'inertial', 'site', 'camera', 'light'].includes(other.name)) c.warnings.push(`body ${name}: <${other.name}> is ignored`);
    }
}

export function parseMjcf(text: string): RobotDesc {
    const root = parseXml(text);
    if (root.name !== 'mujoco') throw new RobotError(`the root element is <${root.name}>, not <mujoco>`);
    const compiler = root.children.find((k) => k.name === 'compiler');
    const unit = compiler?.attrs.angle ?? 'degree';
    if (unit !== 'degree' && unit !== 'radian') throw new RobotError(`compiler angle "${unit}" is not degree or radian`);
    if ((compiler?.attrs.eulerseq ?? 'xyz') !== 'xyz') throw new RobotError(`compiler eulerseq "${compiler?.attrs.eulerseq}" is not supported (xyz is)`);
    const autolimits = compiler?.attrs.autolimits ?? 'true';
    if (autolimits !== 'true' && autolimits !== 'false') throw new RobotError(`compiler autolimits "${autolimits}" is not true or false`);
    const c: Ctx = { degrees: unit === 'degree', autolimits: autolimits === 'true', warnings: [], links: [], joints: [], names: new Set(['world']), reserved: new Set(), jointNames: new Set() };
    for (const el of root.children) {
        if (!['compiler', 'option', 'size', 'visual', 'statistic', 'worldbody', 'asset', 'actuator', 'sensor', 'default', 'contact', 'equality', 'tendon', 'keyframe', 'extension'].includes(el.name)) throw new RobotError(`<${el.name}> is not a MJCF element this reader knows`);
        if (['default', 'asset', 'actuator', 'sensor', 'contact', 'equality', 'tendon', 'keyframe'].includes(el.name)) c.warnings.push(`<${el.name}> is ignored`);
    }
    const usesClasses = (e: XmlElement): boolean => e.attrs.class !== undefined || e.attrs.childclass !== undefined || e.children.some(usesClasses);
    if (usesClasses(root)) c.warnings.push('class and childclass are ignored, so values a <default> would set are not applied');
    const world = root.children.find((k) => k.name === 'worldbody');
    if (!world) throw new RobotError('there is no <worldbody>');
    explicitNames(world, c.reserved);
    c.links.push({ name: 'world', mass: null, shape: null });
    let scenery = 0;
    for (const el of world.children) {
        if (el.name === 'geom') {
            const g = geomOf(c, el, 'worldbody', ZERO);
            if (!g) continue;
            if (c.links.length >= MAX_LINKS) throw new RobotError(`more than ${MAX_LINKS} bodies and geoms`);
            const name = unique(`world_geom${scenery++}`, c.names, c.reserved);
            const jointName = unique(`${name}_joint`, c.jointNames, c.reserved);
            c.links.push({ name, mass: null, shape: g.shape, static: true });
            c.joints.push({ name: jointName, type: 'fixed', parent: 'world', child: name, origin: { p: [0, 0, 0], q: [0, 0, 0, 1] }, axis: [1, 0, 0], limit: null });
        } else if (el.name === 'body') {
            addBody(c, el, 'world', ZERO, true);
        } else if (!['light', 'camera', 'site'].includes(el.name)) {
            c.warnings.push(`worldbody: <${el.name}> is ignored`);
        }
    }
    if (c.links.length === 1) throw new RobotError('the worldbody has no bodies or geoms');
    const order: RobotJointDesc[] = [...c.joints];
    return { name: root.attrs.model ?? 'mujoco', root: 'world', links: c.links, joints: c.joints, order, warnings: c.warnings };
}
