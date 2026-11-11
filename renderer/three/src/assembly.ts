import { Vector3, type Object3D } from 'three';
import { Assembly, type EntityHandle, type MateSpec, type SpawnOptions, type World } from '@dynova/sdk';
import { collidersFromScene, spawnColliders, type ColliderDesc, type ColliderWorld, type SkippedNode } from './colliders.js';

/**
 * What a glTF node's extras (`node.userData.dynova`) may say. Every mesh is a part; a part is dynamic (mass 1) unless it says
 * `static: true` or `mass: 0`. A part may carry one mate or a list of them, each joining it to a partner part, by default the nearest
 * mesh above it in the node hierarchy. Points and axes are in the part's own node frame, so they follow the node when the model is
 * moved or scaled in the editor.
 */
export interface PartExtras {
    static?: boolean;
    mass?: number;
    friction?: number;
    restitution?: number;
    mate?: MateExtras | MateExtras[];
}

export interface MateExtras {
    type: 'fixed' | 'ball' | 'revolute' | 'slider' | 'distance';
    /** Name of the partner part; default the nearest part above this node. */
    with?: string;
    /** Joint point in this node's frame; default the node origin. Not used by `distance`. */
    at?: readonly [number, number, number];
    /** Hinge or slider axis in this node's frame. */
    axis?: readonly [number, number, number];
    /** `[lo, hi]` measured from the authored pose: radians for revolute, metres for slider, stretch for distance. */
    range?: readonly [number, number];
    /** `distance` only: the point on the partner in the partner's frame, and the point on this part in this node's frame. */
    from?: readonly [number, number, number];
    to?: readonly [number, number, number];
}

export interface AssemblyHost extends ColliderWorld {
    spawn(opts: SpawnOptions): EntityHandle;
}

export interface ImportOptions {
    /** Floating origin of the assembly: a point, or `'centre'` (default) for the middle of the bounding box of the parts' shape centres. */
    origin?: readonly [number, number, number] | 'centre';
}

export interface ImportedAssembly {
    assembly: Assembly;
    /** Part names in scene order. */
    parts: string[];
    mates: string[];
    /** Meshes that could not become parts, with the reason. */
    skipped: SkippedNode[];
    /** Parts the world refused (for example an invalid size) and mates that were malformed or named a missing partner. */
    errors: { name: string; error: string }[];
}

type V3 = [number, number, number];

function point(v: unknown, what: string): V3 {
    if (!Array.isArray(v) || v.length !== 3 || !v.every((n) => typeof n === 'number' && Number.isFinite(n))) throw new Error(`${what} must be three finite numbers`);
    return [v[0], v[1], v[2]];
}

function extrasOf(node: Object3D): PartExtras {
    const raw = (node.userData as { dynova?: unknown } | undefined)?.dynova;
    if (raw === undefined) return {};
    if (typeof raw !== 'object' || raw === null || Array.isArray(raw)) throw new Error('userData.dynova must be an object');
    return raw as PartExtras;
}

/** The partner a mate falls back to: the first mesh above the node. If that mesh is not a part, the mate says so instead of reaching past it. */
function nearestPartAbove(node: Object3D, parts: ReadonlyMap<Object3D, string>, failed: ReadonlyMap<Object3D, string>): string {
    for (let p = node.parent; p; p = p.parent) {
        const name = parts.get(p);
        if (name !== undefined) return name;
        const bad = failed.get(p);
        if (bad !== undefined) throw new Error(`the part above it ('${bad}') could not be imported`);
        if ((p as { isMesh?: boolean }).isMesh) throw new Error('the mesh above it is not a part');
    }
    throw new Error('no partner: the mate has no `with` and no part above it in the hierarchy');
}

function ranged(v: unknown, what: string, lo: number, hi: number): number | undefined {
    if (v === undefined) return undefined;
    if (typeof v !== 'number' || !Number.isFinite(v) || v < lo || v > hi) throw new Error(`${what} must be a number from ${lo} to ${hi}`);
    return v;
}

/** Part properties the world would silently drop (it counts them malformed), checked here so they become rows. */
function checkedExtras(node: Object3D): PartExtras {
    const e = extrasOf(node);
    if (e.static !== undefined && typeof e.static !== 'boolean') throw new Error('static must be true or false');
    if (e.mass !== undefined && e.mass !== 0) ranged(e.mass, 'mass', 1e-6, Number.MAX_VALUE);
    ranged(e.friction, 'friction', 0, 2);
    ranged(e.restitution, 'restitution', 0, 1);
    return e;
}

/**
 * Builds an `Assembly` from a loaded scene: each mesh becomes a part through the same shape fitting as `collidersFromScene`, and the
 * mates described in the nodes' extras are made from the pose the scene was authored in. Failures are rows in the result, never
 * exceptions, and one bad part or mate does not stop the rest. The assembly's floating origin defaults to the centre of the parts, so
 * a model modelled far from the origin still simulates near zero.
 */
export function importAssembly(root: Object3D, world: AssemblyHost, options: ImportOptions = {}): ImportedAssembly {
    const found = collidersFromScene(root);
    const skipped = found.skipped;
    const errors: { name: string; error: string }[] = [];
    // A node scaled to zero on an axis decomposes to a NaN pose; it cannot be placed, and one such node must not take the whole import down.
    const colliders = found.colliders.filter((c) => {
        if ([...c.position, ...c.quaternion].every(Number.isFinite)) return true;
        errors.push({ name: c.name, error: 'the node has no finite pose (a zero scale?)' });
        return false;
    });

    let origin: V3;
    if (options.origin === undefined || options.origin === 'centre') {
        const lo: V3 = [Infinity, Infinity, Infinity], hi: V3 = [-Infinity, -Infinity, -Infinity];
        for (const c of colliders) c.position.forEach((v, k) => { lo[k] = Math.min(lo[k], v); hi[k] = Math.max(hi[k], v); });
        origin = colliders.length ? [0, 1, 2].map((k) => (lo[k] + hi[k]) / 2) as V3 : [0, 0, 0];
    } else {
        origin = point(options.origin, 'origin');
    }
    const assembly = new Assembly(world as unknown as World, origin);

    // Each part is spawned through the assembly so its position is localised and it is registered under its node name.
    let current = '';
    const adapter: ColliderWorld = {
        registerHull: (points) => world.registerHull(points),
        spawn: (opts) => assembly.addPart(current, opts),
    };
    const hullCache = new Map<string, number>();
    const partOf = new Map<Object3D, string>();
    const failedOf = new Map<Object3D, string>();
    const extrasOfPart = new Map<string, PartExtras>();
    const originals = new Map<string, string[]>();
    const seen = new Set<string>();
    const spawned: ColliderDesc[] = [];
    for (const c of colliders) {
        if (seen.has(c.name)) {
            errors.push({ name: c.name, error: 'two parts share this name; part names must be unique' });
            failedOf.set(c.node, c.name);
            continue;
        }
        seen.add(c.name);
        // GLTFLoader renames nodes (spaces and brackets go, duplicates get _1) and keeps the file's name here.
        const original = (c.node.userData as { name?: unknown }).name;
        if (typeof original === 'string' && original !== c.name) originals.set(original, [...(originals.get(original) ?? []), c.name]);
        let extras: PartExtras;
        try {
            extras = checkedExtras(c.node);
            const local = c.position.map((v, k) => v - origin[k]);
            if (!local.every((v) => Math.abs(v) <= 1e6)) throw new Error('the part is more than 1,000,000 units from the assembly origin; move the origin closer');
        } catch (err) {
            errors.push({ name: c.name, error: err instanceof Error ? err.message : String(err) });
            failedOf.set(c.node, c.name);
            continue;
        }
        extrasOfPart.set(c.name, extras);
        const isStatic = extras.static === true || extras.mass === 0;
        current = c.name;
        const report = spawnColliders(adapter, [c], {
            hullCache,
            body: () => ({ invMass: isStatic ? 0 : 1 / (extras.mass ?? 1), friction: extras.friction, restitution: extras.restitution }),
        });
        if (report.failed.length) {
            errors.push(...report.failed);
            failedOf.set(c.node, c.name);
        } else {
            partOf.set(c.node, c.name);
            spawned.push(c);
        }
    }

    const mates: string[] = [];
    const toWorld = (node: Object3D, local: V3): V3 => {
        const v = node.localToWorld(new Vector3(local[0], local[1], local[2]));
        return [v.x, v.y, v.z];
    };
    // The node's whole world matrix, so a mirrored or stretched node turns the axis the way it turns the part; transformDirection normalises.
    const directionToWorld = (node: Object3D, local: V3): V3 => {
        const v = new Vector3(local[0], local[1], local[2]).transformDirection(node.matrixWorld);
        return [v.x, v.y, v.z];
    };
    const resolve = (name: unknown): string => {
        if (typeof name !== 'string') throw new Error('`with` must be a part name');
        if (nodeByPart.has(name)) return name;
        const alias = originals.get(name);
        if (alias?.length === 1) return alias[0];
        if (alias && alias.length > 1) throw new Error(`'${name}' names ${alias.length} parts (${alias.join(', ')}); use one of those names`);
        throw new Error(`no part named '${name}'`);
    };
    const nodeByPart = new Map<string, Object3D>(spawned.map((c) => [c.name, c.node]));

    for (const c of spawned) {
        const raw = extrasOfPart.get(c.name)?.mate;
        if (raw === undefined) continue;
        const list = Array.isArray(raw) ? raw : [raw];
        list.forEach((m, i) => {
            const name = Array.isArray(raw) ? `${c.name}#${i}` : c.name;
            try {
                if (typeof m !== 'object' || m === null) throw new Error('a mate must be an object');
                const partner = m.with === undefined ? nearestPartAbove(c.node, partOf, failedOf) : resolve(m.with);
                const partnerNode = nodeByPart.get(partner) as Object3D;
                if (partner === c.name) throw new Error('a part cannot be mated to itself');
                const at = toWorld(c.node, m.at === undefined ? [0, 0, 0] : point(m.at, 'at'));
                const axis = () => directionToWorld(c.node, point(m.axis, 'axis'));
                if (m.range !== undefined && (!Array.isArray(m.range) || m.range.length !== 2 || !m.range.every((n) => typeof n === 'number' && Number.isFinite(n)))) throw new Error('range must be [lo, hi]');
                const range = m.range;
                let spec: MateSpec;
                switch (m.type) {
                    case 'fixed': spec = { type: 'fixed', a: partner, b: c.name, at }; break;
                    case 'ball': spec = { type: 'ball', a: partner, b: c.name, at }; break;
                    case 'revolute': spec = { type: 'revolute', a: partner, b: c.name, at, axis: axis(), range }; break;
                    case 'slider': spec = { type: 'slider', a: partner, b: c.name, at, axis: axis(), range }; break;
                    case 'distance':
                        spec = { type: 'distance', a: partner, b: c.name, pointA: toWorld(partnerNode, point(m.from, 'from')), pointB: toWorld(c.node, point(m.to, 'to')), range };
                        break;
                    default: throw new Error(`unknown mate type '${String((m as { type: unknown }).type)}'`);
                }
                assembly.addMate(name, spec);
                mates.push(name);
            } catch (err) {
                errors.push({ name, error: err instanceof Error ? err.message : String(err) });
            }
        });
    }
    return { assembly, parts: spawned.map((c) => c.name), mates, skipped, errors };
}
