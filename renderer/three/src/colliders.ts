import { Matrix4, Quaternion, Vector3, type BufferAttribute, type InterleavedBufferAttribute, type Mesh, type Object3D } from 'three';
import { ConvexHull } from 'three/examples/jsm/math/ConvexHull.js';
import { MAX_HULL_COORD, MAX_HULL_VERTS, SHAPE_BOX, SHAPE_HULL, SHAPE_SPHERE, type EntityHandle, type SpawnOptions } from '@dynova/sdk';

export type Vec3 = readonly [number, number, number];
export type Quat4 = readonly [number, number, number, number];

/** `center` is the shape's origin relative to the mesh origin in scaled local space; a box's `rotation` is relative to the mesh frame. */
export type Shape =
    | { kind: 'box'; half: Vec3; center: Vec3; rotation: Quat4 }
    | { kind: 'sphere'; radius: number; center: Vec3 }
    | { kind: 'hull'; points: Float32Array; center: Vec3 };

export type SkipReason = 'no-geometry' | 'unsupported' | 'non-finite' | 'too-large' | 'degenerate';

export type FitResult = { ok: true; shape: Shape } | { ok: false; reason: SkipReason };

const MIN_HALF = 0.005;
const SPHERE_RADIAL_SPREAD = 0.05;
const SPHERE_FILL = 0.8;
const BOX_FILL = 0.92;
const QUANTUM = 1e5;
const IDENTITY: Quat4 = [0, 0, 0, 1];
const RUST_MIN_EXTENT = 2e-3;
const RUST_MIN_VOLUME = 4e-6;

type P3 = [number, number, number];

const lex = (a: P3, b: P3) => a[0] - b[0] || a[1] - b[1] || a[2] - b[2];

function uniqueSorted(positions: ArrayLike<number>): P3[] {
    const seen = new Map<string, P3>();
    for (let i = 0; i + 2 < positions.length; i += 3) {
        const key = `${Math.round(positions[i] * QUANTUM)},${Math.round(positions[i + 1] * QUANTUM)},${Math.round(positions[i + 2] * QUANTUM)}`;
        if (!seen.has(key)) seen.set(key, [positions[i], positions[i + 1], positions[i + 2]]);
    }
    return [...seen.values()].sort(lex);
}

function faceLoops(hull: ConvexHull): P3[][] {
    return hull.faces.map((face) => {
        const loop: P3[] = [];
        let edge = face.edge;
        do {
            const p = edge.head().point;
            loop.push([p.x, p.y, p.z]);
            edge = edge.next;
        } while (edge !== face.edge);
        return loop;
    });
}

function hullOf(points: P3[]): { vertices: P3[]; loops: P3[][] } | null {
    if (points.length < 4) return null;
    try {
        const hull = new ConvexHull().setFromPoints(points.map((p) => new Vector3(p[0], p[1], p[2])));
        if (hull.faces.length < 4) return null;
        const loops = faceLoops(hull);
        const seen = new Map<string, P3>();
        for (const loop of loops) {
            for (const p of loop) seen.set(`${p[0]},${p[1]},${p[2]}`, p);
        }
        const vertices = [...seen.values()].sort(lex);
        return vertices.every((p) => p.every(Number.isFinite)) ? { vertices, loops } : null;
    } catch {
        return null;
    }
}

function volumeCentroid(loops: P3[][]): P3 | null {
    let volume = 0;
    const moment: P3 = [0, 0, 0];
    for (const loop of loops) {
        const a = loop[0];
        for (let k = 1; k + 1 < loop.length; k++) {
            const b = loop[k], c = loop[k + 1];
            const v = (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6;
            volume += v;
            for (let i = 0; i < 3; i++) moment[i] += v * (a[i] + b[i] + c[i]) * 0.25;
        }
    }
    if (!(Math.abs(volume) > 1e-12)) return null;
    return [moment[0] / volume, moment[1] / volume, moment[2] / volume];
}

function loopsVolume(loops: P3[][]): number {
    let volume = 0;
    for (const loop of loops) {
        const a = loop[0];
        for (let k = 1; k + 1 < loop.length; k++) {
            const b = loop[k], c = loop[k + 1];
            volume += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6;
        }
    }
    return Math.abs(volume);
}

const sub3 = (a: P3, b: P3): P3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot3 = (a: P3, b: P3) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross3 = (a: P3, b: P3): P3 => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm3 = (a: P3) => Math.hypot(a[0], a[1], a[2]);

function quaternionFromColumns(x: P3, y: P3, z: P3): Quat4 {
    const [m00, m10, m20, m01, m11, m21, m02, m12, m22] = [x[0], x[1], x[2], y[0], y[1], y[2], z[0], z[1], z[2]];
    const trace = m00 + m11 + m22;
    let q: Quat4;
    if (trace > 0) {
        const s = Math.sqrt(trace + 1) * 2;
        q = [(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, s / 4];
    } else if (m00 > m11 && m00 > m22) {
        const s = Math.sqrt(1 + m00 - m11 - m22) * 2;
        q = [s / 4, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s];
    } else if (m11 > m22) {
        const s = Math.sqrt(1 + m11 - m00 - m22) * 2;
        q = [(m01 + m10) / s, s / 4, (m12 + m21) / s, (m02 - m20) / s];
    } else {
        const s = Math.sqrt(1 + m22 - m00 - m11) * 2;
        q = [(m02 + m20) / s, (m12 + m21) / s, s / 4, (m10 - m01) / s];
    }
    const l = Math.hypot(q[0], q[1], q[2], q[3]);
    return [q[0] / l, q[1] / l, q[2] / l, q[3] / l];
}

/** Box for points lying in one plane that is not aligned with a mesh axis (ramps, tilted floors); thickness is clamped. */
function orientedFlatBox(pts: P3[], tolerance: number): Shape | null {
    const p0 = pts[0];
    const p1 = pts.reduce((best, p) => (norm3(sub3(p, p0)) > norm3(sub3(best, p0)) ? p : best), p0);
    const axis = sub3(p1, p0);
    const len = norm3(axis);
    if (len === 0) return null;
    const dir: P3 = [axis[0] / len, axis[1] / len, axis[2] / len];
    const p2 = pts.reduce((best, p) => (norm3(cross3(sub3(p, p0), dir)) > norm3(cross3(sub3(best, p0), dir)) ? p : best), p0);
    const n3 = cross3(sub3(p2, p0), dir);
    const nl = norm3(n3);
    if (nl <= tolerance) return null;
    const n: P3 = [n3[0] / nl, n3[1] / nl, n3[2] / nl];
    if (pts.some((p) => Math.abs(dot3(n, sub3(p, p0))) > tolerance)) return null;
    const u = dir;
    const v = cross3(u, n);
    const range = (axisVec: P3) => pts.reduce<[number, number]>((r, p) => { const d = dot3(axisVec, sub3(p, p0)); return [Math.min(r[0], d), Math.max(r[1], d)]; }, [Infinity, -Infinity]);
    const [ul, uh] = range(u);
    const [vl, vh] = range(v);
    const cu = (ul + uh) / 2, cv = (vl + vh) / 2;
    const center: Vec3 = [p0[0] + u[0] * cu + v[0] * cv, p0[1] + u[1] * cu + v[1] * cv, p0[2] + u[2] * cu + v[2] * cv];
    const half: Vec3 = [Math.max((uh - ul) / 2, MIN_HALF), MIN_HALF, Math.max((vh - vl) / 2, MIN_HALF)];
    return { kind: 'box', half, center, rotation: quaternionFromColumns(u, n, v) };
}

/** Greedy farthest-point sampling; ties broken by coordinates so the result does not depend on input order. */
function sample(points: P3[], limit: number): P3[] {
    const mean = points.reduce<P3>((m, p) => [m[0] + p[0] / points.length, m[1] + p[1] / points.length, m[2] + p[2] / points.length], [0, 0, 0]);
    const dist2 = (a: P3, b: P3) => (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2;
    const farthest = (score: (p: P3) => number, exclude: Set<P3>) => {
        let best: P3 | null = null;
        let bestScore = -1;
        for (const p of points) {
            if (exclude.has(p)) continue;
            const s = score(p);
            if (s > bestScore || (s === bestScore && best && lex(p, best) < 0)) { best = p; bestScore = s; }
        }
        return best;
    };
    const chosen: P3[] = [];
    const taken = new Set<P3>();
    const first = farthest((p) => dist2(p, mean), taken);
    if (!first) return [];
    chosen.push(first);
    taken.add(first);
    while (chosen.length < limit) {
        const next = farthest((p) => Math.min(...chosen.map((c) => dist2(p, c))), taken);
        if (!next) break;
        chosen.push(next);
        taken.add(next);
    }
    return chosen.sort(lex);
}

/**
 * Fits a mesh given as scaled local-space positions and optional triangle indices. Box when the mesh fills most of its
 * bounding box, sphere when it is round, otherwise a convex hull with at most `MAX_HULL_VERTS` vertices.
 */
export function fitShape(positions: ArrayLike<number>, indices: ArrayLike<number> | null): FitResult {
    if (positions.length === 0) return { ok: false, reason: 'no-geometry' };
    if (positions.length % 3 !== 0) return { ok: false, reason: 'degenerate' };
    for (let i = 0; i < positions.length; i++) {
        if (!Number.isFinite(positions[i])) return { ok: false, reason: 'non-finite' };
        if (Math.abs(positions[i]) > MAX_HULL_COORD) return { ok: false, reason: 'too-large' };
    }
    if (indices && indices.length % 3 !== 0) return { ok: false, reason: 'unsupported' };
    const vertexCount = positions.length / 3;
    if (indices) for (let i = 0; i < indices.length; i++) if (!(indices[i] >= 0 && indices[i] < vertexCount)) return { ok: false, reason: 'degenerate' };

    const pts = uniqueSorted(positions);
    if (pts.length < 3) return { ok: false, reason: 'degenerate' };
    const lo: P3 = [Infinity, Infinity, Infinity], hi: P3 = [-Infinity, -Infinity, -Infinity];
    for (const p of pts) for (let i = 0; i < 3; i++) { lo[i] = Math.min(lo[i], p[i]); hi[i] = Math.max(hi[i], p[i]); }
    const center: P3 = [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2];
    const half: P3 = [(hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2, (hi[2] - lo[2]) / 2];
    const axisBox = (): Shape => ({ kind: 'box', half: half.map((h) => Math.max(h, MIN_HALF)) as unknown as Vec3, center, rotation: IDENTITY });
    const thin = half.filter((h) => h < MIN_HALF).length;
    if (thin >= 2) return { ok: false, reason: 'degenerate' };
    if (thin === 1) return { ok: true, shape: axisBox() };

    const extent = Math.max(...half);
    const planar = orientedFlatBox(pts, extent * 1e-4);
    if (planar) return { ok: true, shape: planar };
    const full = hullOf(pts);
    if (!full) return { ok: false, reason: 'degenerate' };

    // Hull volume, not mesh volume: double-sided, overlapping or inconsistently wound meshes cannot skew the fit.
    const fullVolume = loopsVolume(full.loops);
    const radii = pts.map((p) => Math.hypot(p[0] - center[0], p[1] - center[1], p[2] - center[2]));
    const meanRadius = radii.reduce((a, b) => a + b, 0) / radii.length;
    const round = radii.every((r) => Math.abs(r - meanRadius) <= SPHERE_RADIAL_SPREAD * meanRadius);
    // Cube and tetrahedron corners are equidistant from their centre too; only a mesh that fills the sphere is round.
    if (round && fullVolume / ((4 / 3) * Math.PI * meanRadius ** 3) >= SPHERE_FILL) {
        return { ok: true, shape: { kind: 'sphere', radius: meanRadius, center } };
    }
    if (fullVolume / (8 * half[0] * half[1] * half[2]) >= BOX_FILL) return { ok: true, shape: axisBox() };

    const reduced = full.vertices.length > MAX_HULL_VERTS ? hullOf(sample(full.vertices, MAX_HULL_VERTS)) : full;
    if (!reduced) return { ok: false, reason: 'degenerate' };
    const centroid = volumeCentroid(reduced.loops);
    if (!centroid) return { ok: false, reason: 'degenerate' };
    const recentred = reduced.vertices.map((v): P3 => [v[0] - centroid[0], v[1] - centroid[1], v[2] - centroid[2]]);
    const hullExtent = Math.max(...recentred.flat().map(Math.abs));
    if (hullExtent > MAX_HULL_COORD) return { ok: false, reason: 'too-large' };
    // The runtime rejects hulls below these sizes; a thin slab that is not axis aligned falls back to an oriented box.
    if (hullExtent < RUST_MIN_EXTENT || loopsVolume(reduced.loops) <= RUST_MIN_VOLUME * hullExtent ** 3) {
        const flat = orientedFlatBox(pts, extent * 1e-3);
        return { ok: true, shape: flat ?? axisBox() };
    }
    const flat = new Float32Array(recentred.length * 3);
    recentred.forEach((v, i) => flat.set(v, i * 3));
    return { ok: true, shape: { kind: 'hull', points: flat, center: centroid } };
}

export interface ColliderDesc {
    name: string;
    shape: Shape;
    /** World position of the shape's origin (for hulls, the volume centroid). */
    position: Vec3;
    quaternion: Quat4;
    /** Identical geometry at identical scale shares one uploaded hull. */
    geometryKey: string;
    node: Object3D;
}

export interface SkippedNode {
    name: string;
    reason: SkipReason;
}

export interface SceneColliders {
    colliders: ColliderDesc[];
    skipped: SkippedNode[];
}

type Attribute = BufferAttribute | InterleavedBufferAttribute;

export function collidersFromScene(root: Object3D): SceneColliders {
    const colliders: ColliderDesc[] = [];
    const skipped: SkippedNode[] = [];
    root.updateWorldMatrix(true, true);
    const position = new Vector3(), scale = new Vector3(), quaternion = new Quaternion(), matrix = new Matrix4();

    root.traverse((node) => {
        const mesh = node as Mesh;
        if (!mesh.isMesh) return;
        const name = node.name || node.uuid;
        const skip = (reason: SkipReason) => { skipped.push({ name, reason }); };
        if ((mesh as { isInstancedMesh?: boolean }).isInstancedMesh || (mesh as { isSkinnedMesh?: boolean }).isSkinnedMesh) return skip('unsupported');
        const attr = mesh.geometry?.getAttribute('position') as Attribute | undefined;
        if (!attr || attr.count === 0) return skip('no-geometry');

        matrix.copy(mesh.matrixWorld).decompose(position, quaternion, scale);
        const scaled = new Float32Array(attr.count * 3);
        for (let i = 0; i < attr.count; i++) {
            scaled[i * 3] = attr.getX(i) * scale.x;
            scaled[i * 3 + 1] = attr.getY(i) * scale.y;
            scaled[i * 3 + 2] = attr.getZ(i) * scale.z;
        }
        const index = mesh.geometry.index;
        const fit = fitShape(scaled, index ? (index.array as ArrayLike<number>) : null);
        if (!fit.ok) return skip(fit.reason);

        const c = fit.shape.center;
        const offset = new Vector3(c[0], c[1], c[2]).applyQuaternion(quaternion).add(position);
        const world = fit.shape.kind === 'box' ? quaternion.clone().multiply(new Quaternion(...fit.shape.rotation)) : quaternion;
        colliders.push({
            name,
            shape: fit.shape,
            position: [offset.x, offset.y, offset.z],
            quaternion: [world.x, world.y, world.z, world.w],
            geometryKey: `${mesh.geometry.uuid}:${scale.x},${scale.y},${scale.z}`,
            node,
        });
    });
    return { colliders, skipped };
}

export interface ColliderWorld {
    spawn(opts: SpawnOptions): EntityHandle;
    registerHull(points: ArrayLike<number>): number;
}

export interface BodyParams {
    invMass?: number;
    restitution?: number;
    friction?: number;
}

export interface SpawnColliderOptions {
    /** Per-node body parameters. Default: static (`invMass` 0). */
    body?: (node: Object3D) => BodyParams;
}

export interface SpawnedCollider {
    name: string;
    handle: EntityHandle;
    shape: Shape['kind'];
}

export interface SpawnReport {
    spawned: SpawnedCollider[];
    failed: { name: string; error: string }[];
}

export function spawnColliders(world: ColliderWorld, colliders: readonly ColliderDesc[], options: SpawnColliderOptions = {}): SpawnReport {
    const spawned: SpawnedCollider[] = [];
    const failed: { name: string; error: string }[] = [];
    const hulls = new Map<string, number>();
    for (const c of colliders) {
        const params = options.body?.(c.node) ?? {};
        try {
            const base: SpawnOptions = {
                x: c.position[0], y: c.position[1], z: c.position[2],
                qx: c.quaternion[0], qy: c.quaternion[1], qz: c.quaternion[2], qw: c.quaternion[3],
                invMass: params.invMass ?? 0,
                restitution: params.restitution ?? 0,
                friction: params.friction,
            };
            const s = c.shape;
            let handle: EntityHandle;
            if (s.kind === 'box') {
                handle = world.spawn({ ...base, shape: SHAPE_BOX, hx: s.half[0], hy: s.half[1], hz: s.half[2] });
            } else if (s.kind === 'sphere') {
                handle = world.spawn({ ...base, shape: SHAPE_SPHERE, hx: s.radius, hy: s.radius, hz: s.radius });
            } else {
                let hull = hulls.get(c.geometryKey);
                if (hull === undefined) {
                    hull = world.registerHull(s.points);
                    hulls.set(c.geometryKey, hull);
                }
                handle = world.spawn({ ...base, shape: SHAPE_HULL, hull });
            }
            spawned.push({ name: c.name, handle, shape: s.kind });
        } catch (err) {
            failed.push({ name: c.name, error: err instanceof Error ? err.message : String(err) });
        }
    }
    return { spawned, failed };
}
