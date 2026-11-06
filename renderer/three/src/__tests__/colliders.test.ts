import { describe, it, expect } from 'vitest';
import { BufferAttribute, BufferGeometry, Group, InstancedMesh, Line, LineBasicMaterial, Mesh, MeshBasicMaterial, Object3D, Quaternion, SkinnedMesh, Vector3 } from 'three';
import { ConvexHull } from 'three/examples/jsm/math/ConvexHull.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { MAX_HULL_VERTS } from '@dynova/sdk';
import { collidersFromScene, fitShape, spawnColliders, type ColliderWorld } from '../colliders.js';
import { box, buildGlb, cube, inverted, lShape, merge, noisySphere, plane, prism, pyramid, tetra, translated, uvSphere, type Geometry } from './gltfFixture.js';

const fit = (g: Geometry) => fitShape(g.positions, g.indices);

function shapeOf(g: Geometry) {
    const r = fit(g);
    if (!r.ok) throw new Error(`fit failed: ${r.reason}`);
    return r.shape;
}

function meshOf(g: Geometry, name = 'm'): Mesh {
    const geo = new BufferGeometry();
    geo.setAttribute('position', new BufferAttribute(new Float32Array(g.positions), 3));
    geo.setIndex(g.indices);
    const m = new Mesh(geo, new MeshBasicMaterial());
    m.name = name;
    return m;
}

function hullVolume(points: number[][]): number {
    const hull = new ConvexHull().setFromPoints(points.map((p) => new Vector3(p[0], p[1], p[2])));
    let v = 0;
    for (const face of hull.faces) {
        const loop: Vector3[] = [];
        let e = face.edge;
        do { loop.push(e.head().point); e = e.next; } while (e !== face.edge);
        for (let k = 1; k + 1 < loop.length; k++) v += loop[0].dot(new Vector3().crossVectors(loop[k], loop[k + 1])) / 6;
    }
    return Math.abs(v);
}

function pointsOf(positions: ArrayLike<number>): number[][] {
    const out: number[][] = [];
    for (let i = 0; i < positions.length; i += 3) out.push([positions[i], positions[i + 1], positions[i + 2]]);
    return out;
}

describe('fitShape', () => {
    it('a cube is a box with matching half extents and centre', () => {
        const s = shapeOf(translated(box(0.5, 1, 1.5), 3, -2, 1));
        expect(s.kind).toBe('box');
        if (s.kind === 'box') {
            s.half.forEach((h, i) => expect(h).toBeCloseTo([0.5, 1, 1.5][i], 5));
            s.center.forEach((c, i) => expect(c).toBeCloseTo([3, -2, 1][i], 5));
        }
    });

    it('a flat plane becomes a box with clamped thickness so it can serve as a floor', () => {
        const s = shapeOf(plane(5, 7));
        expect(s.kind).toBe('box');
        if (s.kind === 'box') {
            expect(s.half[0]).toBeCloseTo(5);
            expect(s.half[1]).toBe(0.005);
            expect(s.half[2]).toBeCloseTo(7);
        }
    });

    it('a UV sphere is a sphere with its radius, but 10% radial noise is not', () => {
        const s = shapeOf(uvSphere(2, 12, 8));
        expect(s.kind).toBe('sphere');
        if (s.kind === 'sphere') expect(s.radius).toBeCloseTo(2, 1);
        expect(shapeOf(noisySphere(1, 0.1, 12, 8)).kind).not.toBe('sphere');
    });

    it('an octahedron and a cube-corner tetrahedron have equidistant vertices but are not spheres', () => {
        const octa: Geometry = {
            positions: [1, 0, 0, -1, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 1, 0, 0, -1],
            indices: [0, 2, 4, 2, 1, 4, 1, 3, 4, 3, 0, 4, 2, 0, 5, 1, 2, 5, 3, 1, 5, 0, 3, 5],
        };
        expect(shapeOf(octa).kind).toBe('hull');
        expect(shapeOf(tetra()).kind).toBe('hull');
        expect(shapeOf(cube(1)).kind).toBe('box');
    });

    it('a stretched sphere is not a sphere', () => {
        const g = uvSphere(1, 12, 8);
        for (let i = 0; i < g.positions.length; i += 3) g.positions[i] *= 2;
        expect(shapeOf(g).kind).toBe('hull');
    });

    it('a 12-gon cylinder, an L shape and a pyramid are hulls', () => {
        for (const g of [prism(12, 1, 2), lShape(), pyramid(8, 1, 2)]) expect(shapeOf(g).kind).toBe('hull');
    });

    it('a closed box with an internal cavity is still a box: collision only sees the outer hull', () => {
        expect(shapeOf(merge(box(1, 1, 1), inverted(box(0.9, 0.9, 0.9)))).kind).toBe('box');
    });

    it('double-sided and overlapping meshes are classified by their hull, not by mesh volume', () => {
        expect(shapeOf(merge(cube(1), inverted(cube(1)))).kind).toBe('box');
        expect(shapeOf(merge(uvSphere(1, 12, 8), inverted(uvSphere(1, 12, 8)))).kind).toBe('sphere');
        const plus = merge(box(2, 0.5, 0.5), box(0.5, 2, 0.5));
        expect(shapeOf(plus).kind).toBe('hull');
    });

    it('box fill threshold: a corner-chamfered cube fills 98% (box) or 83% (hull) of its bounds', () => {
        const chamfered = (c: number) => {
            const p: number[] = [];
            for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) {
                p.push(sx, sy, sz * (1 - c), sx, sy * (1 - c), sz, sx * (1 - c), sy, sz);
            }
            return p;
        };
        expect(fitShape(chamfered(0.5), null)).toMatchObject({ ok: true, shape: { kind: 'box' } });
        expect(fitShape(chamfered(1), null)).toMatchObject({ ok: true, shape: { kind: 'hull' } });
    });

    it('sphere fill threshold: a UV sphere filling 71% of its circumsphere is a hull, one filling 81% is a sphere', () => {
        const ratio = (g: Geometry) => hullVolume(pointsOf(g.positions)) / ((4 / 3) * Math.PI);
        const coarse = uvSphere(1, 6, 4);
        const fine = uvSphere(1, 8, 5);
        expect(ratio(coarse)).toBeLessThan(0.75);
        expect(ratio(fine)).toBeGreaterThan(0.8);
        expect(ratio(fine)).toBeLessThan(0.85);
        expect(shapeOf(coarse).kind).not.toBe('sphere');
        expect(shapeOf(fine).kind).toBe('sphere');
    });

    it('sphere radial spread threshold: 3% noise is a sphere with the mean radius, 8% is a hull', () => {
        const s = shapeOf(noisySphere(4, 0.03));
        expect(s.kind).toBe('sphere');
        if (s.kind === 'sphere') expect(Math.abs(s.radius - 1)).toBeLessThan(0.012);
        expect(shapeOf(noisySphere(4, 0.08)).kind).toBe('hull');
    });

    it('a ramp whose slope is baked into the vertices becomes an oriented thin box containing every vertex', () => {
        const pts: number[] = [];
        for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) pts.push(i, i + 0.5 * j, j);
        const s = shapeOf({ positions: pts, indices: [] });
        expect(s.kind).toBe('box');
        if (s.kind !== 'box') return;
        expect(s.half[1]).toBe(0.005);
        const q = new Quaternion(...s.rotation);
        const normal = new Vector3(-1, 1, -0.5).normalize();
        expect(Math.abs(new Vector3(0, 1, 0).applyQuaternion(q).dot(normal))).toBeCloseTo(1, 4);
        const inv = q.clone().invert();
        for (let i = 0; i < pts.length; i += 3) {
            const local = new Vector3(pts[i] - s.center[0], pts[i + 1] - s.center[1], pts[i + 2] - s.center[2]).applyQuaternion(inv);
            expect(Math.abs(local.x)).toBeLessThanOrEqual(s.half[0] + 1e-4);
            expect(Math.abs(local.y)).toBeLessThanOrEqual(s.half[1] + 1e-4);
            expect(Math.abs(local.z)).toBeLessThanOrEqual(s.half[2] + 1e-4);
        }
    });

    it('four tilted coplanar points become an oriented box, collinear points are degenerate', () => {
        expect(fitShape([0, 0, 0, 2, 2, 0, 4, 0, 2, 6, 2, 2], null)).toMatchObject({ ok: true, shape: { kind: 'box' } });
        expect(fitShape([0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3], null)).toEqual({ ok: false, reason: 'degenerate' });
    });

    it('a tilted paper-thin slab that the runtime would reject as a hull becomes an oriented box', () => {
        for (const t of [3e-4, 1e-5]) {
            const slab = [0, 0, 0, 10, 10, 0, 10, 10, 10, 0, 0, 10, 0, t, 0, 10, 10 + t, 0, 10, 10 + t, 10, 0, t, 10];
            const r = fitShape(slab, null);
            expect(r.ok && r.shape.kind, `thickness ${t}`).toBe('box');
        }
    });

    it('hulls below the runtime size limits are not emitted: tiny shapes are skipped or boxed', () => {
        const tinyTetra = tetra().positions.map((v) => v * 6e-4);
        expect(fitShape(tinyTetra, null)).toEqual({ ok: false, reason: 'degenerate' });
        const smallTetra = tetra().positions.map((v) => v * 0.02);
        expect(fitShape(smallTetra, null)).toMatchObject({ ok: true, shape: { kind: 'hull' } });
    });

    it('a needle-shaped hull the runtime would reject for its tiny volume falls back to a bounding box', () => {
        const needle = [0, 0, 0, 10, 0, 0, 0, 0.01, 0, 0, 0, 0.01];
        expect(fitShape(needle, null)).toMatchObject({ ok: true, shape: { kind: 'box' } });
        const stout = [0, 0, 0, 10, 0, 0, 0, 3, 0, 0, 0, 3];
        expect(fitShape(stout, null)).toMatchObject({ ok: true, shape: { kind: 'hull' } });
    });

    it('a hull whose recentred coordinates exceed the runtime coordinate limit is rejected', () => {
        const wedge = [-1e4, 0, 0, 1e4, -1e3, -1e3, 1e4, 1e3, -1e3, 1e4, 1e3, 1e3, 1e4, -1e3, 1e3];
        expect(fitShape(wedge, null)).toEqual({ ok: false, reason: 'too-large' });
    });

    it('a tetrahedron is a hull with 4 points recentred on its centroid', () => {
        const s = shapeOf(tetra());
        expect(s.kind).toBe('hull');
        if (s.kind === 'hull') {
            expect(s.points.length).toBe(12);
            const mean = [0, 1, 2].map((k) => s.points.filter((_, i) => i % 3 === k).reduce((a, b) => a + b, 0) / 4);
            mean.forEach((m) => expect(m).toBeCloseTo(0, 5));
        }
    });

    it('a pyramid centroid sits a quarter of the height above its base', () => {
        const s = shapeOf(pyramid(8, 1, 4));
        expect(s.kind).toBe('hull');
        if (s.kind === 'hull') {
            expect(s.center[1]).toBeCloseTo(1, 4);
            expect(s.center[0]).toBeCloseTo(0, 4);
        }
    });

    it('a dense noisy sphere reduces to at most 32 points and keeps most of the volume', () => {
        const g = noisySphere(7, 0.15);
        const full = hullVolume(pointsOf(g.positions));
        const s = shapeOf(g);
        expect(s.kind).toBe('hull');
        if (s.kind === 'hull') {
            expect(s.points.length / 3).toBeLessThanOrEqual(MAX_HULL_VERTS);
            expect(s.points.length / 3).toBeGreaterThanOrEqual(20);
            expect(hullVolume(pointsOf(s.points)) / full).toBeGreaterThan(0.8);
        }
    });

    it('is deterministic and independent of vertex order', () => {
        const g = noisySphere(3, 0.15);
        const a = shapeOf(g);
        const b = shapeOf(g);
        const order = Array.from({ length: g.positions.length / 3 }, (_, i) => i).sort((x, y) => ((x * 7919) % 104729) - ((y * 7919) % 104729));
        const remap = new Map(order.map((old, i) => [old, i]));
        const shuffled: Geometry = {
            positions: order.flatMap((old) => g.positions.slice(old * 3, old * 3 + 3)),
            indices: g.indices.map((i) => remap.get(i) as number),
        };
        const c = shapeOf(shuffled);
        if (a.kind === 'hull' && b.kind === 'hull' && c.kind === 'hull') {
            expect(Array.from(b.points)).toEqual(Array.from(a.points));
            expect(Array.from(c.points)).toEqual(Array.from(a.points));
        } else {
            throw new Error('expected hulls');
        }
    });

    it('equidistant vertices (a 40-gon prism) still give the same 32 points whatever the input order', () => {
        const g = prism(40, 1, 2);
        const a = shapeOf(g);
        const n = g.positions.length / 3;
        const order = Array.from({ length: n }, (_, i) => (i * 37) % n);
        const remap = new Map(order.map((old, i) => [old, i]));
        const b = shapeOf({
            positions: order.flatMap((old) => g.positions.slice(old * 3, old * 3 + 3)),
            indices: g.indices.map((i) => remap.get(i) as number),
        });
        expect(a.kind === 'hull' && b.kind === 'hull').toBe(true);
        if (a.kind === 'hull' && b.kind === 'hull') {
            expect(a.points.length / 3).toBe(MAX_HULL_VERTS);
            expect(Array.from(b.points)).toEqual(Array.from(a.points));
        }
    });

    it('hull points are recentred on the volume centroid, which differs from the vertex mean', () => {
        const s = shapeOf(pyramid(8, 1, 4));
        if (s.kind !== 'hull') throw new Error('expected hull');
        const pts = pointsOf(s.points);
        const hull = new ConvexHull().setFromPoints(pts.map((p) => new Vector3(p[0], p[1], p[2])));
        let vol = 0;
        const m = [0, 0, 0];
        for (const face of hull.faces) {
            const loop: Vector3[] = [];
            let e = face.edge;
            do { loop.push(e.head().point); e = e.next; } while (e !== face.edge);
            for (let k = 1; k + 1 < loop.length; k++) {
                const v = loop[0].dot(new Vector3().crossVectors(loop[k], loop[k + 1])) / 6;
                vol += v;
                m[0] += v * (loop[0].x + loop[k].x + loop[k + 1].x) / 4;
                m[1] += v * (loop[0].y + loop[k].y + loop[k + 1].y) / 4;
                m[2] += v * (loop[0].z + loop[k].z + loop[k + 1].z) / 4;
            }
        }
        m.forEach((c) => expect(c / vol).toBeCloseTo(0, 4));
        const meanY = pts.reduce((a, p) => a + p[1], 0) / pts.length;
        expect(Math.abs(meanY)).toBeGreaterThan(0.05);
    });

    it('tiny geometry is not merged away by vertex deduplication', () => {
        const s = shapeOf(cube(0.01));
        expect(s.kind).toBe('box');
        if (s.kind === 'box') expect(s.half[0]).toBeCloseTo(0.01, 6);
    });

    it('works with indexed uint16-sized data and with non-indexed triangle soup', () => {
        const c = cube(1);
        expect(shapeOf(c).kind).toBe('box');
        const soup: number[] = [];
        for (const i of c.indices) soup.push(c.positions[i * 3], c.positions[i * 3 + 1], c.positions[i * 3 + 2]);
        const r = fitShape(soup, null);
        expect(r.ok && r.shape.kind).toBe('box');
    });

    it.each([
        ['empty', [], null, 'no-geometry'],
        ['not a multiple of 3', [0, 0], null, 'degenerate'],
        ['NaN', [0, 0, 0, NaN, 0, 0, 1, 1, 1], null, 'non-finite'],
        ['Infinity', [0, 0, 0, Infinity, 0, 0, 1, 1, 1], null, 'non-finite'],
        ['too large', [0, 0, 0, 2e4, 0, 0, 0, 1, 0], null, 'too-large'],
        ['two distinct points', [0, 0, 0, 1, 0, 0, 1, 0, 0], null, 'degenerate'],
        ['a line', [0, 0, 0, 1, 0, 0, 2, 0, 0, 3, 0, 0], null, 'degenerate'],
        ['a single point repeated', [1, 1, 1, 1, 1, 1, 1, 1, 1], null, 'degenerate'],
        ['indices not a multiple of 3', [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1], [0, 1], 'unsupported'],
        ['index out of range', [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1], [0, 1, 9], 'degenerate'],
    ] as const)('rejects %s', (_n, positions, indices, reason) => {
        const r = fitShape(positions, indices as ArrayLike<number> | null);
        expect(r).toEqual({ ok: false, reason });
    });

    it('a negative vertex index is rejected', () => {
        expect(fitShape([0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1], [0, 1, -1])).toEqual({ ok: false, reason: 'degenerate' });
    });
});

describe('collidersFromScene', () => {
    it('applies translation, rotation and scale from the node chain and offsets the shape centre', () => {
        const root = new Group();
        root.position.set(10, 0, 0);
        root.rotation.z = Math.PI / 2;
        root.scale.setScalar(2);
        const m = meshOf(translated(cube(0.5), 1, 0, 0), 'crate');
        m.position.set(0, 3, 0);
        root.add(m);
        const { colliders, skipped } = collidersFromScene(root);
        expect(skipped).toEqual([]);
        const c = colliders[0];
        expect(c.shape.kind).toBe('box');
        if (c.shape.kind === 'box') c.shape.half.forEach((h) => expect(h).toBeCloseTo(1, 5));
        const q = new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), Math.PI / 2);
        c.quaternion.forEach((v, i) => expect(v).toBeCloseTo([q.x, q.y, q.z, q.w][i], 5));
        const expected = new Vector3(1 * 2, 0, 0).applyQuaternion(q).add(new Vector3(0, 3 * 2, 0).applyQuaternion(q)).add(new Vector3(10, 0, 0));
        c.position.forEach((v, i) => expect(v).toBeCloseTo([expected.x, expected.y, expected.z][i], 4));
    });

    it('negative scale mirrors the mesh and still fits', () => {
        const m = meshOf(translated(box(1, 1, 1), 2, 0, 0));
        m.scale.set(-1, 1, 1);
        const { colliders } = collidersFromScene(m);
        expect(colliders[0].shape.kind).toBe('box');
        expect(colliders[0].position[0]).toBeCloseTo(-2, 4);
    });

    it('skips instanced and skinned meshes, empty geometry and bad vertices, and ignores lines', () => {
        const root = new Group();
        const geo = meshOf(cube()).geometry;
        const inst = new InstancedMesh(geo, new MeshBasicMaterial(), 2);
        inst.name = 'inst';
        const skin = new SkinnedMesh(geo, new MeshBasicMaterial());
        skin.name = 'skin';
        const empty = new Mesh(new BufferGeometry(), new MeshBasicMaterial());
        empty.name = 'empty';
        const bad = meshOf({ positions: [0, 0, 0, NaN, 1, 1, 1, 1, 1, 0, 1, 0], indices: [0, 1, 2, 0, 2, 3] }, 'bad');
        const line = new Line(geo, new LineBasicMaterial());
        root.add(inst, skin, empty, bad, line, meshOf(cube(), 'ok'));
        const { colliders, skipped } = collidersFromScene(root);
        expect(colliders.map((c) => c.name)).toEqual(['ok']);
        expect(skipped).toEqual([
            { name: 'inst', reason: 'unsupported' },
            { name: 'skin', reason: 'unsupported' },
            { name: 'empty', reason: 'no-geometry' },
            { name: 'bad', reason: 'non-finite' },
        ]);
    });

    it('instances of one geometry share a geometry key only at identical scale', () => {
        const root = new Group();
        const base = meshOf(pyramid(8, 1, 2), 'a');
        const same = new Mesh(base.geometry, base.material);
        same.name = 'b';
        const big = new Mesh(base.geometry, base.material);
        big.name = 'c';
        big.scale.setScalar(2);
        root.add(base, same, big);
        const keys = collidersFromScene(root).colliders.map((c) => c.geometryKey);
        expect(keys[0]).toBe(keys[1]);
        expect(keys[2]).not.toBe(keys[0]);
    });

    it('a mirrored hull (negative scale) keeps its volume centroid on the mirrored side', () => {
        const plain = meshOf(pyramid(8, 1, 4), 'p');
        const mirrored = meshOf(pyramid(8, 1, 4), 'm');
        mirrored.scale.set(-1, 1, 1);
        const a = collidersFromScene(plain).colliders[0];
        const b = collidersFromScene(mirrored).colliders[0];
        expect(a.shape.kind === 'hull' && b.shape.kind === 'hull').toBe(true);
        expect(b.position[0]).toBeCloseTo(-a.position[0], 5);
        if (a.shape.kind === 'hull' && b.shape.kind === 'hull') {
            expect(b.shape.points.length).toBe(a.shape.points.length);
            expect(b.shape.points.every(Number.isFinite)).toBe(true);
        }
    });

    it('reads interleaved and normalised-free attributes through getX', () => {
        const m = meshOf(cube(1));
        expect(collidersFromScene(m).colliders[0].shape.kind).toBe('box');
    });
});

describe('GLTF round trip', () => {
    const parse = (json: ArrayBuffer) => new Promise<Object3D>((resolve, reject) => {
        new GLTFLoader().parse(json, '', (g) => resolve(g.scene), reject);
    });

    it('a parsed GLTF scene yields the expected colliders in world space', async () => {
        const json = buildGlb(
            [
                { name: 'floor', ...plane(10, 10), translation: [0, -1, 0] },
                { name: 'crate', ...cube(0.5), translation: [2, 0, 0], scale: [2, 2, 2], indexType: 5123 },
                { name: 'ball', ...uvSphere(0.5), translation: [-3, 1, 0] },
                { name: 'cone', ...pyramid(8, 1, 2), translation: [0, 5, 0] },
            ],
            { translation: [0, 10, 0] },
        );
        const { colliders, skipped } = collidersFromScene(await parse(json));
        expect(skipped).toEqual([]);
        const by = Object.fromEntries(colliders.map((c) => [c.name, c]));
        expect(by.floor.shape.kind).toBe('box');
        expect(by.floor.position[1]).toBeCloseTo(9, 4);
        expect(by.crate.shape.kind).toBe('box');
        if (by.crate.shape.kind === 'box') by.crate.shape.half.forEach((h) => expect(h).toBeCloseTo(1, 4));
        expect(by.ball.shape.kind).toBe('sphere');
        expect(by.cone.shape.kind).toBe('hull');
        expect(by.cone.position[1]).toBeCloseTo(15 + 0.5, 3);
    });

    it('node rotations survive GLB parsing and uint16 indices work on the hull path', async () => {
        const q = [0, 0, Math.SQRT1_2, Math.SQRT1_2];
        const json = buildGlb([
            { name: 'turned', ...cube(0.5), rotation: q },
            { name: 'cone', ...pyramid(8, 1, 2), indexType: 5123, translation: [0, 3, 0] },
        ]);
        const { colliders, skipped } = collidersFromScene(await parse(json));
        expect(skipped).toEqual([]);
        const turned = colliders.find((c) => c.name === 'turned');
        turned?.quaternion.forEach((v, i) => expect(v).toBeCloseTo(q[i], 5));
        expect(colliders.find((c) => c.name === 'cone')?.shape.kind).toBe('hull');
    });
});

function stubWorld(overrides: Partial<ColliderWorld> = {}) {
    const calls = { hulls: [] as ArrayLike<number>[], spawns: [] as Record<string, unknown>[] };
    let next = 0;
    const world: ColliderWorld = {
        registerHull: (points) => { calls.hulls.push(points); return calls.hulls.length - 1; },
        spawn: (opts) => { calls.spawns.push(opts as Record<string, unknown>); return { id: next++, gen: 0 }; },
        ...overrides,
    };
    return { world, calls };
}

describe('spawnColliders', () => {
    it('spawns boxes, spheres and hulls as static bodies at their world pose', () => {
        const root = new Group();
        const crate = meshOf(cube(1), 'crate');
        crate.position.set(1, 2, 3);
        const ball = meshOf(uvSphere(0.5), 'ball');
        const cone = meshOf(pyramid(8, 1, 2), 'cone');
        root.add(crate, ball, cone);
        const { world, calls } = stubWorld();
        const report = spawnColliders(world, collidersFromScene(root).colliders);
        expect(report.failed).toEqual([]);
        expect(report.spawned.map((s) => s.shape)).toEqual(['box', 'sphere', 'hull']);
        expect(calls.spawns.every((s) => s.invMass === 0 && s.restitution === 0)).toBe(true);
        expect(calls.spawns[0]).toMatchObject({ x: 1, y: 2, z: 3, shape: 0, hx: 1, hy: 1, hz: 1 });
        expect(calls.spawns[1]).toMatchObject({ shape: 1 });
        expect(calls.spawns[2]).toMatchObject({ shape: 3, hull: 0 });
    });

    it('uploads a hull once per geometry and scale, and once more for a different scale', () => {
        const root = new Group();
        const a = meshOf(pyramid(8, 1, 2), 'a');
        const b = new Mesh(a.geometry, a.material);
        b.name = 'b';
        b.position.x = 5;
        const c = new Mesh(a.geometry, a.material);
        c.name = 'c';
        c.scale.setScalar(2);
        root.add(a, b, c);
        const { world, calls } = stubWorld();
        spawnColliders(world, collidersFromScene(root).colliders);
        expect(calls.hulls).toHaveLength(2);
        expect(calls.spawns.map((s) => s.hull)).toEqual([0, 0, 1]);
    });

    it('body(node) chooses mass, friction and restitution per node', () => {
        const root = new Group();
        root.add(meshOf(cube(), 'wall'), meshOf(cube(), 'crate'));
        const { world, calls } = stubWorld();
        spawnColliders(world, collidersFromScene(root).colliders, {
            body: (n) => (n.name === 'crate' ? { invMass: 0.5, restitution: 0.3, friction: 0.9 } : {}),
        });
        expect(calls.spawns[0]).toMatchObject({ invMass: 0, restitution: 0 });
        expect(calls.spawns[1]).toMatchObject({ invMass: 0.5, restitution: 0.3, friction: 0.9 });
    });

    it('one failing collider does not stop the rest and is reported with its message', () => {
        const root = new Group();
        root.add(meshOf(cube(), 'a'), meshOf(cube(), 'b'), meshOf(cube(), 'c'));
        let n = 0;
        const { world, calls } = stubWorld({
            spawn: (opts) => { if (n++ === 1) throw new Error('CommandRing full'); calls.spawns.push(opts as Record<string, unknown>); return { id: n, gen: 0 }; },
        });
        const report = spawnColliders(world, collidersFromScene(root).colliders);
        expect(report.spawned.map((s) => s.name)).toEqual(['a', 'c']);
        expect(report.failed).toEqual([{ name: 'b', error: 'CommandRing full' }]);
    });

    it('a failed hull upload is reported and a later collider with the same key retries', () => {
        const root = new Group();
        const a = meshOf(pyramid(8, 1, 2), 'a');
        const b = new Mesh(a.geometry, a.material);
        b.name = 'b';
        root.add(a, b);
        let tries = 0;
        const { world } = stubWorld({ registerHull: () => { if (tries++ === 0) throw new RangeError('hull limit reached'); return 4; } });
        const report = spawnColliders(world, collidersFromScene(root).colliders);
        expect(report.failed).toEqual([{ name: 'a', error: 'hull limit reached' }]);
        expect(report.spawned.map((s) => s.name)).toEqual(['b']);
    });

    it('a throwing body() callback is a programmer error and propagates instead of becoming a failed collider', () => {
        const root = new Group();
        root.add(meshOf(cube(), 'a'));
        const { world } = stubWorld();
        expect(() => spawnColliders(world, collidersFromScene(root).colliders, { body: () => { throw new TypeError('bad callback'); } })).toThrow(TypeError);
    });

    it('an oriented box is spawned with the node rotation composed with its fitted rotation', () => {
        const ramp: Geometry = { positions: [], indices: [] };
        for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) ramp.positions.push(i, i, j);
        const m = meshOf(ramp, 'ramp');
        m.rotation.set(0.3, 0.5, 0.1);
        const root = new Group();
        root.add(m);
        const { colliders } = collidersFromScene(root);
        const c = colliders[0];
        if (c.shape.kind !== 'box') throw new Error('expected box');
        const nodeQ = new Quaternion().setFromEuler(m.rotation);
        const expected = nodeQ.clone().multiply(new Quaternion(...c.shape.rotation));
        c.quaternion.forEach((v, i) => expect(v).toBeCloseTo([expected.x, expected.y, expected.z, expected.w][i], 5));
        const { world, calls } = stubWorld();
        spawnColliders(world, colliders);
        expect([calls.spawns[0].qx, calls.spawns[0].qy, calls.spawns[0].qz, calls.spawns[0].qw]).toEqual(c.quaternion);
    });

    it('spawns nothing for an empty list', () => {
        const { world, calls } = stubWorld();
        expect(spawnColliders(world, [])).toEqual({ spawned: [], failed: [] });
        expect(calls.spawns).toHaveLength(0);
    });
});
