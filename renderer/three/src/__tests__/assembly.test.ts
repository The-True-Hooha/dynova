import { describe, it, expect } from 'vitest';
import { BoxGeometry, BufferGeometry, Float32BufferAttribute, Group, Mesh, MeshBasicMaterial, SphereGeometry, type Object3D } from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { importAssembly, type AssemblyHost, type MateExtras, type PartExtras } from '../assembly.js';
import { buildGlb, cube, pyramid } from '../testing/gltfFixture.js';
import type { SpawnOptions } from '@dynova/sdk';

interface JointCall { kind: number; a: number; b: number; anchor: number[]; axis?: number[]; anchorB?: number[]; limits?: number[] }

/** Records what the importer asks of a world; enough of `World` for `Assembly` to run on. */
function stub() {
    const calls = { spawns: [] as SpawnOptions[], joints: [] as JointCall[], hulls: 0 };
    let next = 0;
    const world = {
        spawn: (o: SpawnOptions) => { calls.spawns.push(o); return { id: next++, gen: 0 }; },
        registerHull: () => calls.hulls++,
        createJoint: (o: JointCall & { a: { id: number }; b: { id: number } }) => { calls.joints.push({ ...o, a: o.a.id, b: o.b.id }); return { id: calls.joints.length - 1 }; },
        readJointStates: () => [],
        driveJoints: () => [],
    } as unknown as AssemblyHost;
    return { world, calls };
}

const material = new MeshBasicMaterial();
function part(name: string, size: [number, number, number], at: [number, number, number], extras?: PartExtras): Mesh {
    const m = new Mesh(new BoxGeometry(...size), material);
    m.name = name;
    m.position.set(...at);
    if (extras) m.userData.dynova = extras;
    return m;
}
const mate = (m: MateExtras): PartExtras => ({ mate: m });

describe('importAssembly parts', () => {
    it('makes every mesh a part under its node name, dynamic with mass 1 unless it says otherwise', () => {
        const root = new Group();
        root.add(part('base', [1, 1, 1], [0, 0, 0], { static: true }), part('arm', [1, 1, 1], [3, 0, 0]), part('heavy', [1, 1, 1], [6, 0, 0], { mass: 4 }), part('pinned', [1, 1, 1], [9, 0, 0], { mass: 0 }));
        const { world, calls } = stub();
        const r = importAssembly(root, world);
        expect(r.parts).toEqual(['base', 'arm', 'heavy', 'pinned']);
        expect(r.errors).toEqual([]);
        expect(calls.spawns.map((s) => s.invMass)).toEqual([0, 1, 0.25, 0]);
        expect(r.assembly.partNames).toEqual(r.parts);
    });

    it('passes friction and restitution through and reports meshes it cannot use', () => {
        const root = new Group();
        root.add(part('a', [1, 1, 1], [0, 0, 0], { friction: 0.3, restitution: 0.2 }));
        const empty = new Mesh(new BufferGeometry(), material);
        empty.name = 'empty';
        root.add(empty);
        const { world, calls } = stub();
        const r = importAssembly(root, world);
        expect(calls.spawns[0]).toMatchObject({ friction: 0.3, restitution: 0.2 });
        expect(r.skipped).toEqual([{ name: 'empty', reason: 'no-geometry' }]);
        expect(r.parts).toEqual(['a']);
    });

    it('reports a duplicate name, a bad mass and a bad extras object without stopping the others', () => {
        const root = new Group();
        const bad = part('bad', [1, 1, 1], [0, 0, 0]);
        bad.userData.dynova = [1, 2];
        root.add(part('p', [1, 1, 1], [0, 0, 0]), part('p', [1, 1, 1], [3, 0, 0]), part('neg', [1, 1, 1], [6, 0, 0], { mass: -1 }), part('nan', [1, 1, 1], [9, 0, 0], { mass: NaN }), bad, part('ok', [1, 1, 1], [12, 0, 0]));
        const r = importAssembly(root, stub().world);
        expect(r.parts).toEqual(['p', 'ok']);
        expect(r.errors.map((e) => e.name)).toEqual(['p', 'neg', 'nan', 'bad']);
        expect(r.errors[0].error).toMatch(/share this name/);
        expect(r.errors[1].error).toMatch(/mass must be a number/);
        expect(r.errors[3].error).toMatch(/must be an object/);
    });

    it('uploads one hull for parts that share geometry and scale', () => {
        const geo = new BufferGeometry();
        const p = pyramid(8, 1, 2);
        geo.setAttribute('position', new Float32BufferAttribute(p.positions, 3));
        geo.setIndex(p.indices);
        const root = new Group();
        for (const [i, x] of [0, 5, 10].entries()) {
            const m = new Mesh(geo, material);
            m.name = `cone${i}`;
            m.position.x = x;
            root.add(m);
        }
        const { world, calls } = stub();
        const r = importAssembly(root, world);
        expect(r.errors).toEqual([]);
        expect(calls.hulls).toBe(1);
        expect(calls.spawns.every((s) => s.shape === 3 && s.hull === 0)).toBe(true);
    });

    it('a world that refuses a part leaves the rest and a row for it', () => {
        const root = new Group();
        root.add(part('a', [1, 1, 1], [0, 0, 0]), part('b', [1, 1, 1], [3, 0, 0]), part('c', [1, 1, 1], [6, 0, 0]));
        const { world } = stub();
        let n = 0;
        const refusing = { ...world, spawn: (o: SpawnOptions) => { if (n++ === 1) throw new RangeError('refused'); return world.spawn(o); } } as AssemblyHost;
        const r = importAssembly(root, refusing);
        expect(r.parts).toEqual(['a', 'c']);
        expect(r.errors).toEqual([{ name: 'b', error: 'refused' }]);
    });
});

describe('importAssembly origin', () => {
    const far = (x: number) => { const r = new Group(); r.add(part('a', [1, 1, 1], [x, 0, 0], { static: true }), part('b', [1, 1, 1], [x + 4, 2, 6])); return r; };

    it('defaults to the middle of the parts, so a model modelled far away is spawned near zero', () => {
        const { world, calls } = stub();
        const r = importAssembly(far(600_000), world);
        expect(r.assembly.origin).toEqual([600_002, 1, 3]);
        expect(calls.spawns.map((s) => [s.x, s.y, s.z])).toEqual([[-2, -1, -3], [2, 1, 3]]);
    });

    it('takes an explicit origin, including the world origin to switch the floating origin off', () => {
        const a = importAssembly(far(100), stub().world, { origin: [100, 0, 0] });
        expect(a.assembly.origin).toEqual([100, 0, 0]);
        const { world, calls } = stub();
        importAssembly(far(100), world, { origin: [0, 0, 0] });
        expect(calls.spawns[0].x).toBe(100);
    });

    it('rejects a malformed origin and handles a scene with no parts', () => {
        expect(() => importAssembly(far(0), stub().world, { origin: [NaN, 0, 0] })).toThrow(/origin/);
        const r = importAssembly(new Group(), stub().world);
        expect(r.assembly.origin).toEqual([0, 0, 0]);
        expect(r.parts).toEqual([]);
    });
});

describe('importAssembly mates', () => {
    function arm(extras: MateExtras | MateExtras[]) {
        const root = new Group();
        const base = part('base', [1, 1, 1], [0, 0, 0], { static: true });
        const link = part('link', [2, 0.2, 0.2], [1.5, 0, 0], { mate: extras });
        root.add(base, link);
        return root;
    }

    it('turns each mate type into the matching joint at the authored pose', () => {
        for (const [spec, kind] of [
            [{ type: 'fixed' }, 2], [{ type: 'ball' }, 0], [{ type: 'revolute', axis: [0, 0, 1] }, 1], [{ type: 'slider', axis: [1, 0, 0] }, 3],
        ] as Array<[MateExtras, number]>) {
            const { world, calls } = stub();
            const r = importAssembly(arm({ ...spec, with: 'base' }), world, { origin: [0, 0, 0] });
            expect(r.errors, spec.type).toEqual([]);
            expect(calls.joints[0].kind).toBe(kind);
            expect([calls.joints[0].a, calls.joints[0].b]).toEqual([0, 1]);
            expect(r.mates).toEqual(['link']);
        }
    });

    it('takes the mate point and axis in the node frame, so a rotated, moved and scaled node carries them along', () => {
        const root = new Group();
        root.add(part('base', [1, 1, 1], [0, 0, 0], { static: true }));
        const link = part('link', [1, 1, 1], [10, 0, 0], { mate: { type: 'revolute', with: 'base', at: [1, 0, 0], axis: [0, 0, 1], range: [-1, 1] } });
        link.rotation.y = Math.PI / 2;
        link.scale.set(2, 2, 2);
        root.add(link);
        const { world, calls } = stub();
        importAssembly(root, world, { origin: [0, 0, 0] });
        const j = calls.joints[0];
        expect(j.anchor[0]).toBeCloseTo(10, 5);
        expect(j.anchor[2]).toBeCloseTo(-2, 5);
        expect(j.axis![0]).toBeCloseTo(1, 5);
        expect(Math.abs(j.axis![2])).toBeLessThan(1e-6);
        expect(j.limits).toEqual([-1, 1]);
    });

    it('defaults the partner to the nearest part above in the hierarchy and the point to the node origin', () => {
        const root = new Group();
        const base = part('base', [1, 1, 1], [0, 0, 0], { static: true });
        const mid = part('mid', [1, 1, 1], [3, 0, 0]);
        const tip = part('tip', [1, 1, 1], [3, 0, 0], { mate: { type: 'ball' } });
        mid.userData.dynova = { mate: { type: 'ball' } };
        base.add(mid);
        mid.add(tip);
        root.add(base);
        const { world, calls } = stub();
        const r = importAssembly(root, world, { origin: [0, 0, 0] });
        expect(r.errors).toEqual([]);
        expect(r.mates).toEqual(['mid', 'tip']);
        expect(calls.joints.map((j) => [j.a, j.b])).toEqual([[0, 1], [1, 2]]);
        expect(calls.joints[1].anchor).toEqual([6, 0, 0]);
    });

    it('a part can carry several mates, named by position, so a connecting rod can close a loop', () => {
        const root = new Group();
        root.add(part('base', [1, 1, 1], [0, 0, 0], { static: true }), part('crank', [1, 1, 1], [1, 0, 0]), part('slide', [1, 1, 1], [5, 0, 0]));
        const rod = part('rod', [3, 0.1, 0.1], [3, 0, 0], {
            mate: [
                { type: 'revolute', with: 'crank', at: [-1.5, 0, 0], axis: [0, 0, 1] },
                { type: 'distance', with: 'slide', from: [0, 0, 0], to: [1.5, 0, 0] },
            ],
        });
        root.add(rod);
        const { world, calls } = stub();
        const r = importAssembly(root, world, { origin: [0, 0, 0] });
        expect(r.errors).toEqual([]);
        expect(r.mates).toEqual(['rod#0', 'rod#1']);
        expect(calls.joints[1]).toMatchObject({ kind: 4, anchor: [5, 0, 0], anchorB: [4.5, 0, 0] });
    });

    it('reports a bad mate as a row and still makes the others', () => {
        const root = new Group();
        root.add(part('base', [1, 1, 1], [0, 0, 0], { static: true }));
        const bads: Array<[string, unknown]> = [
            ['unknown', { type: 'weld', with: 'base' }], ['missing', { type: 'ball', with: 'nowhere' }], ['self', { type: 'ball', with: 'self' }],
            ['noaxis', { type: 'revolute', with: 'base' }], ['badat', { type: 'ball', with: 'base', at: [1, NaN, 0] }], ['badrange', { type: 'slider', with: 'base', axis: [1, 0, 0], range: [1] }],
            ['nofrom', { type: 'distance', with: 'base', to: [1, 0, 0] }], ['notobj', 7], ['orphan', { type: 'ball' }],
        ];
        for (const [i, [name, m]] of bads.entries()) root.add(part(name, [1, 1, 1], [3 + i * 3, 0, 0], { mate: m as MateExtras }));
        root.add(part('good', [1, 1, 1], [60, 0, 0], { mate: { type: 'ball', with: 'base' } }));
        const { world, calls } = stub();
        const r = importAssembly(root, world, { origin: [0, 0, 0] });
        expect(r.errors.map((e) => e.name)).toEqual(['unknown', 'missing', 'self', 'noaxis', 'badat', 'badrange', 'nofrom', 'notobj', 'orphan']);
        expect(r.errors[0].error).toMatch(/unknown mate type 'weld'/);
        expect(r.errors[1].error).toMatch(/no part named 'nowhere'/);
        expect(r.errors[2].error).toMatch(/itself/);
        expect(r.errors[8].error).toMatch(/no partner/);
        expect(r.mates).toEqual(['good']);
        expect(calls.joints).toHaveLength(1);
    });

    it('passes the world its own validation errors (limits that exclude the pose) as rows', () => {
        const { world } = stub();
        const strict = { ...world, createJoint: () => { throw new RangeError('joint limits must satisfy lo <= 0 <= hi'); } } as AssemblyHost;
        const r = importAssembly(arm({ type: 'revolute', with: 'base', axis: [0, 0, 1], range: [0.5, 1] }), strict);
        expect(r.mates).toEqual([]);
        expect(r.errors).toEqual([{ name: 'link', error: expect.stringMatching(/lo <= 0 <= hi/) }]);
    });
});

describe('importAssembly from a glTF file', () => {
    const parse = (glb: ArrayBuffer) => new Promise<Object3D>((resolve, reject) => { new GLTFLoader().parse(glb, '', (g) => resolve(g.scene), reject); });

    it('reads parts, nesting and mates from node extras through GLTFLoader', async () => {
        const glb = buildGlb([
            { name: 'base', ...cube(0.5), extras: { dynova: { static: true } } },
            { name: 'arm', ...cube(0.5), translation: [2, 0, 0], parent: 0, extras: { dynova: { mass: 2, mate: { type: 'revolute', at: [-1, 0, 0], axis: [0, 0, 1], range: [-1.2, 1.2] } } } },
            { name: 'cone', ...pyramid(8, 1, 2), translation: [0, 5, 0], extras: { dynova: { static: true } } },
        ], { translation: [1000, 0, 0] });
        const { world, calls } = stub();
        const r = importAssembly(await parse(glb), world);
        expect(r.errors).toEqual([]);
        expect(r.parts).toEqual(['base', 'arm', 'cone']);
        expect(r.mates).toEqual(['arm']);
        expect(calls.spawns[1].invMass).toBe(0.5);
        expect(calls.spawns[2].shape).toBe(3);
        const j = calls.joints[0];
        expect([j.kind, j.a, j.b, j.limits]).toEqual([1, 0, 1, [-1.2, 1.2]]);
        expect(r.assembly.origin[0]).toBeCloseTo(1001, 3);
        expect(Math.abs(calls.spawns[0].x as number)).toBeLessThan(2.2);
    });
});

describe('importAssembly review fixes', () => {
    const run = (root: Object3D, options = {}) => { const { world, calls } = stub(); return { r: importAssembly(root, world, options), calls }; };
    const group = (...kids: Object3D[]) => { const g = new Group(); g.add(...kids); return g; };

    it('refuses part properties the world would silently drop, each as its own row, and does not spawn them', () => {
        const bad: Array<[string, PartExtras | Record<string, unknown>]> = [
            ['tiny', { mass: 1e-40 }], ['inf', { mass: Infinity }], ['str', { mass: '2' }], ['fric', { friction: 3 }], ['fricneg', { friction: -1 }],
            ['fricstr', { friction: '0.5' }], ['rest', { restitution: 1.5 }], ['static', { static: 'yes' }], ['one', { static: 1 }],
        ];
        const root = group(...bad.map(([n, e], i) => part(n, [1, 1, 1], [i * 3, 0, 0], e as PartExtras)), part('ok', [1, 1, 1], [60, 0, 0], { mass: 1e-6, friction: 2, restitution: 1 }));
        const { r, calls } = run(root, { origin: [0, 0, 0] });
        expect(r.errors.map((e) => e.name)).toEqual(bad.map(([n]) => n));
        expect(r.parts).toEqual(['ok']);
        expect(calls.spawns).toHaveLength(1);
    });

    it('refuses a part beyond the runtime coordinate bound instead of letting the world drop it', () => {
        const root = group(part('far', [1, 1, 1], [2_000_000, 0, 0]), part('near', [1, 1, 1], [0, 0, 0]));
        const { r } = run(root, { origin: [0, 0, 0] });
        expect(r.errors).toEqual([{ name: 'far', error: expect.stringMatching(/1,000,000 units/) }]);
        expect(r.parts).toEqual(['near']);
        expect(run(root).r.errors, 'with the default origin the middle of the model is the origin, so both fit').toEqual([]);
    });

    it('a node scaled to zero on an axis is a row, not an exception, and does not poison the origin', () => {
        const flat = part('flat', [1, 1, 1], [5, 0, 0]);
        flat.scale.set(1, 1, 0);
        const { r } = run(group(flat, part('a', [1, 1, 1], [0, 0, 0]), part('b', [1, 1, 1], [4, 0, 0])));
        expect(r.errors).toEqual([{ name: 'flat', error: expect.stringMatching(/no finite pose/) }]);
        expect(r.parts).toEqual(['a', 'b']);
        expect(r.assembly.origin).toEqual([2, 0, 0]);
    });

    it('finds a partner by the name the file gave it when GLTFLoader renamed the node, and says so when that is ambiguous', async () => {
        const parse = (glb: ArrayBuffer) => new Promise<Object3D>((res, rej) => new GLTFLoader().parse(glb, '', (g) => res(g.scene), rej));
        const glb = buildGlb([
            { name: 'my part.1', ...cube(0.5), extras: { dynova: { static: true } } },
            { name: 'dup', ...cube(0.5), translation: [3, 0, 0], extras: { dynova: { mate: { type: 'ball', with: 'my part.1' } } } },
            { name: 'dup', ...cube(0.5), translation: [6, 0, 0], extras: { dynova: { mate: { type: 'ball', with: 'dup' } } } },
            { name: 'a b', ...cube(0.5), translation: [9, 0, 0], extras: { dynova: { static: true } } },
            { name: 'a b', ...cube(0.5), translation: [12, 0, 0], extras: { dynova: { static: true } } },
            { name: 'user', ...cube(0.5), translation: [15, 0, 0], extras: { dynova: { mate: { type: 'ball', with: 'a b' } } } },
        ]);
        const { world } = stub();
        const r = importAssembly(await parse(glb), world);
        expect(r.parts).toEqual(['my_part1', 'dup', 'dup_1', 'a_b', 'a_b_1', 'user']);
        expect(r.mates, 'a real part name always wins over an original name').toEqual(['dup', 'dup_1']);
        expect(r.errors).toEqual([{ name: 'user', error: expect.stringMatching(/'a b' names 2 parts \(a_b, a_b_1\)/) }]);
    });

    it('a part name wins over an original name, and `with` must be a string', () => {
        const root = new Group();
        const base = part('base', [1, 1, 1], [0, 0, 0], { static: true });
        base.userData.name = 'link';
        const link = part('link', [1, 1, 1], [3, 0, 0], { static: true });
        const m1 = part('m1', [1, 1, 1], [6, 0, 0], { mate: { type: 'ball', with: 'link' } });
        const m2 = part('m2', [1, 1, 1], [9, 0, 0], { mate: { type: 'ball', with: 7 as never } });
        root.add(base, link, m1, m2);
        const { world, calls } = stub();
        const r = importAssembly(root, world, { origin: [0, 0, 0] });
        expect(calls.joints[0].a).toBe(1);
        expect(r.errors).toEqual([{ name: 'm2', error: expect.stringMatching(/must be a part name/) }]);
    });

    it('does not reach past a failed or non-part mesh for the default partner', () => {
        const root = new Group();
        const base = part('base', [1, 1, 1], [0, 0, 0], { static: true });
        const mid = part('mid', [1, 1, 1], [3, 0, 0], { mass: -1 });
        const tip = part('tip', [1, 1, 1], [3, 0, 0], { mate: { type: 'ball' } });
        const empty = new Mesh(new BufferGeometry(), material);
        const tip2 = part('tip2', [1, 1, 1], [3, 0, 0], { mate: { type: 'ball' } });
        base.add(mid);
        mid.add(tip);
        base.add(empty);
        empty.add(tip2);
        root.add(base);
        const { world, calls } = stub();
        const r = importAssembly(root, world, { origin: [0, 0, 0] });
        expect(r.errors.map((e) => e.name)).toEqual(['mid', 'tip', 'tip2']);
        expect(r.errors[1].error).toMatch(/part above it \('mid'\) could not be imported/);
        expect(r.errors[2].error).toMatch(/mesh above it is not a part/);
        expect(calls.joints).toEqual([]);
    });

    it('goes up through plain groups to the nearest part, and an explicit `with` beats the hierarchy', () => {
        const root = new Group();
        const base = part('base', [1, 1, 1], [0, 0, 0], { static: true });
        const other = part('other', [1, 1, 1], [9, 0, 0], { static: true });
        const holder = new Group();
        const child = part('child', [1, 1, 1], [3, 0, 0], { mate: { type: 'ball' } });
        const pinned = part('pinned', [1, 1, 1], [3, 0, 0], { mate: { type: 'ball', with: 'other' } });
        holder.add(child, pinned);
        base.add(holder);
        root.add(base, other);
        const { world, calls } = stub();
        importAssembly(root, world, { origin: [0, 0, 0] });
        // Entity ids follow scene traversal: base 0, child 1, pinned 2, other 3.
        expect(calls.joints.map((j) => [j.a, j.b])).toEqual([[0, 1], [3, 2]]);
    });

    it('a mate whose partner failed, or is declared later, or is named by a failed part, reports instead of crashing', () => {
        const root = new Group();
        root.add(part('early', [1, 1, 1], [0, 0, 0], { mate: { type: 'ball', with: 'late' } }), part('late', [1, 1, 1], [3, 0, 0], { static: true }), part('broken', [1, 1, 1], [6, 0, 0], { mass: -1 }), part('user', [1, 1, 1], [9, 0, 0], { mate: { type: 'ball', with: 'broken' } }));
        const r = importAssembly(root, stub().world, { origin: [0, 0, 0] });
        expect(r.mates).toEqual(['early']);
        expect(r.errors.map((e) => e.name)).toEqual(['broken', 'user']);
        expect(r.errors[1].error).toMatch(/no part named 'broken'/);
    });

    it('accepts origin "centre" written out, and takes the middle of the shape centres, not of the node origins', () => {
        const off = new BufferGeometry();
        off.setAttribute('position', new Float32BufferAttribute(cube(0.5).positions.map((v, i) => (i % 3 === 0 ? v + 10 : v)), 3));
        off.setIndex(cube(0.5).indices);
        const a = new Mesh(off, material);
        a.name = 'offcentre';
        const root = group(a, part('b', [1, 1, 1], [20, 0, 0]));
        const { r, calls } = run(root, { origin: 'centre' });
        expect(calls.spawns[0].x).toBeCloseTo(-5, 4);
        expect(r.assembly.origin[0]).toBeCloseTo(15, 4);
    });

    it('puts a part at the centre of its fitted shape while a default mate point stays at the node origin', () => {
        const off = new BufferGeometry();
        off.setAttribute('position', new Float32BufferAttribute(cube(0.5).positions.map((v, i) => (i % 3 === 0 ? v + 3 : v)), 3));
        off.setIndex(cube(0.5).indices);
        const arm = new Mesh(off, material);
        arm.name = 'arm';
        arm.position.x = 10;
        arm.userData.dynova = { mate: { type: 'ball', with: 'base' } };
        const { calls } = run(group(part('base', [1, 1, 1], [0, 0, 0], { static: true }), arm), { origin: [0, 0, 0] });
        expect(calls.spawns[1].x).toBeCloseTo(13, 4);
        expect(calls.joints[0].anchor[0]).toBeCloseTo(10, 4);
    });

    it('carries slider and distance ranges and rejects a range with a non-number', () => {
        const root = new Group();
        root.add(part('base', [1, 1, 1], [0, 0, 0], { static: true }));
        root.add(part('s', [1, 1, 1], [3, 0, 0], { mate: { type: 'slider', with: 'base', axis: [1, 0, 0], range: [-2, 3] } }));
        root.add(part('d', [1, 1, 1], [6, 0, 0], { mate: { type: 'distance', with: 'base', from: [0, 0, 0], to: [-3, 0, 0], range: [-0.5, 0.5] } }));
        root.add(part('x', [1, 1, 1], [9, 0, 0], { mate: { type: 'slider', with: 'base', axis: [1, 0, 0], range: [-1, NaN] as never } }));
        const { world, calls } = stub();
        const r = importAssembly(root, world, { origin: [0, 0, 0] });
        expect(calls.joints.map((j) => j.limits)).toEqual([[-2, 3], [-0.5, 0.5]]);
        expect(r.errors).toEqual([{ name: 'x', error: expect.stringMatching(/range must be/) }]);
    });

    it('turns the axis with the whole node matrix: mirrored and stretched nodes included', () => {
        const mirrored = part('m', [1, 1, 1], [3, 0, 0], { mate: { type: 'revolute', with: 'base', axis: [1, 1, 0] } });
        mirrored.scale.set(-1, 1, 1);
        const stretched = part('s', [1, 1, 1], [6, 0, 0], { mate: { type: 'revolute', with: 'base', axis: [1, 1, 0] } });
        stretched.scale.set(1, 3, 1);
        const { calls } = run(group(part('base', [1, 1, 1], [0, 0, 0], { static: true }), mirrored, stretched), { origin: [0, 0, 0] });
        const norm = (v: number[] | undefined) => Math.hypot(...(v as number[]));
        expect(calls.joints[0].axis![0]).toBeCloseTo(-Math.SQRT1_2, 5);
        expect(calls.joints[0].axis![1]).toBeCloseTo(Math.SQRT1_2, 5);
        expect(norm(calls.joints[0].axis)).toBeCloseTo(1, 5);
        expect(calls.joints[1].axis![1] / calls.joints[1].axis![0]).toBeCloseTo(3, 5);
    });

    it('reads extras once per part, so a getter that throws on the second read cannot escape', () => {
        const base = part('base', [1, 1, 1], [0, 0, 0], { static: true });
        const flaky = part('flaky', [1, 1, 1], [3, 0, 0]);
        let reads = 0;
        Object.defineProperty(flaky.userData, 'dynova', { get() { if (++reads > 1) throw new Error('second read'); return { mate: { type: 'ball', with: 'base' } }; } });
        const r = importAssembly(group(base, flaky), stub().world, { origin: [0, 0, 0] });
        expect(r.errors).toEqual([]);
        expect(r.mates).toEqual(['flaky']);
    });

    it('rejects a null mate and ignores an empty list of mates', () => {
        const root = new Group();
        root.add(part('base', [1, 1, 1], [0, 0, 0], { static: true }), part('n', [1, 1, 1], [3, 0, 0], { mate: null as never }), part('e', [1, 1, 1], [6, 0, 0], { mate: [] }));
        const r = importAssembly(root, stub().world, { origin: [0, 0, 0] });
        expect(r.errors).toEqual([{ name: 'n', error: expect.stringMatching(/must be an object/) }]);
        expect(r.mates).toEqual([]);
    });
});
