import { afterAll, beforeAll, describe, expect, it } from 'vitest';
import { createServer, type Server } from 'node:http';
import { readFileSync } from 'node:fs';
import type { AddressInfo } from 'node:net';
import { Mesh, MeshStandardMaterial } from 'three';
import { loadGltfScene, loadModelFiles, loadModelUrl, type ModelFile } from '../loaders.js';
import { collidersFromScene } from '../colliders.js';
import { box, buildGlb, cube, rampGrid } from '../testing/gltfFixture.js';

const fixture = (name: string): ArrayBuffer => {
    const b = readFileSync(new URL(`../testing/fixtures/${name}`, import.meta.url));
    return b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength) as ArrayBuffer;
};
const file = (name: string, data: ArrayBuffer): ModelFile => ({ name, data });
const text = (s: string): ArrayBuffer => new TextEncoder().encode(s).buffer as ArrayBuffer;
const kinds = (scene: Parameters<typeof collidersFromScene>[0]) => collidersFromScene(scene).colliders.map((c) => `${c.name}:${c.shape.kind}`);
const LEVEL = ['floor:box', 'wall-n:box', 'ramp:box', 'pillar:hull'];

describe('glTF (glb) loading', () => {
    it('parses GLB bytes into a scene whose meshes become colliders', async () => {
        const glb = buildGlb([
            { name: 'floor', ...box(5, 0.25, 5), translation: [0, -0.25, 0], color: [0.2, 0.6, 0.2] },
            { name: 'ramp', ...rampGrid(4, 0.5), translation: [-2, 0, -2] },
            { name: 'crate', ...cube(0.5), translation: [1, 0.5, 1] },
        ]);
        const r = await loadGltfScene(glb);
        expect(r.ok && r.format).toBe('glb');
        if (!r.ok) return;
        const { colliders, skipped } = collidersFromScene(r.scene);
        expect(skipped).toEqual([]);
        expect(colliders.map((c) => `${c.name}:${c.shape.kind}`)).toEqual(['floor:box', 'ramp:box', 'crate:box']);
    });

    it('keeps material colours from the file', async () => {
        const r = await loadGltfScene(buildGlb([{ name: 'a', ...cube(), color: [1, 0, 0] }]));
        if (!r.ok) throw new Error(r.error);
        let rgb: number[] = [];
        r.scene.traverse((n) => { if ((n as Mesh).isMesh) { const c = ((n as Mesh).material as MeshStandardMaterial).color; rgb = [c.r, c.g, c.b]; } });
        expect(rgb[0]).toBeCloseTo(1, 5);
        expect(rgb[1]).toBeCloseTo(0, 5);
        expect(rgb[2]).toBeCloseTo(0, 5);
    });

    it('returns an error for an empty buffer, random bytes and truncated GLB instead of throwing', async () => {
        expect(await loadGltfScene(new ArrayBuffer(0))).toEqual({ ok: false, error: 'empty file' });
        const noise = new Uint8Array(256).map((_, i) => (i * 37) & 255);
        const bad = await loadGltfScene(noise.buffer);
        expect(bad.ok).toBe(false);
        if (!bad.ok) expect(bad.error.length).toBeGreaterThan(0);
        expect((await loadGltfScene(buildGlb([{ ...cube() }]).slice(0, 40))).ok).toBe(false);
    });

    it('a glTF with no meshes loads as an empty scene with no colliders', async () => {
        const r = await loadGltfScene(buildGlb([]));
        if (!r.ok) throw new Error(r.error);
        expect(collidersFromScene(r.scene)).toEqual({ colliders: [], skipped: [] });
    });

    it('says a text or HTML file is not a glTF instead of showing a JSON parser message', async () => {
        const r = await loadGltfScene(text('<!doctype html><html><body>404</body></html>'));
        expect(r.ok).toBe(false);
        if (!r.ok) expect(r.error).toContain('not a valid glTF or glb file');
    });
});

describe('compressed glTF', () => {
    it('decodes Meshopt-compressed geometry and the colliders keep the right size', async () => {
        const r = await loadGltfScene(fixture('level-meshopt.glb'));
        if (!r.ok) throw new Error(r.error);
        expect(kinds(r.scene)).toEqual(LEVEL);
        const floor = collidersFromScene(r.scene).colliders[0].shape;
        if (floor.kind !== 'box') throw new Error('expected box');
        expect(floor.half[0]).toBeCloseTo(6, 1);
        expect(floor.half[1]).toBeCloseTo(0.25, 1);
    });

    it('a Draco file without a decoder path gives an actionable error', async () => {
        const r = await loadGltfScene(fixture('level-draco.glb'));
        expect(r.ok).toBe(false);
        if (!r.ok) expect(r.error).toContain('options.dracoDecoderPath');
    });

    it('optional KTX2 textures are skipped so the geometry still loads', async () => {
        const r = await loadGltfScene(fixture('level-ktx2.glb'));
        if (!r.ok) throw new Error(r.error);
        expect(kinds(r.scene)).toEqual(LEVEL);
    });

    it('required KTX2 textures fail with an actionable error when no ktx2Loader is given', async () => {
        const r = await loadGltfScene(fixture('level-ktx2-required.glb'));
        expect(r).toEqual({ ok: false, error: 'this file requires KTX2 textures: pass options.ktx2Loader' });
    });

    it('PNG textures in a glb still load as real images and are not replaced by the KTX2 stub', async () => {
        const g = globalThis as { createImageBitmap?: unknown };
        const had = g.createImageBitmap;
        (g as { self?: unknown }).self ??= globalThis;
        g.createImageBitmap = async () => ({ width: 1, height: 1, close() {} });
        let r;
        try {
            r = await loadGltfScene(fixture('level-png.glb'));
        } finally {
            g.createImageBitmap = had;
        }
        if (!r.ok) throw new Error(r.error);
        let hasMap = false;
        r.scene.traverse((n) => { const m = (n as Mesh).material as MeshStandardMaterial | undefined; if (m?.map) hasMap = m.map.image !== undefined && m.map.image !== null; });
        expect(hasMap).toBe(true);
    });

    it('a Draco decoder path without a trailing slash still works', async () => {
        const r = await loadGltfScene(fixture('level-meshopt.glb'), { dracoDecoderPath: '/draco' });
        expect(r.ok).toBe(true);
    });

    it('a Draco decoder path on a file that does not need it is harmless', async () => {
        const r = await loadGltfScene(fixture('level-meshopt.glb'), { dracoDecoderPath: '/draco/' });
        expect(r.ok).toBe(true);
    });
});

describe('multi-file .gltf', () => {
    it('resolves the .bin from the other provided files', async () => {
        const r = await loadModelFiles([file('level-split.gltf', fixture('level-split.gltf')), file('level-split.bin', fixture('level-split.bin'))]);
        if (!r.ok) throw new Error(r.error);
        expect(r.format).toBe('gltf');
        expect(kinds(r.scene)).toEqual(LEVEL);
    });

    it('file order and folder prefixes do not matter; unrelated files are ignored', async () => {
        const r = await loadModelFiles([
            file('notes.txt', text('hello')),
            file('assets/level-split.bin', fixture('level-split.bin')),
            file('assets/level-split.gltf', fixture('level-split.gltf')),
        ]);
        expect(r.ok).toBe(true);
    });

    it('revokes every blob URL it created, on success and on failure', async () => {
        const created: string[] = [];
        const revoked: string[] = [];
        const make = URL.createObjectURL.bind(URL);
        const revoke = URL.revokeObjectURL.bind(URL);
        URL.createObjectURL = (b: Blob | MediaSource) => { const u = make(b); created.push(u); return u; };
        URL.revokeObjectURL = (u: string) => { revoked.push(u); revoke(u); };
        try {
            await loadModelFiles([file('level-split.gltf', fixture('level-split.gltf')), file('level-split.bin', fixture('level-split.bin')), file('extra.bin', text('x'))]);
            await loadModelFiles([file('bad.gltf', text('{ "buffers": [{ "uri": "x.bin" }] }')), file('x.bin', text('x'))]);
        } finally {
            URL.createObjectURL = make;
            URL.revokeObjectURL = revoke;
        }
        expect(created.length).toBeGreaterThanOrEqual(1);
        expect(revoked.sort()).toEqual(created.sort());
    });

    it('binds each reference to the file with the matching folder path and reports ambiguous names', async () => {
        const triangle = (size: number) => { const a = new Float32Array([0, 0, 0, size, 0, 0, 0, size, 0]); return a.buffer.slice(0) as ArrayBuffer; };
        const gltf = (uris: string[]) => text(JSON.stringify({
            asset: { version: '2.0' },
            scene: 0,
            scenes: [{ nodes: uris.map((_, i) => i) }],
            nodes: uris.map((_, i) => ({ mesh: i, name: `n${i}` })),
            meshes: uris.map((_, i) => ({ primitives: [{ attributes: { POSITION: i } }] })),
            accessors: uris.map((_, i) => ({ bufferView: i, componentType: 5126, count: 3, type: 'VEC3', min: [0, 0, 0], max: [1, 1, 0] })),
            bufferViews: uris.map((_, i) => ({ buffer: i, byteLength: 36 })),
            buffers: uris.map((uri) => ({ uri, byteLength: 36 })),
        }));
        const ambiguous = await loadModelFiles([file('m.gltf', gltf(['x.bin'])), file('a/x.bin', triangle(1)), file('b/x.bin', triangle(2))]);
        expect(ambiguous.ok).toBe(false);
        if (!ambiguous.ok) expect(ambiguous.error).toContain('share that name');

        const r = await loadModelFiles([file('m.gltf', gltf(['a/x.bin', 'b/x.bin'])), file('a/x.bin', triangle(1)), file('b/x.bin', triangle(5))]);
        if (!r.ok) throw new Error(r.error);
        const widths = r.scene.children.map((n) => { const m = n as Mesh; m.geometry.computeBoundingBox(); return m.geometry.boundingBox!.max.x; });
        expect(widths).toEqual([1, 5]);
    });

    it('a literal % in a file name does not break resolution', async () => {
        const json = text(JSON.stringify({ asset: { version: '2.0' }, buffers: [{ uri: '100%.bin', byteLength: 1 }] }));
        const r = await loadModelFiles([file('m.gltf', json), file('100%.bin', text('x'))]);
        expect(r.ok).toBe(true);
    });

    it('lists every referenced file that was not provided', async () => {
        const r = await loadModelFiles([file('level-split.gltf', fixture('level-split.gltf'))]);
        expect(r).toEqual({ ok: false, error: 'the .gltf references files that were not provided: level-split.bin' });
    });

    it('a self-contained .gltf with embedded data needs no other files', async () => {
        const json = JSON.stringify({ asset: { version: '2.0' }, scenes: [{ nodes: [] }], scene: 0 });
        const r = await loadModelFiles([file('empty.gltf', text(json))]);
        expect(r.ok && r.format).toBe('gltf');
    });

    it('invalid .gltf JSON is reported as not a valid glTF', async () => {
        const r = await loadModelFiles([file('bad.gltf', text('{ not json'))]);
        expect(r.ok).toBe(false);
        if (!r.ok) expect(r.error).toContain('not a valid glTF or glb file');
    });
});

const PYRAMID_OBJ = `
o pyramid
v 0 0 0
v 2 0 0
v 2 0 2
v 0 0 2
v 1 3 1
f 1 3 2
f 1 4 3
f 1 2 5
f 2 3 5
f 3 4 5
f 4 1 5
`;

const PYRAMID_STL = `solid pyramid
facet normal 0 0 0
 outer loop
  vertex 0 0 0
  vertex 2 0 2
  vertex 2 0 0
 endloop
endfacet
facet normal 0 0 0
 outer loop
  vertex 0 0 0
  vertex 0 0 2
  vertex 2 0 2
 endloop
endfacet
facet normal 0 0 0
 outer loop
  vertex 0 0 0
  vertex 2 0 0
  vertex 1 3 1
 endloop
endfacet
facet normal 0 0 0
 outer loop
  vertex 2 0 0
  vertex 2 0 2
  vertex 1 3 1
 endloop
endfacet
facet normal 0 0 0
 outer loop
  vertex 2 0 2
  vertex 0 0 2
  vertex 1 3 1
 endloop
endfacet
facet normal 0 0 0
 outer loop
  vertex 0 0 2
  vertex 0 0 0
  vertex 1 3 1
 endloop
endfacet
endsolid pyramid
`;

function binaryStlOfCube(): ArrayBuffer {
    const tris: number[][] = [];
    const c = [[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1], [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]];
    const faces = [[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4], [2, 3, 7], [2, 7, 6], [0, 4, 7], [0, 7, 3], [1, 2, 6], [1, 6, 5]];
    for (const f of faces) tris.push(f.flatMap((i) => c[i]));
    const buf = new ArrayBuffer(84 + tris.length * 50);
    const dv = new DataView(buf);
    dv.setUint32(80, tris.length, true);
    tris.forEach((t, i) => t.forEach((v, k) => dv.setFloat32(84 + i * 50 + 12 + k * 4, v, true)));
    return buf;
}

const PYRAMID_PLY = `ply
format ascii 1.0
element vertex 5
property float x
property float y
property float z
element face 6
property list uchar int vertex_indices
end_header
0 0 0
2 0 0
2 0 2
0 0 2
1 3 1
3 0 2 1
3 0 3 2
3 0 1 4
3 1 2 4
3 2 3 4
3 3 0 4
`;

describe('OBJ, STL and PLY', () => {
    it.each([
        ['model.obj', text(PYRAMID_OBJ), 'obj'],
        ['model.stl', text(PYRAMID_STL), 'stl'],
        ['model.ply', text(PYRAMID_PLY), 'ply'],
    ] as const)('%s loads and becomes a hull collider', async (name, data, format) => {
        const r = await loadModelFiles([file(name, data)]);
        if (!r.ok) throw new Error(r.error);
        expect(r.format).toBe(format);
        const { colliders, skipped } = collidersFromScene(r.scene);
        expect(skipped).toEqual([]);
        expect(colliders).toHaveLength(1);
        expect(colliders[0].shape.kind).toBe('hull');
    });

    it('binary STL is detected and a cube becomes a box collider of the right size', async () => {
        const r = await loadModelFiles([file('cube.stl', binaryStlOfCube())]);
        if (!r.ok) throw new Error(r.error);
        const shape = collidersFromScene(r.scene).colliders[0].shape;
        expect(shape.kind).toBe('box');
        if (shape.kind === 'box') shape.half.forEach((h) => expect(h).toBeCloseTo(1, 5));
    });

    it('an OBJ with several objects keeps one mesh per object', async () => {
        const two = `${PYRAMID_OBJ}\no second\nv 5 0 0\nv 6 0 0\nv 6 1 0\nv 5 0 1\nf 6 7 8\nf 6 8 9\nf 6 9 7\nf 7 9 8\n`;
        const r = await loadModelFiles([file('two.obj', text(two))]);
        if (!r.ok) throw new Error(r.error);
        expect(collidersFromScene(r.scene).colliders.length).toBe(2);
    });

    it('an OBJ with no geometry loads as an empty scene for the caller to reject', async () => {
        const r = await loadModelFiles([file('nothing.obj', text('# nothing here\n'))]);
        if (!r.ok) throw new Error(r.error);
        expect(collidersFromScene(r.scene).colliders).toEqual([]);
    });

    it('a truncated binary PLY or a garbage STL is an error result, not an exception', async () => {
        const ply = await loadModelFiles([file('bad.ply', text('ply\nformat binary_little_endian 1.0\nelement vertex 3\nproperty float x\nend_header\n12'))]);
        expect(ply).toEqual({ ok: false, error: 'the PLY file is truncated or corrupt' });
        const stl = await loadModelFiles([file('bad.stl', text('hello world this is not an stl file at all, definitely not.'))]);
        expect(stl).toEqual({ ok: false, error: 'the STL file is truncated or corrupt' });
    });
});

describe('loadModelFiles selection', () => {
    it('prefers glb over gltf over obj over stl over ply, regardless of order', async () => {
        const glb = file('a.glb', buildGlb([{ name: 'from-glb', ...cube() }]));
        const r = await loadModelFiles([file('b.ply', text(PYRAMID_PLY)), file('c.stl', text(PYRAMID_STL)), file('d.obj', text(PYRAMID_OBJ)), glb]);
        expect(r.ok && r.format).toBe('glb');
        const o = await loadModelFiles([file('b.ply', text(PYRAMID_PLY)), file('c.stl', text(PYRAMID_STL)), file('d.obj', text(PYRAMID_OBJ))]);
        expect(o.ok && o.format).toBe('obj');
        const s = await loadModelFiles([file('b.ply', text(PYRAMID_PLY)), file('c.stl', text(PYRAMID_STL))]);
        expect(s.ok && s.format).toBe('stl');
    });

    it('extensions are case-insensitive', async () => {
        const r = await loadModelFiles([file('MODEL.GLB', buildGlb([{ name: 'a', ...cube() }]))]);
        expect(r.ok && r.format).toBe('glb');
    });

    it('reports no files, an unsupported set, and an empty primary file', async () => {
        expect(await loadModelFiles([])).toEqual({ ok: false, error: 'no files' });
        const none = await loadModelFiles([file('a.png', new ArrayBuffer(4)), file('b.mtl', new ArrayBuffer(4))]);
        expect(none.ok).toBe(false);
        if (!none.ok) expect(none.error).toContain('a.png, b.mtl');
        expect(await loadModelFiles([file('empty.glb', new ArrayBuffer(0))])).toEqual({ ok: false, error: 'empty file' });
    });
});

describe('loadModelUrl', () => {
    let server: Server;
    let base = '';
    const routes: Record<string, { type: string; body: Uint8Array }> = {};
    beforeAll(async () => {
        routes['/level.glb'] = { type: 'model/gltf-binary', body: new Uint8Array(fixture('level-meshopt.glb')) };
        routes['/pyramid.obj'] = { type: 'text/plain', body: new TextEncoder().encode(PYRAMID_OBJ) };
        routes['/level-split.gltf'] = { type: 'model/gltf+json', body: new Uint8Array(fixture('level-split.gltf')) };
        routes['/level-split.bin'] = { type: 'application/octet-stream', body: new Uint8Array(fixture('level-split.bin')) };
        server = createServer((req, res) => {
            const route = routes[(req.url ?? '').split('?')[0]];
            if (!route) { res.statusCode = 404; res.end('nope'); return; }
            res.setHeader('content-type', route.type);
            res.end(route.body);
        });
        await new Promise<void>((resolve) => server.listen(0, '127.0.0.1', resolve));
        base = `http://127.0.0.1:${(server.address() as AddressInfo).port}`;
    });
    afterAll(() => new Promise<void>((resolve) => server.close(() => resolve())));

    it('loads a glb, an obj and a .gltf with its sibling .bin over HTTP', async () => {
        for (const [path, expected] of [['/level.glb', LEVEL], ['/level-split.gltf', LEVEL]] as const) {
            const r = await loadModelUrl(base + path);
            if (!r.ok) throw new Error(`${path}: ${r.error}`);
            expect(kinds(r.scene)).toEqual(expected);
        }
        const obj = await loadModelUrl(`${base}/pyramid.obj?cache=1`);
        expect(obj.ok && obj.format).toBe('obj');
    });

    it('reports an HTTP error for a missing obj, rejects unknown extensions and an unreachable host', async () => {
        const missing = await loadModelUrl(`${base}/missing.obj`);
        expect(missing).toEqual({ ok: false, error: `could not load ${base}/missing.obj: HTTP 404` });
        const unknown = await loadModelUrl(`${base}/thing.fbx`);
        expect(unknown.ok).toBe(false);
        if (!unknown.ok) expect(unknown.error).toContain('unsupported file type');
        expect((await loadModelUrl('http://127.0.0.1:1/x.glb')).ok).toBe(false);
        expect((await loadModelUrl('http://127.0.0.1:1/x.obj')).ok).toBe(false);
    });

    it('loadGltfScene(url) is the same as loadModelUrl for glTF', async () => {
        const r = await loadGltfScene(`${base}/level.glb`);
        expect(r.ok).toBe(true);
    });
});
