import { describe, it, expect } from 'vitest';
import { Mesh, MeshStandardMaterial } from 'three';
import { loadGltfScene } from '../gltf.js';
import { collidersFromScene } from '../colliders.js';
import { box, buildGlb, cube, rampGrid } from '../testing/gltfFixture.js';

describe('loadGltfScene', () => {
    it('parses GLB bytes into a scene whose meshes become colliders', async () => {
        const glb = buildGlb([
            { name: 'floor', ...box(5, 0.25, 5), translation: [0, -0.25, 0], color: [0.2, 0.6, 0.2] },
            { name: 'ramp', ...rampGrid(4, 0.5), translation: [-2, 0, -2] },
            { name: 'crate', ...cube(0.5), translation: [1, 0.5, 1] },
        ]);
        const r = await loadGltfScene(glb);
        expect(r.ok).toBe(true);
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
        const glb = buildGlb([{ ...cube() }]);
        expect((await loadGltfScene(glb.slice(0, 40))).ok).toBe(false);
    });

    it('a glTF with no meshes loads as an empty scene with no colliders', async () => {
        const r = await loadGltfScene(buildGlb([]));
        if (!r.ok) throw new Error(r.error);
        expect(collidersFromScene(r.scene)).toEqual({ colliders: [], skipped: [] });
    });

    it('rejects a missing URL with a readable error', async () => {
        const r = await loadGltfScene('http://127.0.0.1:1/missing.glb');
        expect(r.ok).toBe(false);
        if (!r.ok) expect(r.error.length).toBeGreaterThan(0);
    });

    it('says a text or HTML file is not a glTF instead of showing a JSON parser message', async () => {
        const html = new TextEncoder().encode('<!doctype html><html><body>404</body></html>');
        const r = await loadGltfScene(html.buffer);
        expect(r.ok).toBe(false);
        if (!r.ok) expect(r.error).toContain('not a valid glTF or glb file');
    });
});
