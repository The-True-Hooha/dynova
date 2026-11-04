import { describe, it, expect } from 'vitest';
import { DynamicDrawUsage, BoxGeometry, InstancedMesh, MeshBasicMaterial, Matrix4, Quaternion, Vector3 } from 'three';
import { threeAdapter } from '../threeAdapter.js';

const FLOATS = 7;

function meshOf(count: number) {
    return new InstancedMesh(new BoxGeometry(1, 1, 1), new MeshBasicMaterial(), count);
}

function frame(...entities: number[][]): Float32Array {
    const buf = new Float32Array(entities.length * FLOATS);
    entities.forEach((e, i) => buf.set(e, i * FLOATS));
    return buf;
}

function matrixOf(mesh: InstancedMesh, i: number): Matrix4 {
    const m = new Matrix4();
    mesh.getMatrixAt(i, m);
    return m;
}

const source = (buf: Float32Array) => ({ readTransforms: () => buf });

describe('threeAdapter', () => {
    it('writes position and rotation exactly like THREE.Matrix4.compose', () => {
        const mesh = meshOf(2);
        const q = new Quaternion(0.1, 0.5, -0.3, 0.8).normalize();
        const buf = frame([1, 2, 3, 0, 0, 0, 1], [-4, 5.5, 6, q.x, q.y, q.z, q.w]);
        const before = mesh.instanceMatrix.version;
        expect(threeAdapter(source(buf), mesh).update()).toBe(2);
        const expected = new Matrix4().compose(new Vector3(-4, 5.5, 6), q, new Vector3(1, 1, 1));
        const got = matrixOf(mesh, 1);
        got.elements.forEach((v, k) => expect(v).toBeCloseTo(expected.elements[k], 5));
        expect(matrixOf(mesh, 0).elements[12]).toBe(1);
        expect(mesh.instanceMatrix.version).toBeGreaterThan(before);
    });

    it('tombstoned slots (zero quaternion) become degenerate zero-scale matrices, not identity ghosts', () => {
        const mesh = meshOf(2);
        const buf = frame([9, 9, 9, 0, 0, 0, 0], [1, 1, 1, 0, 0, 0, 1]);
        expect(threeAdapter(source(buf), mesh).update()).toBe(1);
        const dead = matrixOf(mesh, 0).elements;
        expect(dead.slice(0, 12).every((v) => v === 0)).toBe(true);
        expect(dead[15]).toBe(1);
    });

    it('normalises non-unit quaternions and survives a zero-length-but-nonempty one', () => {
        const mesh = meshOf(2);
        const buf = frame([0, 0, 0, 0, 0, 0, 4], [0, 0, 0, 0, 0, 0, 1e-30]);
        threeAdapter(source(buf), mesh).update();
        expect(matrixOf(mesh, 0).elements.slice(0, 3)).toEqual([1, 0, 0].map((v) => v));
        expect(matrixOf(mesh, 1).elements.every(Number.isFinite)).toBe(true);
    });

    it('never writes past mesh.count or past the available transforms', () => {
        const small = meshOf(2);
        const buf = frame([1, 0, 0, 0, 0, 0, 1], [2, 0, 0, 0, 0, 0, 1], [3, 0, 0, 0, 0, 0, 1]);
        const sentinel = small.instanceMatrix.array as Float32Array;
        sentinel.fill(7, 16);
        small.count = 1;
        threeAdapter(source(buf), small).update();
        expect(Array.from(sentinel.slice(16))).toEqual(new Array(16).fill(7));
        const big = meshOf(5);
        const live = threeAdapter(source(frame([1, 0, 0, 0, 0, 0, 1])), big).update();
        expect(live).toBe(1);
        expect(matrixOf(big, 3).elements[12]).toBe(0);
    });

    it('idOffset maps entity ids onto a later block of instances', () => {
        const mesh = meshOf(2);
        const buf = frame([1, 0, 0, 0, 0, 0, 1], [2, 0, 0, 0, 0, 0, 1], [3, 0, 0, 0, 0, 0, 1]);
        threeAdapter(source(buf), mesh, { idOffset: 1 }).update();
        expect(matrixOf(mesh, 0).elements[12]).toBe(2);
        expect(matrixOf(mesh, 1).elements[12]).toBe(3);
    });

    it('follows capacity growth: a larger buffer on the next update is picked up', () => {
        const mesh = meshOf(4);
        let buf = frame([1, 0, 0, 0, 0, 0, 1], [2, 0, 0, 0, 0, 0, 1]);
        const adapter = threeAdapter({ readTransforms: () => buf }, mesh);
        expect(adapter.update()).toBe(2);
        buf = frame([1, 0, 0, 0, 0, 0, 1], [2, 0, 0, 0, 0, 0, 1], [3, 0, 0, 0, 0, 0, 1], [4, 0, 0, 0, 0, 0, 1]);
        expect(adapter.update()).toBe(4);
        expect(matrixOf(mesh, 3).elements[12]).toBe(4);
    });

    it('is idempotent for an unchanged frame and an empty buffer is harmless', () => {
        const mesh = meshOf(2);
        const buf = frame([1, 2, 3, 0, 0, 0, 1]);
        const adapter = threeAdapter(source(buf), mesh);
        adapter.update();
        const first = Array.from(mesh.instanceMatrix.array);
        adapter.update();
        expect(Array.from(mesh.instanceMatrix.array)).toEqual(first);
        expect(threeAdapter(source(new Float32Array(0)), mesh).update()).toBe(0);
    });

    it('never creates scene objects: the mesh has no children after updates', () => {
        const mesh = meshOf(3);
        threeAdapter(source(frame([1, 0, 0, 0, 0, 0, 1])), mesh).update();
        expect(mesh.children).toHaveLength(0);
        expect(mesh.frustumCulled).toBe(false);
    });

    it('a scaled quaternion (0,0,2,2) renders as a clean 90 degree turn about Z', () => {
        const mesh = meshOf(1);
        threeAdapter(source(frame([0, 0, 0, 0, 0, 2, 2])), mesh).update();
        const e = matrixOf(mesh, 0).elements;
        expect(e[0]).toBeCloseTo(0, 5);
        expect(e[1]).toBeCloseTo(1, 5);
        expect(e[5]).toBeCloseTo(0, 5);
    });

    it.each([[-1], [0.5], [NaN], [Infinity]])('rejects idOffset %s up front', (bad) => {
        expect(() => threeAdapter(source(frame([1, 0, 0, 0, 0, 0, 1])), meshOf(1), { idOffset: bad })).toThrow(RangeError);
    });

    it.each([
        ['NaN position', [NaN, 0, 0, 0, 0, 0, 1]],
        ['Infinity position', [0, Infinity, 0, 0, 0, 0, 1]],
        ['NaN quaternion', [0, 0, 0, NaN, 0, 0, 1]],
        ['Infinity quaternion', [0, 0, 0, 0, 0, 0, Infinity]],
    ])('hides an entity with %s instead of poisoning the instance buffer', (_n, e) => {
        const mesh = meshOf(2);
        const live = threeAdapter(source(frame(e as number[], [1, 0, 0, 0, 0, 0, 1])), mesh).update();
        expect(live).toBe(1);
        expect((mesh.instanceMatrix.array as Float32Array).every(Number.isFinite)).toBe(true);
        expect(matrixOf(mesh, 0).elements.slice(0, 12).every((v) => v === 0)).toBe(true);
    });

    it('clears instances beyond the available entities when the buffer shrinks', () => {
        const mesh = meshOf(3);
        const adapter = threeAdapter({ readTransforms: () => buf }, mesh);
        let buf = frame([1, 0, 0, 0, 0, 0, 1], [2, 0, 0, 0, 0, 0, 1], [3, 0, 0, 0, 0, 0, 1]);
        adapter.update();
        buf = frame([1, 0, 0, 0, 0, 0, 1]);
        expect(adapter.update()).toBe(1);
        expect(matrixOf(mesh, 1).elements.slice(0, 12).every((v) => v === 0)).toBe(true);
        expect(matrixOf(mesh, 2).elements[12]).toBe(0);
    });

    it('uses dynamic draw usage for per-frame uploads', () => {
        const mesh = meshOf(1);
        threeAdapter(source(frame([1, 0, 0, 0, 0, 0, 1])), mesh);
        expect(mesh.instanceMatrix.usage).toBe(DynamicDrawUsage);
    });

    it('respects a reduced mesh.count even when the instance array is larger', () => {
        const mesh = meshOf(3);
        mesh.count = 1;
        threeAdapter(source(frame([1, 0, 0, 0, 0, 0, 1], [2, 0, 0, 0, 0, 0, 1], [3, 0, 0, 0, 0, 0, 1])), mesh).update();
        expect(matrixOf(mesh, 0).elements[12]).toBe(1);
        expect(matrixOf(mesh, 1).elements[12]).toBe(0);
    });
});
