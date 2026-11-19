import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { WorldBatch, type BatchRuntime } from '../batch.js';
import { lookAt, lensWords, poseWords, idColors, type CameraPose } from '../camera.js';
import { NO_ENTITY, SHAPE_SPHERE } from '../CommandRing.js';

// The real single-threaded wasm module (make build-st): the images are computed by the shipped core, not a stand-in.
const glue = await import('../../wasm-st/dynova_core.js');
const wasm = glue.initSync({ module: readFileSync(new URL('../../wasm-st/dynova_core_bg.wasm', import.meta.url)) });
const config = { capacity: 16, ringCapacity: 64 };
const FACING: CameraPose = { position: [0, 0, 0], rotation: [0, 0, 0, 1] };

function batchOf(worlds: number): { raw: InstanceType<typeof glue.Batch>; set: WorldBatch } {
    const raw = new glue.Batch(worlds, config.capacity, config.ringCapacity);
    raw.enable_physics();
    return { raw, set: new WorldBatch(config, wasm.memory, 'fallback', raw as unknown as BatchRuntime) };
}

/** FNV-1a over each 32-bit word widened to 64 bits, as the Rust test hashes an image. */
function hashImage(depth: Float32Array, id: Uint32Array): bigint {
    let h = 0xcbf29ce484222325n;
    const feed = (word: number) => {
        for (let k = 0; k < 8; k++) {
            h ^= BigInt(k < 4 ? (word >>> (8 * k)) & 0xff : 0);
            h = BigInt.asUintN(64, h * 0x100000001b3n);
        }
    };
    new Uint32Array(depth.buffer, depth.byteOffset, depth.length).forEach(feed);
    id.forEach(feed);
    return h;
}

describe('camera sensors on real wasm worlds', () => {
    it('a flat wall reads its distance at every pixel of every world, and ids are the spawned entity ids', () => {
        const { set } = batchOf(3);
        const walls = set.worlds.map((w, i) => w.spawn({ x: 0, y: 0, z: -(i + 3.5), hx: 50, hy: 50, hz: 0.5, invMass: 0 }));
        set.step();
        const img = set.render({ width: 32, height: 24, fovY: Math.PI / 3 }, set.worlds.map(() => FACING));
        expect(img.depth.length).toBe(3 * 32 * 24);
        for (let i = 0; i < 3; i++) {
            const { depth, id } = img.world(i);
            expect(Math.max(...depth.map((z) => Math.abs(z - (i + 3)))), `world ${i}`).toBeLessThan(1e-3);
            expect(new Set(id)).toEqual(new Set([walls[i].id]));
        }
    });

    it('a sphere reads the analytic distance within 1 mm and the pixels around it read the wall behind', () => {
        const { set } = batchOf(1);
        const w = set.worlds[0];
        w.spawn({ x: 0, y: 0, z: -20, hx: 50, hy: 50, hz: 0.5, invMass: 0 });
        const ball = w.spawn({ x: 0, y: 0, z: -6, shape: SHAPE_SPHERE, hx: 1, invMass: 0 });
        set.step();
        const camera = { width: 33, height: 33, fovY: Math.PI / 3 };
        const { depth, id } = set.render(camera, [FACING]).world(0);
        const centre = 16 * 33 + 16;
        // The centre pixel's ray is within 0.03 degrees of the axis: the sphere's near surface is 5 m along it.
        expect(depth[centre]).toBeGreaterThan(4.999);
        expect(depth[centre]).toBeLessThan(5.0001);
        expect(id[centre]).toBe(ball.id);
        expect(depth[0]).toBeCloseTo(19.5, 3);
        expect(id[0]).not.toBe(ball.id);
    });

    it('matches the pinned image of the same scene the Rust core renders', () => {
        const { raw, set } = batchOf(1);
        const w = set.worlds[0];
        w.spawn({ x: 0.3, y: 0.2, z: -5, shape: SHAPE_SPHERE, hx: 1, invMass: 0 });
        w.spawn({ x: -2, y: -1, z: -7, hx: 1, hy: 0.5, hz: 2, invMass: 0 });
        w.spawn({ x: 0, y: -3, z: -5, hx: 40, hy: 0.5, hz: 40, invMass: 0 });
        set.step();
        raw.render_all(48, 48, new Float32Array([40, 40, 24, 24, 0, 100]), new Float32Array([0.25, 0.5, 0, 0.0625, 0.125, 0, 1]), NO_ENTITY);
        const n = raw.image_pixels();
        const depth = new Float32Array(wasm.memory.buffer, raw.depth_ptr(), n);
        const id = new Uint32Array(wasm.memory.buffer, raw.id_ptr(), n);
        expect(hashImage(depth, id)).toBe(10825879982174112534n);
    });

    it('refuses a bad camera, the wrong number of poses and a failed world, naming the problem', () => {
        const { set } = batchOf(2);
        const cam = { width: 8, height: 8, fovY: 1 };
        expect(() => set.render(cam, [FACING])).toThrow(/one pose per world: 2 worlds, 1 poses/);
        expect(() => set.render({ ...cam, width: 0 }, [FACING, FACING])).toThrow(/image size/);
        expect(() => set.render({ ...cam, fovY: 0 }, [FACING, FACING])).toThrow(/fovY/);
        expect(() => set.render({ ...cam, near: 2, far: 1 }, [FACING, FACING])).toThrow(/near < far/);
        expect(() => set.render(cam, [FACING, { position: [Number.NaN, 0, 0], rotation: [0, 0, 0, 1] }])).toThrow(/pose 1 needs a finite/);
        expect(() => set.render({ width: 4096, height: 4096, fovY: 1 }, [FACING, FACING])).toThrow(/pixels in total/);
        for (const ignore of [-1, 1.5, Number.NaN, 2 ** 32]) expect(() => set.render({ ...cam, ignore }, [FACING, FACING])).toThrow(/ignore must be a body id/);
        set.worlds[1].markFatal('gone');
        expect(() => set.render(cam, [FACING, FACING])).toThrow(/world unavailable: gone/);
    });

    it('a trap in render fails every world; a refusal does not', () => {
        const real = new glue.Batch(2, config.capacity, config.ringCapacity);
        real.enable_physics();
        let error: Error = new Error('declined');
        const wrapped = Object.assign(Object.create(real), { render_all: () => { throw error; } }) as BatchRuntime;
        const set = new WorldBatch(config, wasm.memory, 'fallback', wrapped);
        expect(() => set.render({ width: 4, height: 4, fovY: 1 }, [FACING, FACING])).toThrow('declined');
        expect(set.worlds.map((w) => w.failure)).toEqual([null, null]);
        error = new WebAssembly.RuntimeError('unreachable');
        expect(() => set.render({ width: 4, height: 4, fovY: 1 }, [FACING, FACING])).toThrow('unreachable');
        expect(set.worlds.map((w) => w.failure)).toEqual([expect.stringMatching(/trapped/), expect.stringMatching(/trapped/)]);
    });

    it('world(i) refuses an index outside the batch', () => {
        const { set } = batchOf(2);
        set.step();
        const img = set.render({ width: 4, height: 4, fovY: 1 }, [FACING, FACING]);
        expect(() => img.world(2)).toThrow(RangeError);
        expect(() => img.world(-1)).toThrow(RangeError);
        expect(() => img.world(0.5)).toThrow(RangeError);
    });

    it('a camera mounted on a body does not see that body when it is ignored', () => {
        const { set } = batchOf(1);
        const w = set.worlds[0];
        const holder = w.spawn({ x: 0, y: 0, z: 0, hx: 1, invMass: 0 });
        w.spawn({ x: 0, y: 0, z: -5, hx: 50, hy: 50, hz: 0.5, invMass: 0 });
        set.step();
        const cam = { width: 4, height: 4, fovY: 1 };
        expect(new Set(set.render(cam, [FACING]).world(0).id)).toEqual(new Set([holder.id]));
        const seen = set.render({ ...cam, ignore: holder.id }, [FACING]).world(0);
        expect(seen.id.every((i) => i !== holder.id) && seen.depth.every((z) => Math.abs(z - 4.5) < 1e-3)).toBe(true);
    });
});

describe('camera helpers', () => {
    it('lookAt points the camera down -z at the target, with +y up the image', () => {
        const eye = [1, 2, 3] as const;
        const pose = lookAt(eye, [1, 2, -7]);
        expect(pose.position).toEqual(eye);
        expect(pose.rotation.map((v) => Math.abs(v))).toEqual([0, 0, 0, 1].map((v) => expect.closeTo(v, 6)));
        const turned = lookAt([0, 0, 0], [5, 0, 0]);
        const [x, y, z, w] = turned.rotation;
        // Rotating the local -z axis by the quaternion must give +x.
        const v = [0, 0, -1];
        const t = [2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])];
        const dir = [v[0] + w * t[0] + (y * t[2] - z * t[1]), v[1] + w * t[1] + (z * t[0] - x * t[2]), v[2] + w * t[2] + (x * t[1] - y * t[0])];
        expect(dir[0]).toBeCloseTo(1, 6);
        expect(dir[1]).toBeCloseTo(0, 6);
        expect(dir[2]).toBeCloseTo(0, 6);
    });

    it('lookAt looking straight down, with its default up, is refused; a given up fixes it', () => {
        expect(() => lookAt([0, 5, 0], [0, 0, 0])).toThrow(/parallel to up/);
        expect(() => lookAt([0, 5, 0], [0, 0, 0], [0, 0, -1])).not.toThrow();
        expect(() => lookAt([1, 1, 1], [1, 1, 1])).toThrow(/apart/);
    });

    it('lookAt handles every branch of the matrix to quaternion conversion with a unit result', () => {
        for (const [target, up] of [[[0, 0, -1], [0, 1, 0]], [[1, 0, 0], [0, 1, 0]], [[0, -1, 0], [0, 0, -1]], [[0, 0, 1], [0, 1, 0]], [[-1, 0, 0], [0, 1, 0]], [[0, 1, 0], [1, 0, 0]], [[1, 1, 1], [0, 1, 0]]] as const) {
            const q = lookAt([0, 0, 0], target, up).rotation;
            expect(Math.hypot(...q), `${target} ${up}`).toBeCloseTo(1, 6);
        }
    });

    it('lensWords turns the field of view into focal lengths and puts the principal point at the centre', () => {
        const w = lensWords({ width: 64, height: 48, fovY: Math.PI / 2, near: 0.1, far: 50 });
        expect(Array.from(w)).toEqual(Array.from(new Float32Array([24, 24, 32, 24, 0.1, 50])));
        expect(poseWords([{ position: [1, 2, 3], rotation: [0, 0, 0, 1] }])).toEqual(new Float32Array([1, 2, 3, 0, 0, 0, 1]));
    });

    it('idColors gives each body its own visible colour and no body black', () => {
        const rgba = idColors(new Uint32Array([0, 1, 1, NO_ENTITY]));
        expect(Array.from(rgba.slice(8, 12))).toEqual(Array.from(rgba.slice(4, 8)));
        expect(Array.from(rgba.slice(0, 4))).not.toEqual(Array.from(rgba.slice(4, 8)));
        expect(Array.from(rgba.slice(12, 16))).toEqual([0, 0, 0, 0]);
        for (const k of [0, 4]) expect(rgba[k + 3]).toBe(255);
    });
});
