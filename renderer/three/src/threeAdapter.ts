import { DynamicDrawUsage, type InstancedMesh } from 'three';
import { FLOATS_PER_TRANSFORM } from '@dynova/sdk';

export interface TransformSource {
    readTransforms(): Float32Array;
}

export interface ThreeAdapterOptions {
    /** Instance index of entity id 0. Entities map to `id + idOffset`. Default 0. */
    idOffset?: number;
}

export interface ThreeAdapter {
    /** Copy the latest frame into the mesh's instance matrices. Returns the number of live instances written. */
    update(): number;
}

export function threeAdapter(source: TransformSource, mesh: InstancedMesh, options: ThreeAdapterOptions = {}): ThreeAdapter {
    const offset = options.idOffset ?? 0;
    if (!Number.isInteger(offset) || offset < 0) throw new RangeError(`idOffset must be a non-negative integer, got ${offset}`);
    mesh.instanceMatrix.setUsage(DynamicDrawUsage);
    // three caches the bounding sphere on first render; moving instances would be culled once they leave it.
    mesh.frustumCulled = false;
    return {
        update() {
            const buf = source.readTransforms();
            const entities = Math.floor(buf.length / FLOATS_PER_TRANSFORM);
            const out = mesh.instanceMatrix.array as Float32Array;
            const capacity = Math.min(mesh.count, out.length / 16);
            const slots = Math.max(0, Math.min(capacity, entities - offset));
            let live = 0;
            for (let i = 0; i < slots; i++) {
                const o = (i + offset) * FLOATS_PER_TRANSFORM;
                const qx = buf[o + 3], qy = buf[o + 4], qz = buf[o + 5], qw = buf[o + 6];
                const m = i * 16;
                const finite = Number.isFinite(buf[o] + buf[o + 1] + buf[o + 2] + qx + qy + qz + qw);
                if (!finite || (qw === 0 && qx === 0 && qy === 0 && qz === 0)) {
                    hide(out, m);
                    continue;
                }
                const len = Math.hypot(qx, qy, qz, qw);
                const inv = 1 / len;
                composeInto(out, m, buf[o], buf[o + 1], buf[o + 2], qx * inv, qy * inv, qz * inv, qw * inv);
                live++;
            }
            for (let i = slots; i < capacity; i++) hide(out, i * 16);
            mesh.instanceMatrix.needsUpdate = true;
            return live;
        },
    };
}

function hide(out: Float32Array, m: number): void {
    out.fill(0, m, m + 16);
    out[m + 15] = 1;
}

function composeInto(out: Float32Array, m: number, x: number, y: number, z: number, qx: number, qy: number, qz: number, qw: number): void {
    const xx = qx * qx, yy = qy * qy, zz = qz * qz;
    const xy = qx * qy, xz = qx * qz, yz = qy * qz;
    const wx = qw * qx, wy = qw * qy, wz = qw * qz;
    out[m] = 1 - 2 * (yy + zz); out[m + 1] = 2 * (xy + wz);     out[m + 2] = 2 * (xz - wy);      out[m + 3] = 0;
    out[m + 4] = 2 * (xy - wz); out[m + 5] = 1 - 2 * (xx + zz); out[m + 6] = 2 * (yz + wx);      out[m + 7] = 0;
    out[m + 8] = 2 * (xz + wy); out[m + 9] = 2 * (yz - wx);     out[m + 10] = 1 - 2 * (xx + yy); out[m + 11] = 0;
    out[m + 12] = x;            out[m + 13] = y;                out[m + 14] = z;                 out[m + 15] = 1;
}
