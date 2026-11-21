import { DynamicDrawUsage, type InstancedMesh } from 'three';
import { assertIdOffset, decodeTransforms, type AdapterOptions, type InstanceSink, type TransformSource, type UpdateResult } from '@dynova/sdk';

export type { TransformSource, UpdateResult };
export type ThreeAdapterOptions = AdapterOptions;

export interface ThreeAdapter {
    update(): UpdateResult;
}

export function threeAdapter(source: TransformSource, mesh: InstancedMesh, options: ThreeAdapterOptions = {}): ThreeAdapter {
    assertIdOffset(options.idOffset ?? 0);
    mesh.instanceMatrix.setUsage(DynamicDrawUsage);
    // three caches the bounding sphere on first render; moving instances would be culled once they leave it.
    mesh.frustumCulled = false;
    return {
        update() {
            const out = mesh.instanceMatrix.array as Float32Array;
            const sink: InstanceSink = {
                show: (slot, _id, x, y, z, qx, qy, qz, qw, sx, sy, sz) => composeInto(out, slot * 16, x, y, z, qx, qy, qz, qw, sx, sy, sz),
                hide: (slot) => hide(out, slot * 16),
            };
            const result = decodeTransforms(source, Math.min(mesh.count, out.length / 16), sink, options);
            mesh.instanceMatrix.needsUpdate = true;
            return result;
        },
    };
}

function hide(out: Float32Array, m: number): void {
    out.fill(0, m, m + 16);
    out[m + 15] = 1;
}

function composeInto(
    out: Float32Array, m: number, x: number, y: number, z: number,
    qx: number, qy: number, qz: number, qw: number, sx: number, sy: number, sz: number,
): void {
    const xx = qx * qx, yy = qy * qy, zz = qz * qz;
    const xy = qx * qy, xz = qx * qz, yz = qy * qz;
    const wx = qw * qx, wy = qw * qy, wz = qw * qz;
    out[m] = (1 - 2 * (yy + zz)) * sx; out[m + 1] = 2 * (xy + wz) * sx;     out[m + 2] = 2 * (xz - wy) * sx;      out[m + 3] = 0;
    out[m + 4] = 2 * (xy - wz) * sy;   out[m + 5] = (1 - 2 * (xx + zz)) * sy; out[m + 6] = 2 * (yz + wx) * sy;    out[m + 7] = 0;
    out[m + 8] = 2 * (xz + wy) * sz;   out[m + 9] = 2 * (yz - wx) * sz;     out[m + 10] = (1 - 2 * (xx + yy)) * sz; out[m + 11] = 0;
    out[m + 12] = x;                   out[m + 13] = y;                     out[m + 14] = z;                      out[m + 15] = 1;
}
