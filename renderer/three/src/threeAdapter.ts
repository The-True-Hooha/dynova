import { DynamicDrawUsage, type InstancedMesh } from 'three';
import { FLOATS_PER_TRANSFORM } from '@dynova/sdk';

export interface TransformSource {
    readTransforms(): Float32Array;
}

export interface ThreeAdapterOptions {
    /** Instance `i` shows entity `i + idOffset`. Non-negative integer, default 0. */
    idOffset?: number;
    /**
     * Per-entity scale, 3 floats (x, y, z) per entity id, indexed by entity id. The transform buffer carries no
     * size, so pass the dimensions you spawned with. Read on every update; missing entries mean 1.
     */
    scales?: ArrayLike<number>;
}

export interface UpdateResult {
    /** Instances drawn this update. */
    live: number;
    /** Instances hidden because the entity is destroyed, empty or non-finite. */
    hidden: number;
    /** Entities that exist but have no instance because `mesh.count` is too small. */
    truncated: number;
}

export interface ThreeAdapter {
    update(): UpdateResult;
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
            const entities = Math.max(0, Math.floor(buf.length / FLOATS_PER_TRANSFORM) - offset);
            const out = mesh.instanceMatrix.array as Float32Array;
            const capacity = Math.min(mesh.count, out.length / 16);
            const slots = Math.min(capacity, entities);
            const scales = options.scales;
            let live = 0;
            for (let i = 0; i < slots; i++) {
                const id = i + offset;
                const o = id * FLOATS_PER_TRANSFORM;
                const m = i * 16;
                const qx = buf[o + 3], qy = buf[o + 4], qz = buf[o + 5], qw = buf[o + 6];
                const sx = scales?.[id * 3] ?? 1, sy = scales?.[id * 3 + 1] ?? 1, sz = scales?.[id * 3 + 2] ?? 1;
                const finite = Number.isFinite(buf[o] + buf[o + 1] + buf[o + 2] + qx + qy + qz + qw + sx + sy + sz);
                if (!finite || (qw === 0 && qx === 0 && qy === 0 && qz === 0)) {
                    hide(out, m);
                    continue;
                }
                const inv = 1 / Math.hypot(qx, qy, qz, qw);
                composeInto(out, m, buf[o], buf[o + 1], buf[o + 2], qx * inv, qy * inv, qz * inv, qw * inv, sx, sy, sz);
                live++;
            }
            for (let i = slots; i < capacity; i++) hide(out, i * 16);
            mesh.instanceMatrix.needsUpdate = true;
            return { live, hidden: slots - live, truncated: Math.max(0, entities - capacity) };
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
