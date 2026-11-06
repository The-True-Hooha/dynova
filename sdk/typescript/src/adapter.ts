import { FLOATS_PER_TRANSFORM } from './TransformView.js';

export interface TransformSource {
    readTransforms(): Float32Array;
}

export interface AdapterOptions {
    /** Slot `i` shows entity `i + idOffset`. Non-negative integer, default 0. */
    idOffset?: number;
    /**
     * Per-entity scale, 3 floats (x, y, z) per entity id, indexed by entity id. The transform buffer carries no
     * size, so pass the dimensions you spawned with. Read on every update; missing entries mean 1.
     */
    scales?: ArrayLike<number>;
}

export interface UpdateResult {
    /** Slots drawn this update. */
    live: number;
    /** Slots hidden because the entity is destroyed, empty or non-finite. */
    hidden: number;
    /** Entities that exist but have no slot because the renderer's capacity is too small. */
    truncated: number;
}

/** What a renderer implements: receives decoded, renderer-neutral poses. The quaternion is unit length. */
export interface InstanceSink {
    show(slot: number, id: number, x: number, y: number, z: number, qx: number, qy: number, qz: number, qw: number, sx: number, sy: number, sz: number): void;
    hide(slot: number): void;
}

export function assertIdOffset(offset: number): void {
    if (!Number.isInteger(offset) || offset < 0) throw new RangeError(`idOffset must be a non-negative integer, got ${offset}`);
}

/**
 * Reads the latest transform frame and feeds every slot below `capacity` to `sink`. Slots beyond the available
 * entities are hidden so stale instances never linger. Pass `Infinity` when the renderer has no fixed slot count.
 */
export function decodeTransforms(source: TransformSource, capacity: number, sink: InstanceSink, options: AdapterOptions = {}): UpdateResult {
    const offset = options.idOffset ?? 0;
    assertIdOffset(offset);
    if (typeof capacity !== 'number' || Number.isNaN(capacity) || capacity < 0) throw new RangeError(`capacity must be a non-negative number, got ${capacity}`);
    capacity = Math.floor(capacity);
    const buf = source.readTransforms();
    const entities = Math.max(0, Math.floor(buf.length / FLOATS_PER_TRANSFORM) - offset);
    const slots = Math.min(capacity, entities);
    const scales = options.scales;
    let live = 0;
    for (let slot = 0; slot < slots; slot++) {
        const id = slot + offset;
        const o = id * FLOATS_PER_TRANSFORM;
        const qx = buf[o + 3], qy = buf[o + 4], qz = buf[o + 5], qw = buf[o + 6];
        const sx = scales?.[id * 3] ?? 1, sy = scales?.[id * 3 + 1] ?? 1, sz = scales?.[id * 3 + 2] ?? 1;
        const finite = Number.isFinite(buf[o]) && Number.isFinite(buf[o + 1]) && Number.isFinite(buf[o + 2])
            && Number.isFinite(qx) && Number.isFinite(qy) && Number.isFinite(qz) && Number.isFinite(qw)
            && Number.isFinite(sx) && Number.isFinite(sy) && Number.isFinite(sz);
        if (!finite || (qw === 0 && qx === 0 && qy === 0 && qz === 0)) {
            sink.hide(slot);
            continue;
        }
        const inv = 1 / Math.hypot(qx, qy, qz, qw);
        sink.show(slot, id, buf[o], buf[o + 1], buf[o + 2], qx * inv, qy * inv, qz * inv, qw * inv, sx, sy, sz);
        live++;
    }
    if (Number.isFinite(capacity)) for (let slot = slots; slot < capacity; slot++) sink.hide(slot);
    return { live, hidden: slots - live, truncated: Math.max(0, entities - capacity) };
}
