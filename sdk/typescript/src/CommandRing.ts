export const SLOT_BYTES = 64;

export const TAG_SPAWN   = 0;
export const TAG_DESTROY = 1;
export const TAG_FORCE   = 2;
export const TAG_GROW    = 3;
export const TAG_GRAVITY = 4;
export const TAG_HULL_VERTS = 5;
export const TAG_HULL_COMMIT = 6;
export const TAG_INSPECT = 7;
export const TAG_TELEPORT = 8;
export const TAG_RECORD = 9;
export const TAG_SEEK = 10;
export const TAG_PAUSE = 11;
export const TAG_JOINT_CREATE = 12;
export const TAG_JOINT_DESTROY = 13;
export const TAG_JOINT_MOTOR = 14;
export const TAG_KINEMATIC_BATCH = 15;
export const TAG_RAYCAST = 16;
export const TAG_OVERLAP_SPHERE = 17;
export const TAG_CHECKPOINT = 18;
export const TAG_RESET = 19;
export const NO_ENTITY = 0xffffffff;
export const MAX_QUERY_RANGE = 1e6;
export const KIN_SAMPLES_PER_SLOT = 3;
export const MAX_KIN_TICKS = 65535;
export const HULL_VERTS_PER_SLOT = 4;
export const MAX_HULL_VERTS = 32;
export const MAX_HULLS = 1024;
export const MAX_HULL_COORD = 1e4;
export const MAX_CAPACITY = 1 << 18;

export const JOINT_BALL = 0;
export const JOINT_HINGE = 1;
export const JOINT_FIXED = 2;
export const JOINT_SLIDER = 3;
export const JOINT_DISTANCE = 4;
export const MAX_JOINT_ID = 1 << 24;
export const MAX_ANGLE_LIMIT = 3.1;
export const MAX_LINEAR_LIMIT = 1e4;
export const MAX_MOTOR_SPEED = 1e3;
export const MAX_MOTOR_FORCE = 1e6;

export const SHAPE_BOX    = 0;
export const SHAPE_SPHERE = 1;
export const SHAPE_CAPSULE = 2;
export const SHAPE_HULL = 3;

export interface SpawnDesc {
    id: number;
    gen: number;
    x: number; y: number; z: number;
    qx: number; qy: number; qz: number; qw: number;
    shape?: number;
    hx?: number; hy?: number; hz?: number;
    invMass?: number;
    restitution?: number;
    friction?: number;
    hull?: number;
}

export interface DestroyDesc {
    id: number;
    gen: number;
}

export interface TeleportDesc {
    id: number;
    gen: number;
    x: number; y: number; z: number;
}

/** One telemetry sample: glide static body `id` to (x, y, z), arriving after `ticks` ticks. */
export interface KinematicSample {
    id: number; gen: number;
    x: number; y: number; z: number;
    ticks: number;
}

export interface RaycastDesc {
    id: number;
    origin: readonly [number, number, number];
    dir: readonly [number, number, number];
    maxDistance: number;
    ignore: number;
}

export interface OverlapSphereDesc {
    id: number;
    center: readonly [number, number, number];
    radius: number;
    ignore: number;
}

export interface JointCreateDesc {
    id: number;
    kind: number;
    a: number; genA: number;
    b: number; genB: number;
    anchor: readonly [number, number, number];
    axis: readonly [number, number, number];
    limits?: readonly [number, number];
    motorVelocity?: number;
    motorMax?: number;
}

export interface ForceDesc {
    id: number;
    gen: number;
    x: number; y: number; z: number;
}

export class CommandRing {
    private head: Int32Array;
    private tail: Int32Array;
    private capacity: number;
    private mask: number;
    private view: DataView;

    constructor(
        memory: WebAssembly.Memory,
        slotsByteOffset: number,
        headByteOffset: number,
        tailByteOffset: number,
        capacity: number,
    ) {
        this.capacity = capacity;
        this.mask = capacity - 1;
        this.head = new Int32Array(memory.buffer, headByteOffset, 1);
        this.tail = new Int32Array(memory.buffer, tailByteOffset, 1);
        this.view = new DataView(memory.buffer, slotsByteOffset, capacity * SLOT_BYTES);
    }

    private isFull(): boolean {
        const tail = Atomics.load(this.tail, 0);
        const head = Atomics.load(this.head, 0);
        return (tail - head) >>> 0 >= this.capacity;
    }

    private slotOffset(tail: number): number {
        return (tail & this.mask) * SLOT_BYTES;
    }

    spawn(desc: SpawnDesc): boolean {
        if (this.isFull()) return false;
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        this.view.setUint8(off,      TAG_SPAWN);
        this.view.setUint32(off + 1, desc.id,  true);
        this.view.setUint8(off + 5,  desc.gen);
        this.view.setFloat32(off + 6,  desc.x,  true);
        this.view.setFloat32(off + 10, desc.y,  true);
        this.view.setFloat32(off + 14, desc.z,  true);
        this.view.setFloat32(off + 18, desc.qx, true);
        this.view.setFloat32(off + 22, desc.qy, true);
        this.view.setFloat32(off + 26, desc.qz, true);
        this.view.setFloat32(off + 30, desc.qw, true);
        this.view.setUint8(off + 34, desc.shape ?? SHAPE_BOX);
        this.view.setUint8(off + 35, 0);
        this.view.setFloat32(off + 36, desc.hx ?? 0.5, true);
        this.view.setFloat32(off + 40, desc.hy ?? 0.5, true);
        this.view.setFloat32(off + 44, desc.hz ?? 0.5, true);
        this.view.setFloat32(off + 48, desc.invMass ?? 1, true);
        this.view.setFloat32(off + 52, desc.restitution ?? 0.3, true);
        this.view.setFloat32(off + 56, desc.friction ?? 0.5, true);
        this.view.setUint32(off + 60, desc.hull ?? 0, true);
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    destroy(desc: DestroyDesc): boolean {
        if (this.isFull()) return false;
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        this.view.setUint8(off,      TAG_DESTROY);
        this.view.setUint32(off + 1, desc.id,  true);
        this.view.setUint8(off + 5,  desc.gen);
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    force(desc: ForceDesc): boolean {
        if (this.isFull()) return false;
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        this.view.setUint8(off,      TAG_FORCE);
        this.view.setUint32(off + 1, desc.id,  true);
        this.view.setUint8(off + 5,  desc.gen);
        this.view.setFloat32(off + 6,  desc.x,  true);
        this.view.setFloat32(off + 10, desc.y,  true);
        this.view.setFloat32(off + 14, desc.z,  true);
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    grow(capacity: number): boolean {
        if (this.isFull()) return false;
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        for (let i = 0; i < SLOT_BYTES; i++) this.view.setUint8(off + i, 0);
        this.view.setUint8(off, TAG_GROW);
        this.view.setUint32(off + 6, capacity, true);
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    /** `coords` holds 1 to 4 vertices as flat x,y,z triples. */
    hullVerts(hull: number, coords: ArrayLike<number>): boolean {
        const count = coords.length / 3;
        if (!Number.isInteger(count) || count < 1 || count > HULL_VERTS_PER_SLOT) throw new RangeError(`a hull chunk holds 1 to ${HULL_VERTS_PER_SLOT} vertices, got ${count}`);
        if (this.isFull()) return false;
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        for (let i = 0; i < SLOT_BYTES; i++) this.view.setUint8(off + i, 0);
        this.view.setUint8(off, TAG_HULL_VERTS);
        this.view.setUint32(off + 1, hull, true);
        this.view.setUint8(off + 5, count);
        for (let i = 0; i < coords.length; i++) this.view.setFloat32(off + 6 + i * 4, coords[i], true);
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    /** `count` is the total number of vertices uploaded for this hull; the runtime rejects a mismatch. */
    hullCommit(hull: number, count: number): boolean {
        if (this.isFull()) return false;
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        for (let i = 0; i < SLOT_BYTES; i++) this.view.setUint8(off + i, 0);
        this.view.setUint8(off, TAG_HULL_COMMIT);
        this.view.setUint32(off + 1, hull, true);
        this.view.setUint8(off + 5, count);
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    inspect(on: boolean): boolean {
        if (this.isFull()) return false;
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        for (let i = 0; i < SLOT_BYTES; i++) this.view.setUint8(off + i, 0);
        this.view.setUint8(off, TAG_INSPECT);
        this.view.setUint8(off + 6, on ? 1 : 0);
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    teleport(desc: TeleportDesc): boolean {
        if (this.isFull()) return false;
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        for (let i = 0; i < SLOT_BYTES; i++) this.view.setUint8(off + i, 0);
        this.view.setUint8(off, TAG_TELEPORT);
        this.view.setUint32(off + 1, desc.id, true);
        this.view.setUint8(off + 5, desc.gen);
        this.view.setFloat32(off + 6, desc.x, true);
        this.view.setFloat32(off + 10, desc.y, true);
        this.view.setFloat32(off + 14, desc.z, true);
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    jointCreate(d: JointCreateDesc): boolean {
        if (this.isFull()) return false;
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        for (let i = 0; i < SLOT_BYTES; i++) this.view.setUint8(off + i, 0);
        this.view.setUint8(off, TAG_JOINT_CREATE);
        this.view.setUint32(off + 1, d.id, true);
        this.view.setUint8(off + 5, d.kind);
        this.view.setUint8(off + 6, d.genA);
        this.view.setUint8(off + 7, d.genB);
        this.view.setUint32(off + 8, d.a, true);
        this.view.setUint32(off + 12, d.b, true);
        for (let i = 0; i < 3; i++) {
            this.view.setFloat32(off + 16 + i * 4, d.anchor[i], true);
            this.view.setFloat32(off + 28 + i * 4, d.axis[i], true);
        }
        if (d.limits) {
            this.view.setFloat32(off + 40, d.limits[0], true);
            this.view.setFloat32(off + 44, d.limits[1], true);
            this.view.setUint8(off + 56, 1);
        }
        this.view.setFloat32(off + 48, d.motorVelocity ?? 0, true);
        this.view.setFloat32(off + 52, d.motorMax ?? 0, true);
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    jointDestroy(id: number): boolean {
        if (this.isFull()) return false;
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        for (let i = 0; i < SLOT_BYTES; i++) this.view.setUint8(off + i, 0);
        this.view.setUint8(off, TAG_JOINT_DESTROY);
        this.view.setUint32(off + 1, id, true);
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    jointMotor(id: number, velocity: number, maxForce: number): boolean {
        if (this.isFull()) return false;
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        for (let i = 0; i < SLOT_BYTES; i++) this.view.setUint8(off + i, 0);
        this.view.setUint8(off, TAG_JOINT_MOTOR);
        this.view.setUint32(off + 1, id, true);
        this.view.setFloat32(off + 6, velocity, true);
        this.view.setFloat32(off + 10, maxForce, true);
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    raycast(d: RaycastDesc): boolean {
        if (this.isFull()) return false;
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        for (let i = 0; i < SLOT_BYTES; i++) this.view.setUint8(off + i, 0);
        this.view.setUint8(off, TAG_RAYCAST);
        this.view.setUint32(off + 1, d.id, true);
        d.origin.forEach((v, k) => this.view.setFloat32(off + 6 + k * 4, v, true));
        d.dir.forEach((v, k) => this.view.setFloat32(off + 18 + k * 4, v, true));
        this.view.setFloat32(off + 30, d.maxDistance, true);
        this.view.setUint32(off + 34, d.ignore, true);
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    overlapSphere(d: OverlapSphereDesc): boolean {
        if (this.isFull()) return false;
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        for (let i = 0; i < SLOT_BYTES; i++) this.view.setUint8(off + i, 0);
        this.view.setUint8(off, TAG_OVERLAP_SPHERE);
        this.view.setUint32(off + 1, d.id, true);
        d.center.forEach((v, k) => this.view.setFloat32(off + 6 + k * 4, v, true));
        this.view.setFloat32(off + 18, d.radius, true);
        this.view.setUint32(off + 34, d.ignore, true);
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    /** Writes up to `KIN_SAMPLES_PER_SLOT` samples into one slot. */
    kinematicBatch(samples: readonly KinematicSample[]): boolean {
        if (samples.length < 1 || samples.length > KIN_SAMPLES_PER_SLOT) throw new RangeError(`a slot holds 1..${KIN_SAMPLES_PER_SLOT} kinematic samples, got ${samples.length}`);
        if (this.isFull()) return false;
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        for (let i = 0; i < SLOT_BYTES; i++) this.view.setUint8(off + i, 0);
        this.view.setUint8(off, TAG_KINEMATIC_BATCH);
        this.view.setUint8(off + 1, samples.length);
        samples.forEach((s, k) => {
            const o = off + 4 + k * 20;
            this.view.setUint32(o, s.id, true);
            this.view.setUint8(o + 4, s.gen);
            this.view.setUint16(o + 6, s.ticks, true);
            this.view.setFloat32(o + 8, s.x, true);
            this.view.setFloat32(o + 12, s.y, true);
            this.view.setFloat32(o + 16, s.z, true);
        });
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    private flag(tag: number, on: boolean): boolean {
        if (this.isFull()) return false;
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        for (let i = 0; i < SLOT_BYTES; i++) this.view.setUint8(off + i, 0);
        this.view.setUint8(off, tag);
        this.view.setUint8(off + 6, on ? 1 : 0);
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    private tagOnly(tag: number): boolean {
        if (this.isFull()) return false;
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        for (let i = 0; i < SLOT_BYTES; i++) this.view.setUint8(off + i, 0);
        this.view.setUint8(off, tag);
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    checkpoint(): boolean {
        return this.tagOnly(TAG_CHECKPOINT);
    }

    reset(): boolean {
        return this.tagOnly(TAG_RESET);
    }

    record(on: boolean): boolean {
        return this.flag(TAG_RECORD, on);
    }

    pause(on: boolean): boolean {
        return this.flag(TAG_PAUSE, on);
    }

    seek(tick: number): boolean {
        if (this.isFull()) return false;
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        for (let i = 0; i < SLOT_BYTES; i++) this.view.setUint8(off + i, 0);
        this.view.setUint8(off, TAG_SEEK);
        this.view.setUint32(off + 1, tick, true);
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    gravity(x: number, y: number, z: number): boolean {
        if (this.isFull()) return false;
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        for (let i = 0; i < SLOT_BYTES; i++) this.view.setUint8(off + i, 0);
        this.view.setUint8(off, TAG_GRAVITY);
        this.view.setFloat32(off + 6, x, true);
        this.view.setFloat32(off + 10, y, true);
        this.view.setFloat32(off + 14, z, true);
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    get pendingCount(): number {
        const tail = Atomics.load(this.tail, 0);
        const head = Atomics.load(this.head, 0);
        return (tail - head) >>> 0;
    }

    reacquire(memory: WebAssembly.Memory, slotsByteOffset: number, headByteOffset: number, tailByteOffset: number): void {
        this.head = new Int32Array(memory.buffer, headByteOffset, 1);
        this.tail = new Int32Array(memory.buffer, tailByteOffset, 1);
        this.view = new DataView(memory.buffer, slotsByteOffset, this.capacity * SLOT_BYTES);
    }
}
