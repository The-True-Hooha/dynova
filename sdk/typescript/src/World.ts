import { HEADER_WORDS, QUERY_ROW_WORDS, TransformView } from './TransformView.js';
import { positionControl, type PositionGains } from './control.js';
import type { RecordingState } from './TransformView.js';
import { CommandRing, JOINT_BALL, JOINT_HINGE, JOINT_SLIDER, JOINT_DISTANCE, MAX_ANGLE_LIMIT, MAX_JOINT_ID, MAX_LINEAR_LIMIT, MAX_MOTOR_FORCE, MAX_MOTOR_SPEED, SHAPE_HULL, HULL_VERTS_PER_SLOT, MAX_CAPACITY, MAX_HULLS, MAX_HULL_COORD, MAX_HULL_VERTS, SpawnDesc, ForceDesc, DestroyDesc, TeleportDesc, JointCreateDesc, KIN_SAMPLES_PER_SLOT, MAX_KIN_TICKS, KinematicSample, RaycastDesc, OverlapSphereDesc, NO_ENTITY, MAX_QUERY_RANGE, MAX_PARTICLES, MAX_CHAIN, MAX_CLOTH_SIDE, MAX_SOFT_SIDE, MAX_GROUP_ID, ChainDesc, ClothDesc, SoftDesc, AttachDesc } from './CommandRing.js';
import { EntityIdAllocator, EntityHandle } from './EntityIdAllocator.js';
import type { MemoryMode } from './memory.js';

export interface WorldConfig {
    capacity: number;
    ringCapacity: number;
    /** Commands held in JS when the ring is full before spawn/destroy/force throw. Default 65536. */
    maxDeferredCommands?: number;
}

export interface WorldLayout {
    headerByteOffset: number;
    ringSlotsByteOffset: number;
    ringHeadByteOffset: number;
    ringTailByteOffset: number;
    idCounterByteOffset: number;
}

export interface RuntimePointers {
    header_ptr(): number;
    ring_slots_ptr(): number;
    ring_head_ptr(): number;
    ring_tail_ptr(): number;
    id_counter_ptr(): number;
    /** Header size the wasm build was compiled with; compared with this package's `HEADER_WORDS`. */
    header_words?(): number;
}

export function layoutFromRuntime(rt: RuntimePointers): WorldLayout {
    const words = rt.header_words?.();
    if (words !== undefined && words !== HEADER_WORDS) {
        throw new Error(`wasm header has ${words} words but the SDK expects ${HEADER_WORDS}: rebuild the wasm package (make build)`);
    }
    return {
        headerByteOffset: rt.header_ptr(),
        ringSlotsByteOffset: rt.ring_slots_ptr(),
        ringHeadByteOffset: rt.ring_head_ptr(),
        ringTailByteOffset: rt.ring_tail_ptr(),
        idCounterByteOffset: rt.id_counter_ptr(),
    };
}

export interface SpawnOptions {
    x?: number; y?: number; z?: number;
    qx?: number; qy?: number; qz?: number; qw?: number;
    shape?: number;
    hx?: number; hy?: number; hz?: number;
    invMass?: number;
    restitution?: number;
    friction?: number;
    /** Hull id from `registerHull`; required with `shape: SHAPE_HULL`. */
    hull?: number;
}

export type QueryResult =
    | { query: number; kind: 'hit'; entity: number; distance: number; point: [number, number, number]; normal: [number, number, number] }
    | { query: number; kind: 'miss' }
    | { query: number; kind: 'overlap'; entity: number; depth: number; point: [number, number, number]; normal: [number, number, number] }
    | { query: number; kind: 'overlap-none' }
    | { query: number; kind: 'overlap-more' };

export interface RaycastOptions {
    origin: readonly [number, number, number];
    /** Any non-zero direction; it is normalized, and `maxDistance` is in world units along it. */
    direction: readonly [number, number, number];
    maxDistance: number;
    /** A body the ray passes through, for example the robot that carries the sensor. */
    ignore?: EntityHandle;
}

export interface OverlapSphereOptions {
    center: readonly [number, number, number];
    radius: number;
    ignore?: EntityHandle;
}

export interface JointState {
    joint: number;
    kind: number;
    /** Hinge angle in radians or slider offset in metres, with the sign limits and the motor use. */
    position: number;
    /** Rate of `position`, in rad/s or m/s. */
    velocity: number;
}

export interface JointDrive {
    joint: JointHandle;
    target: number;
    gains: PositionGains;
}

export interface KinematicTarget {
    handle: EntityHandle;
    x: number; y: number; z: number;
    /** Ticks to arrive in, an integer from 1 to `MAX_KIN_TICKS`. */
    ticks: number;
}

export interface JointOptions {
    /** One of `JOINT_BALL`, `JOINT_HINGE`, `JOINT_FIXED`, `JOINT_SLIDER`, `JOINT_DISTANCE`. */
    kind: number;
    a: EntityHandle;
    b: EntityHandle;
    /** World-space point both bodies are pinned to at creation. For `JOINT_DISTANCE` it is the point on body `a`. */
    anchor: readonly [number, number, number];
    /** `JOINT_DISTANCE` only: the world-space point on body `b`. The two points must be at least 1 mm apart; their distance at creation is the rod length. */
    anchorB?: readonly [number, number, number];
    /** World-space hinge or slider axis; required for those kinds. */
    axis?: readonly [number, number, number];
    /** Hinge angle in radians (within +-3.1), slider offset, or the stretch of a distance joint (its length minus the rod length), all measured from the pose at creation; not for ball and fixed joints. */
    limits?: readonly [number, number];
    /** Motor target in rad/s or m/s and the largest torque or force it may apply; only for hinge and slider. */
    motor?: { velocity: number; maxForce: number };
}

/** A group of particles a command made: its id, the index of its first particle in `readParticles()` and how many it has. */
export interface ParticleGroup {
    readonly id: number;
    readonly first: number;
    readonly count: number;
}

export interface JointHandle {
    readonly id: number;
}

export interface StepDriver {
    step(): void;
}

export interface GrowthEvent {
    epoch: number;
    capacity: number;
}

type Queued =
    | { k: 'spawn'; d: SpawnDesc }
    | { k: 'destroy'; d: DestroyDesc }
    | { k: 'force'; d: ForceDesc }
    | { k: 'grow'; capacity: number }
    | { k: 'gravity'; v: [number, number, number] }
    | { k: 'inspect'; on: boolean }
    | { k: 'teleport'; d: TeleportDesc }
    | { k: 'record'; on: boolean }
    | { k: 'pause'; on: boolean }
    | { k: 'checkpoint' }
    | { k: 'reset' }
    | { k: 'seek'; tick: number }
    | { k: 'jointCreate'; d: JointCreateDesc }
    | { k: 'jointDestroy'; id: number }
    | { k: 'jointMotor'; id: number; velocity: number; maxForce: number }
    | { k: 'kinematic'; samples: KinematicSample[] }
    | { k: 'raycast'; d: RaycastDesc }
    | { k: 'overlap'; d: OverlapSphereDesc }
    | { k: 'chain'; d: ChainDesc }
    | { k: 'cloth'; d: ClothDesc }
    | { k: 'soft'; d: SoftDesc }
    | { k: 'attach'; d: AttachDesc }
    | { k: 'detach'; particle: number }
    | { k: 'hullVerts'; hull: number; coords: number[] }
    | { k: 'hullCommit'; hull: number; count: number };

const DEFAULT_MAX_DEFERRED = 1 << 16;
const MAX_TELEPORT_COORD = 1e6;
const MAX_PARTICLE_GROUPS = 4096;

export class World {
    readonly memory: WebAssembly.Memory;
    readonly mode: MemoryMode;
    readonly layout: WorldLayout;
    onBufferGrowth: ((e: GrowthEvent) => void) | null = null;

    private transforms: TransformView;
    private ring: CommandRing;
    private ids: EntityIdAllocator;
    private ringCapacity: number;
    private driver: StepDriver | null;
    private lastBuffer: ArrayBufferLike;
    private knownEpoch: number;
    private fatal: string | null = null;
    private onDispose: (() => void) | null = null;
    private deferred: Queued[] = [];
    private deferredHead = 0;
    private maxDeferred: number;
    private nextQuery = 0;
    private queryTail = 0;
    private deferredTotal = 0;
    private rejected = 0;
    private growths = 0;
    private pendingGrow = 0;
    private nextHull = 0;
    /** Joint ids only grow: a seek can drop joints the SDK believes are live, and a reused id could then hit a different joint. */
    private nextJoint = 0;
    /** Group ids only grow, like joint ids, so a seek that drops a group cannot hand its id to another. */
    private nextGroup = 0;
    /** Particles the commands sent so far would make if every one is accepted: the index the next group starts at. */
    private nextParticle = 0;
    private knownSeeks = 0;

    private constructor(
        config: WorldConfig,
        memory: WebAssembly.Memory,
        mode: MemoryMode,
        layout: WorldLayout,
        driver: StepDriver | null,
    ) {
        this.memory = memory;
        this.mode = mode;
        this.layout = layout;
        this.driver = driver;
        this.ringCapacity = config.ringCapacity;
        this.maxDeferred = config.maxDeferredCommands ?? DEFAULT_MAX_DEFERRED;
        this.lastBuffer = memory.buffer;

        this.transforms = new TransformView(memory, layout.headerByteOffset);
        this.knownEpoch = this.transforms.epoch;
        this.ring = new CommandRing(
            memory,
            layout.ringSlotsByteOffset,
            layout.ringHeadByteOffset,
            layout.ringTailByteOffset,
            config.ringCapacity,
        );
        this.ids = new EntityIdAllocator(memory, layout.idCounterByteOffset, this.transforms.capacity);
    }

    /** Worker mode: another thread owns the runtime and steps it. */
    static attach(config: WorldConfig, memory: WebAssembly.Memory, mode: MemoryMode, layout: WorldLayout): World {
        return new World(config, memory, mode, layout, null);
    }

    /** Fallback mode: this thread owns the runtime; call `step()` yourself. */
    static attachSync(
        config: WorldConfig,
        memory: WebAssembly.Memory,
        mode: MemoryMode,
        layout: WorldLayout,
        driver: StepDriver,
    ): World {
        return new World(config, memory, mode, layout, driver);
    }

    /** @internal set by launchWorld so dispose() can stop the simulation worker. */
    setDisposer(fn: () => void): void { this.onDispose = fn; }

    /** @internal called when the simulation worker dies; further commands throw. */
    markFatal(message: string): void { this.fatal ??= message; }

    get failure(): string | null { return this.fatal; }

    dispose(): void {
        this.fatal ??= 'world disposed';
        this.onDispose?.();
        this.onDispose = null;
    }

    private assertAlive(): void {
        if (this.fatal) throw new Error(`world unavailable: ${this.fatal}`);
        this.syncLayout();
    }

    syncLayout(): boolean {
        let changed = false;
        if (this.memory.buffer !== this.lastBuffer) {
            this.lastBuffer = this.memory.buffer;
            const { layout } = this;
            this.transforms.reacquire();
            this.ring.reacquire(this.memory, layout.ringSlotsByteOffset, layout.ringHeadByteOffset, layout.ringTailByteOffset);
            this.ids.reacquire(this.memory, layout.idCounterByteOffset);
            changed = true;
        }
        const seeks = this.transforms.seekCount();
        // Only once nothing is in flight: a queued Spawn or Destroy is not in the runtime's tables yet, so resyncing now
        // would free its id (and hand it out twice) or revive a handle that is about to be destroyed.
        if (seeks !== this.knownSeeks && this.pendingCommands === 0) {
            this.knownSeeks = seeks;
            const { generations, live } = this.transforms.slotTables();
            this.ids.resync(generations, live);
        }
        const peek = this.transforms.peekEpoch();
        if ((peek & 1) === 0 && peek !== this.knownEpoch) {
            this.transforms.reacquire();
            this.applyEpoch();
            changed = true;
        }
        return changed;
    }

    private applyEpoch(): void {
        this.knownEpoch = this.transforms.epoch;
        this.ids.setCapacity(this.transforms.capacity);
        this.growths++;
        if (this.pendingGrow <= this.transforms.capacity) this.pendingGrow = 0;
        try {
            this.onBufferGrowth?.({ epoch: this.knownEpoch >> 1, capacity: this.transforms.capacity });
        } catch (err) {
            queueMicrotask(() => { throw err; });
        }
    }

    reacquireIfGrown(): boolean { return this.syncLayout(); }

    step(): void {
        if (!this.driver) throw new Error('step() is not available here: this world is stepped elsewhere (its worker, or the WorldBatch it belongs to)');
        const t0 = performance.now();
        this.driver.step();
        this.finishStep((performance.now() - t0) * 1000);
    }

    /** @internal what follows a step: whoever stepped the runtime (a `WorldBatch` steps many at once) reports the time it took. */
    finishStep(micros: number): void {
        this.transforms.recordStep(micros);
        this.syncLayout();
        this.flush();
    }

    private tryPush(c: Queued): boolean {
        switch (c.k) {
            case 'spawn': return this.ring.spawn(c.d);
            case 'destroy': return this.ring.destroy(c.d);
            case 'force': return this.ring.force(c.d);
            case 'grow': return this.ring.grow(c.capacity);
            case 'gravity': return this.ring.gravity(...c.v);
            case 'inspect': return this.ring.inspect(c.on);
            case 'teleport': return this.ring.teleport(c.d);
            case 'record': return this.ring.record(c.on);
            case 'pause': return this.ring.pause(c.on);
            case 'checkpoint': return this.ring.checkpoint();
            case 'reset': return this.ring.reset();
            case 'seek': return this.ring.seek(c.tick);
            case 'jointCreate': return this.ring.jointCreate(c.d);
            case 'jointDestroy': return this.ring.jointDestroy(c.id);
            case 'jointMotor': return this.ring.jointMotor(c.id, c.velocity, c.maxForce);
            case 'kinematic': return this.ring.kinematicBatch(c.samples);
            case 'raycast': return this.ring.raycast(c.d);
            case 'overlap': return this.ring.overlapSphere(c.d);
            case 'chain': return this.ring.particleChain(c.d);
            case 'cloth': return this.ring.particleCloth(c.d);
            case 'soft': return this.ring.particleSoft(c.d);
            case 'attach': return this.ring.particleAttach(c.d);
            case 'detach': return this.ring.particleDetach(c.particle);
            case 'hullVerts': return this.ring.hullVerts(c.hull, c.coords);
            case 'hullCommit': return this.ring.hullCommit(c.hull, c.count);
        }
    }

    flush(): number {
        while (this.deferredHead < this.deferred.length) {
            if (!this.tryPush(this.deferred[this.deferredHead])) break;
            this.deferredHead++;
        }
        if (this.deferredHead === this.deferred.length) {
            this.deferred.length = 0;
            this.deferredHead = 0;
        } else if (this.deferredHead > 1024 && this.deferredHead * 2 > this.deferred.length) {
            this.deferred = this.deferred.slice(this.deferredHead);
            this.deferredHead = 0;
        }
        return this.deferred.length - this.deferredHead;
    }

    private enqueue(c: Queued): boolean {
        if (this.flush() === 0 && this.tryPush(c)) return true;
        if (this.deferred.length - this.deferredHead >= this.maxDeferred) {
            this.rejected++;
            return false;
        }
        this.deferred.push(c);
        this.deferredTotal++;
        return true;
    }

    spawn(opts: SpawnOptions = {}): EntityHandle {
        this.assertAlive();
        if (opts.shape === SHAPE_HULL && !(Number.isInteger(opts.hull) && (opts.hull as number) >= 0 && (opts.hull as number) < this.nextHull)) {
            throw new RangeError(`spawn with SHAPE_HULL needs a hull id returned by registerHull, got ${opts.hull}`);
        }
        const handle = this.ids.alloc();
        const desc: SpawnDesc = {
            id: handle.id, gen: handle.gen,
            x: opts.x ?? 0, y: opts.y ?? 0, z: opts.z ?? 0,
            qx: opts.qx ?? 0, qy: opts.qy ?? 0, qz: opts.qz ?? 0, qw: opts.qw ?? 1,
            shape: opts.shape, hx: opts.hx, hy: opts.hy, hz: opts.hz,
            invMass: opts.invMass, restitution: opts.restitution, friction: opts.friction, hull: opts.hull,
        };
        if (!this.enqueue({ k: 'spawn', d: desc })) {
            this.ids.release(handle);
            throw new Error(`CommandRing full — ${this.deferred.length - this.deferredHead} commands deferred; increase ringCapacity or maxDeferredCommands`);
        }
        return handle;
    }

    /** The live handle for an entity id (what a tool that only knows ids, like the inspector, needs to issue commands), or null. */
    handleOf(id: number): EntityHandle | null {
        this.assertAlive();
        return this.ids.handleOf(id);
    }

    destroy(handle: EntityHandle): void {
        this.assertAlive();
        if (!this.ids.isLive(handle)) throw new Error(`stale handle {id: ${handle.id}, gen: ${handle.gen}}`);
        if (!this.enqueue({ k: 'destroy', d: { id: handle.id, gen: handle.gen } })) throw new Error('CommandRing full');
        this.ids.free(handle);
    }

    force(handle: EntityHandle, x: number, y: number, z: number): void {
        this.assertAlive();
        if (!this.ids.isLive(handle)) throw new Error(`stale handle {id: ${handle.id}, gen: ${handle.gen}}`);
        if (!this.enqueue({ k: 'force', d: { id: handle.id, gen: handle.gen, x, y, z } })) throw new Error('CommandRing full');
    }

    /**
     * Move a body to a position through the command ring, clearing its motion. Needs the physics module; the body keeps
     * its rotation.
     */
    teleport(handle: EntityHandle, x: number, y: number, z: number): void {
        this.assertAlive();
        if (![x, y, z].every((v) => Number.isFinite(v) && Math.abs(v) <= MAX_TELEPORT_COORD)) throw new RangeError(`teleport position must be finite and within +-${MAX_TELEPORT_COORD}`);
        if (!this.ids.isLive(handle)) throw new Error(`stale handle {id: ${handle.id}, gen: ${handle.gen}}`);
        if (!this.enqueue({ k: 'teleport', d: { id: handle.id, gen: handle.gen, x, y, z } })) throw new Error('CommandRing full');
    }

    /**
     * Pin two bodies together with a joint, using their poses at the time the command runs. At least one body must be
     * dynamic. A joint disappears when either body is destroyed; a seek restores the joints of the target tick.
     */
    createJoint(o: JointOptions): JointHandle {
        this.assertAlive();
        const finite3 = (v: readonly number[]) => v.length === 3 && v.every(Number.isFinite);
        if (!Number.isInteger(o.kind) || o.kind < JOINT_BALL || o.kind > JOINT_DISTANCE) throw new RangeError(`unknown joint kind ${o.kind}`);
        for (const h of [o.a, o.b]) if (!this.ids.isLive(h)) throw new Error(`stale handle {id: ${h.id}, gen: ${h.gen}}`);
        if (o.a.id === o.b.id) throw new RangeError('a joint needs two different bodies');
        if (!finite3(o.anchor) || o.anchor.some((v) => Math.abs(v) > MAX_TELEPORT_COORD)) throw new RangeError(`joint anchor must be finite and within +-${MAX_TELEPORT_COORD}`);
        const axial = o.kind === JOINT_HINGE || o.kind === JOINT_SLIDER;
        const distance = o.kind === JOINT_DISTANCE;
        if (!distance && o.anchorB) throw new RangeError('only a distance joint takes a second anchor');
        if (distance && o.axis) throw new RangeError('a distance joint is set by its two anchors, not an axis');
        if (distance) {
            const b = o.anchorB;
            if (!b || !finite3(b) || b.some((v) => Math.abs(v) > MAX_TELEPORT_COORD)) throw new RangeError(`a distance joint needs a second anchor, finite and within +-${MAX_TELEPORT_COORD}`);
            // The runtime squares the 32-bit differences, so the rod length is judged the same way here.
            const sq = o.anchor.reduce((sum, v, k) => Math.fround(sum + Math.fround(Math.fround(Math.fround(v) - Math.fround(b[k])) ** 2)), 0);
            if (!(sq >= 1e-6)) throw new RangeError('the two anchors of a distance joint must be at least 1 mm apart');
        }
        const axis = distance ? o.anchorB as readonly [number, number, number] : o.axis ?? [0, 0, 0];
        if (!finite3(axis)) throw new RangeError('joint axis must be finite');
        if (axial && !(Math.hypot(...axis) > 1e-6 && Number.isFinite(axis.reduce((sum, v) => Math.fround(sum + Math.fround(Math.fround(v) ** 2)), 0)))) throw new RangeError('hinge and slider joints need a finite, non-zero axis');
        if (o.motor && !axial) throw new RangeError('only hinge and slider joints take a motor');
        if (o.limits && !axial && !distance) throw new RangeError('only hinge, slider and distance joints take limits');
        if (o.limits) {
            const bound = o.kind === JOINT_HINGE ? MAX_ANGLE_LIMIT : MAX_LINEAR_LIMIT;
            const [lo, hi] = o.limits;
            if (!Number.isFinite(lo) || !Number.isFinite(hi) || lo > 0 || hi < 0 || Math.abs(lo) > bound || Math.abs(hi) > bound) throw new RangeError(`joint limits must satisfy lo <= 0 <= hi within +-${bound}; the angle or slide is measured from the pose at creation`);
        }
        if (o.motor) this.assertMotor(o.motor.velocity, o.motor.maxForce);
        if (this.nextJoint >= MAX_JOINT_ID) throw new RangeError(`joint limit reached (${MAX_JOINT_ID})`);
        const id = this.nextJoint;
        const desc = { id, kind: o.kind, a: o.a.id, genA: o.a.gen, b: o.b.id, genB: o.b.gen, anchor: o.anchor, axis, limits: o.limits, motorVelocity: o.motor?.velocity, motorMax: o.motor?.maxForce };
        if (!this.enqueue({ k: 'jointCreate', d: desc })) throw new Error('CommandRing full');
        this.nextJoint++;
        return { id };
    }

    private assertMotor(velocity: number, maxForce: number): void {
        if (!Number.isFinite(velocity) || Math.abs(velocity) > MAX_MOTOR_SPEED) throw new RangeError(`motor velocity must be finite and within +-${MAX_MOTOR_SPEED}`);
        if (!Number.isFinite(maxForce) || maxForce < 0 || maxForce > MAX_MOTOR_FORCE) throw new RangeError(`motor maxForce must be within 0..${MAX_MOTOR_FORCE}`);
    }

    /** Change a hinge or slider motor target; `maxForce` 0 switches it off. A handle whose joint is gone is dropped by the runtime and counted as stale. */
    setJointMotor(joint: JointHandle, velocity: number, maxForce: number): void {
        this.assertAlive();
        this.assertMotor(velocity, maxForce);
        if (!this.enqueue({ k: 'jointMotor', id: joint.id, velocity, maxForce })) throw new Error('CommandRing full');
    }

    /**
     * Glide static bodies to telemetry samples: each body moves at constant speed and arrives after `ticks` ticks, pushing and
     * carrying dynamic bodies on the way. A new sample for a body replaces its unfinished one. Every sample is checked before
     * any is queued, and the batch is queued whole or not at all.
     */
    moveKinematic(samples: readonly KinematicTarget[]): void {
        this.assertAlive();
        for (const s of samples) {
            if (!this.ids.isLive(s.handle)) throw new Error(`stale handle {id: ${s.handle.id}, gen: ${s.handle.gen}}`);
            if (![s.x, s.y, s.z].every((v) => Number.isFinite(v) && Math.abs(v) <= MAX_TELEPORT_COORD)) throw new RangeError(`kinematic target must be finite and within +-${MAX_TELEPORT_COORD}`);
            if (!Number.isInteger(s.ticks) || s.ticks < 1 || s.ticks > MAX_KIN_TICKS) throw new RangeError(`kinematic ticks must be an integer in 1..${MAX_KIN_TICKS}, got ${s.ticks}`);
        }
        const slots = Math.ceil(samples.length / KIN_SAMPLES_PER_SLOT);
        if (slots > this.ringCapacity - this.ring.pendingCount + this.maxDeferred - this.deferredCommands) {
            this.rejected += slots;
            throw new Error('CommandRing full: not enough room for this batch');
        }
        for (let i = 0; i < samples.length; i += KIN_SAMPLES_PER_SLOT) {
            const chunk = samples.slice(i, i + KIN_SAMPLES_PER_SLOT).map((s) => ({ id: s.handle.id, gen: s.handle.gen, x: s.x, y: s.y, z: s.z, ticks: s.ticks }));
            if (!this.enqueue({ k: 'kinematic', samples: chunk })) throw new Error('CommandRing full');
        }
    }

    private checkQuery(point: readonly number[], what: string): void {
        if (point.length !== 3 || !point.every((v) => Number.isFinite(v) && Math.abs(v) <= MAX_TELEPORT_COORD)) throw new RangeError(`${what} must be three finite numbers within +-${MAX_TELEPORT_COORD}`);
    }

    /**
     * Ask what a ray hits. The answer is one row, read later with `readQueryResults()` under the returned id: the nearest body, or a
     * miss. The runtime answers while it applies this command, from the poses the last step left, so the result is there after the
     * next `step()` (or, on a paused world, after the runtime next drains its ring). A query changes nothing and is not recorded.
     */
    raycast(o: RaycastOptions): number {
        this.assertAlive();
        this.checkQuery(o.origin, 'ray origin');
        // The runtime squares the f32 components, so a direction whose square overflows f32 would be dropped there.
        const lenSq = o.direction.reduce((sum, v) => Math.fround(sum + Math.fround(Math.fround(v) ** 2)), 0);
        if (o.direction.length !== 3 || !o.direction.every(Number.isFinite) || !Number.isFinite(lenSq) || !(lenSq > 1e-12)) throw new RangeError('ray direction must be three finite numbers whose squares fit in 32 bits, and not zero');
        if (!Number.isFinite(o.maxDistance) || !(Math.fround(o.maxDistance) > 0) || o.maxDistance > MAX_QUERY_RANGE) throw new RangeError(`ray length must be in (0, ${MAX_QUERY_RANGE}]`);
        const id = this.nextQuery;
        if (!this.enqueue({ k: 'raycast', d: { id, origin: o.origin, dir: o.direction, maxDistance: o.maxDistance, ignore: o.ignore?.id ?? NO_ENTITY } })) throw new Error('CommandRing full');
        this.nextQuery = (this.nextQuery + 1) >>> 0;
        return id;
    }

    /** Ask which bodies a sphere overlaps: up to 64 rows, lowest entity ids first, then an `overlap-more` row if there were more. */
    overlapSphere(o: OverlapSphereOptions): number {
        this.assertAlive();
        this.checkQuery(o.center, 'sphere center');
        if (!Number.isFinite(o.radius) || !(Math.fround(o.radius) > 0) || o.radius > MAX_QUERY_RANGE) throw new RangeError(`sphere radius must be in (0, ${MAX_QUERY_RANGE}]`);
        const id = this.nextQuery;
        if (!this.enqueue({ k: 'overlap', d: { id, center: o.center, radius: o.radius, ignore: o.ignore?.id ?? NO_ENTITY } })) throw new Error('CommandRing full');
        this.nextQuery = (this.nextQuery + 1) >>> 0;
        return id;
    }

    /**
     * Rows answered since the last call, oldest first. `lost` counts rows the runtime overwrote before they were read (the buffer
     * holds 4096 rows). A query that is still queued has no row yet.
     */
    readQueryResults(): { results: QueryResult[]; lost: number } {
        this.assertAlive();
        const { rows, next, lost } = this.transforms.queryRows(this.queryTail);
        this.queryTail = next;
        const f = new Float32Array(rows.buffer);
        const results: QueryResult[] = [];
        for (let o = 0; o < rows.length; o += QUERY_ROW_WORDS) {
            const query = rows[o], kind = rows[o + 1];
            const vec = (at: number): [number, number, number] => [f[o + at], f[o + at + 1], f[o + at + 2]];
            if (kind === 0) results.push({ query, kind: 'hit', entity: rows[o + 2], distance: f[o + 3], point: vec(4), normal: vec(7) });
            else if (kind === 1) results.push({ query, kind: 'miss' });
            else if (kind === 2) results.push({ query, kind: 'overlap', entity: rows[o + 2], depth: f[o + 3], point: vec(4), normal: vec(7) });
            else if (kind === 3) results.push({ query, kind: 'overlap-none' });
            else if (kind === 4) results.push({ query, kind: 'overlap-more' });
            else throw new Error(`unknown query result kind ${kind}`);
        }
        return { results, lost };
    }

    /**
     * Make a rope of `count` particles from `from` to `to`, joined by distance links. Returns the group id, the index of its first particle in `readParticles()` and its size.
     * `first` assumes every earlier particle command was accepted and no seek went back before one; a refused command (counted in `malformedDropped`) shifts later groups.
     */
    createChain(o: { from: readonly [number, number, number]; to: readonly [number, number, number]; count: number; pinFirst?: boolean; pinLast?: boolean; radius?: number; invMass?: number; stiffness?: number }): ParticleGroup {
        this.assertAlive();
        const d = { from: o.from, to: o.to, count: o.count, pinFirst: o.pinFirst ?? false, pinLast: o.pinLast ?? false, radius: o.radius ?? 0.02, invMass: o.invMass ?? 1, stiffness: o.stiffness ?? 1 };
        if (!Number.isInteger(d.count) || d.count < 2 || d.count > MAX_CHAIN) throw new RangeError(`a chain has 2..${MAX_CHAIN} particles, got ${d.count}`);
        this.assertParticleBody(d.from, d.to, d.radius, d.invMass, d.stiffness);
        const span = Math.hypot(Math.fround(d.to[0]) - Math.fround(d.from[0]), Math.fround(d.to[1]) - Math.fround(d.from[1]), Math.fround(d.to[2]) - Math.fround(d.from[2]));
        if (!(span > 1e-4)) throw new RangeError('the two ends of a chain must be apart');
        return this.queueGroup(d.count, (id) => ({ k: 'chain', d: { id, ...d } }));
    }

    /** A `nu` by `nv` sheet (2..64 a side) spanned by edge vectors `u` and `v`. `pins`: bits 0-3 pin the corners, bit 4 the whole first row. */
    createCloth(o: { origin: readonly [number, number, number]; u: readonly [number, number, number]; v: readonly [number, number, number]; nu: number; nv: number; pins?: number; radius?: number; invMass?: number; stiffness?: number; bendStiffness?: number }): ParticleGroup {
        this.assertAlive();
        const d = { origin: o.origin, u: o.u, v: o.v, nu: o.nu, nv: o.nv, pins: o.pins ?? 0, radius: o.radius ?? 0.05, invMass: o.invMass ?? 1, stiffness: o.stiffness ?? 1, bendStiffness: o.bendStiffness ?? 0.3 };
        for (const n of [d.nu, d.nv]) if (!Number.isInteger(n) || n < 2 || n > MAX_CLOTH_SIDE) throw new RangeError(`a cloth is 2..${MAX_CLOTH_SIDE} particles a side, got ${n}`);
        if (!Number.isInteger(d.pins) || d.pins < 0 || d.pins > 31) throw new RangeError('cloth pins is a 5 bit mask');
        if (!Number.isFinite(d.bendStiffness) || d.bendStiffness < 0 || d.bendStiffness > 1) throw new RangeError('bendStiffness must be within 0..1');
        const [u, v] = [d.u.map(Math.fround), d.v.map(Math.fround)];
        const area = Math.hypot(Math.fround(u[1] * v[2] - u[2] * v[1]), Math.fround(u[2] * v[0] - u[0] * v[2]), Math.fround(u[0] * v[1] - u[1] * v[0]));
        if (!(area > 1e-4)) throw new RangeError('the edge vectors u and v of a cloth must span an area');
        this.assertParticleBody(d.origin, d.u, d.radius, d.invMass, d.stiffness);
        this.assertParticleBody(d.v, d.v, d.radius, d.invMass, d.stiffness);
        return this.queueGroup(d.nu * d.nv, (id) => ({ k: 'cloth', d: { id, ...d } }));
    }

    /** An axis-aligned lattice of `n` particles (2..12 each) filling the box from `origin` to `origin + size`. */
    createSoftBlock(o: { origin: readonly [number, number, number]; size: readonly [number, number, number]; n: readonly [number, number, number]; radius?: number; invMass?: number; stiffness?: number }): ParticleGroup {
        this.assertAlive();
        const d = { origin: o.origin, size: o.size, n: o.n, radius: o.radius ?? 0.05, invMass: o.invMass ?? 1, stiffness: o.stiffness ?? 1 };
        if (d.n.length !== 3) throw new RangeError('n is three counts, one per axis');
        for (const n of d.n) if (!Number.isInteger(n) || n < 2 || n > MAX_SOFT_SIDE) throw new RangeError(`a soft block is 2..${MAX_SOFT_SIDE} particles along each axis, got ${n}`);
        if (d.size.length !== 3 || !d.size.every((s) => Number.isFinite(s) && Math.fround(s) > Math.fround(1e-3))) throw new RangeError('a soft block needs a size above 1 mm on every axis');
        this.assertParticleBody(d.origin, d.size, d.radius, d.invMass, d.stiffness);
        return this.queueGroup(d.n[0] * d.n[1] * d.n[2], (id) => ({ k: 'soft', d: { id, ...d } }));
    }

    /** Hold particle `particle` (an index into `readParticles()`) on `body` at `local` in the body's frame. A particle has one attachment. */
    attachParticle(particle: number, body: EntityHandle, local: readonly [number, number, number]): void {
        this.assertAlive();
        if (!Number.isInteger(particle) || particle < 0 || particle >= MAX_PARTICLES) throw new RangeError(`particle must be an index below ${MAX_PARTICLES}, got ${particle}`);
        if (!local.every((v) => Number.isFinite(v) && Math.abs(v) <= MAX_TELEPORT_COORD)) throw new RangeError('the attachment offset must be finite and within range');
        if (!this.enqueue({ k: 'attach', d: { particle, body: body.id, gen: body.gen, local } })) throw new Error('CommandRing full');
    }

    detachParticle(particle: number): void {
        this.assertAlive();
        if (!Number.isInteger(particle) || particle < 0 || particle >= MAX_PARTICLES) throw new RangeError(`particle must be an index below ${MAX_PARTICLES}, got ${particle}`);
        if (!this.enqueue({ k: 'detach', particle })) throw new Error('CommandRing full');
    }

    /** `[x, y, z, radius]` per particle as of the last published tick, in one read; the particle index is the row number. */
    readParticles(): Float32Array {
        this.assertAlive();
        return this.transforms.particles();
    }

    private assertParticleBody(a: readonly number[], b: readonly number[], radius: number, invMass: number, stiffness: number): void {
        if (a.length !== 3 || b.length !== 3) throw new RangeError('particle positions and sizes are 3-vectors');
        // The core works in f32, so the limits are judged on the f32 values it will see.
        radius = Math.fround(radius); invMass = Math.fround(invMass); stiffness = Math.fround(stiffness);
        if (![...a, ...b].every((v) => Number.isFinite(v) && Math.abs(v) <= MAX_TELEPORT_COORD)) throw new RangeError('particle positions and sizes must be finite and within range');
        if (!(radius >= 1e-3 && radius <= 100)) throw new RangeError(`particle radius must be within 0.001..100, got ${radius}`);
        if (!(invMass >= 0 && invMass <= 1e6)) throw new RangeError(`particle invMass must be within 0..1e6, got ${invMass}`);
        if (!(stiffness > 0 && stiffness <= 1)) throw new RangeError(`stiffness must be within (0, 1], got ${stiffness}`);
    }

    private queueGroup(count: number, make: (id: number) => Queued): ParticleGroup {
        if (this.nextGroup >= MAX_GROUP_ID) throw new RangeError('group id limit reached');
        if (this.nextGroup >= MAX_PARTICLE_GROUPS) throw new RangeError(`a world holds at most ${MAX_PARTICLE_GROUPS} particle groups`);
        if (this.nextParticle + count > MAX_PARTICLES) throw new RangeError(`a world holds at most ${MAX_PARTICLES} particles`);
        const id = this.nextGroup;
        if (!this.enqueue(make(id))) throw new Error('CommandRing full');
        this.nextGroup++;
        const first = this.nextParticle;
        this.nextParticle += count;
        return { id, first, count };
    }

    /**
     * Position and speed of every hinge, slider and distance joint as of the last tick the runtime published (a seek republishes them for the sought
     * tick). A controller reads these after `step()`, decides, and sends motor commands that take effect on the next step.
     */
    readJointStates(): JointState[] {
        this.assertAlive();
        return this.transforms.jointStates().map((r) => ({ joint: r.id, kind: r.kind, position: r.position, velocity: r.velocity }));
    }

    /**
     * One control tick: read the joint states once, run position control for each drive and queue the motor commands (all are validated first, so a bad value queues nothing; a full ring can still stop it part way). A drive whose
     * joint has no state yet (the joint command has not been applied, or the joint is gone) is skipped and its id returned.
     */
    driveJoints(drives: readonly JointDrive[]): number[] {
        this.assertAlive();
        const states = new Map(this.readJointStates().map((s) => [s.joint, s]));
        const commands = [];
        const skipped: number[] = [];
        for (const d of drives) {
            const state = states.get(d.joint.id);
            if (!state) { skipped.push(d.joint.id); continue; }
            // A distance joint publishes a state but has no motor: the runtime would refuse the command and count it malformed every tick.
            if (state.kind !== JOINT_HINGE && state.kind !== JOINT_SLIDER) throw new RangeError(`joint ${d.joint.id} has no motor (only hinge and slider joints can be driven)`);
            commands.push({ joint: d.joint, ...positionControl(state, d.target, d.gains) });
        }
        for (const c of commands) this.assertMotor(c.velocity, c.maxForce);
        for (const c of commands) this.setJointMotor(c.joint, c.velocity, c.maxForce);
        return skipped;
    }

    destroyJoint(joint: JointHandle): void {
        this.assertAlive();
        if (!this.enqueue({ k: 'jointDestroy', id: joint.id })) throw new Error('CommandRing full');
    }

    /**
     * Start or stop recording. While on, the runtime keeps keyframes and a command log so `seek` can jump to any tick in
     * `recording.start..recording.end`. Needs the physics module.
     */
    record(on: boolean): void {
        this.assertAlive();
        if (!this.enqueue({ k: 'record', on })) throw new Error('CommandRing full');
    }

    /**
     * Stop or resume the simulation clock. A paused runtime still applies commands (so `seek` and `teleport` work) but
     * runs no physics step and publishes no new tick.
     */
    pause(on: boolean): void {
        this.assertAlive();
        if (!this.enqueue({ k: 'pause', on })) throw new Error('CommandRing full');
    }

    /**
     * Remember the world as it is when the runtime reaches this command; `reset()` returns to it. Replaces an earlier checkpoint.
     * Needs the physics module; refused (counted in `runtimeCounters.malformedDropped`) without it.
     */
    checkpoint(): void {
        this.assertAlive();
        if (!this.enqueue({ k: 'checkpoint' })) throw new Error('CommandRing full');
    }

    /**
     * Return the world to its checkpoint without advancing it. Bodies, joints and entity ids go back to the checkpoint; the tick
     * counter, a larger capacity and the counters do not. Commands queued before this one in the same step have their effect on the
     * world undone, commands after it act on the restored world. Refused (counted in `runtimeCounters.malformedDropped`) with no checkpoint
     * or while a recording runs. Entity handles are re-synced as after `seek`: those for entities that did not exist at the
     * checkpoint go stale.
     */
    reset(): void {
        this.assertAlive();
        if (!this.enqueue({ k: 'reset' })) throw new Error('CommandRing full');
    }

    /**
     * Jump to a recorded tick: the world becomes the recorded state at that tick, before any command applied there, so an
     * edit made while paused at that tick is discarded (a live step commits it). Takes effect when the runtime processes
     * the command. Entity handles are re-synced with the runtime on the first read or command after the runtime has
     * drained every queued command: handles for entities that did not exist at that tick go stale and their ids come
     * back. Pause first, because a running world forks the timeline at its next step. A tick outside
     * `recording.start..recording.end` counts in `runtimeCounters.malformedDropped`.
     */
    seek(tick: number): void {
        this.assertAlive();
        if (!Number.isInteger(tick) || tick < 0 || tick > 0xffffffff) throw new RangeError(`tick must be an integer in [0, 2^32), got ${tick}`);
        if (!this.enqueue({ k: 'seek', tick })) throw new Error('CommandRing full');
    }

    /** Recording range, paused flag, tick and state hash; `null` if the runtime kept rewriting them through every read attempt. */
    get recording(): RecordingState | null {
        return this.transforms.recording();
    }

    /**
     * Ask the runtime to reallocate its buffers for `capacity` entities. Takes effect when the runtime
     * processes the command; `onBufferGrowth` fires and `capacity` rises once this thread observes it.
     */
    requestGrow(capacity: number): void {
        this.assertAlive();
        const floor = Math.max(this.capacity, this.pendingGrow);
        if (!Number.isInteger(capacity) || capacity <= floor || capacity > MAX_CAPACITY) {
            throw new RangeError(`capacity must be an integer in (${floor}, ${MAX_CAPACITY}], got ${capacity}`);
        }
        if (!this.enqueue({ k: 'grow', capacity })) throw new Error('CommandRing full');
        this.pendingGrow = capacity;
    }

    /**
     * Upload a convex hull from flat x,y,z points (4 to 32 vertices, |coordinate| <= 1e4) and return its id for
     * `spawn({ shape: SHAPE_HULL, hull })`. The runtime recentres the hull on its volume centroid, so the body
     * origin is that centroid. Invalid geometry (for example coplanar points) is rejected by the runtime and counted
     * in `runtimeCounters.malformedDropped`; spawns that reference it are dropped the same way.
     */
    registerHull(points: ArrayLike<number>): number {
        this.assertAlive();
        const count = points.length / 3;
        if (!Number.isInteger(count) || count < 4 || count > MAX_HULL_VERTS) {
            throw new RangeError(`a hull needs 4 to ${MAX_HULL_VERTS} vertices as x,y,z triples, got ${points.length} numbers`);
        }
        for (let i = 0; i < points.length; i++) {
            if (!Number.isFinite(points[i]) || Math.abs(points[i]) > MAX_HULL_COORD) throw new RangeError(`hull coordinate ${i} is not finite or exceeds ${MAX_HULL_COORD}`);
        }
        if (this.nextHull >= MAX_HULLS) throw new RangeError(`hull limit reached (${MAX_HULLS})`);
        const chunks = Math.ceil(count / HULL_VERTS_PER_SLOT);
        const room = this.ringCapacity - this.ring.pendingCount + this.maxDeferred - this.deferredCommands;
        if (chunks + 1 > room) throw new Error('CommandRing full: not enough room to upload this hull');
        const hull = this.nextHull++;
        for (let c = 0; c < chunks; c++) {
            const coords = Array.from(points).slice(c * HULL_VERTS_PER_SLOT * 3, (c + 1) * HULL_VERTS_PER_SLOT * 3);
            this.enqueue({ k: 'hullVerts', hull, coords });
        }
        this.enqueue({ k: 'hullCommit', hull, count });
        return hull;
    }

    /** Set world gravity through the command ring (values must be finite and within +-1000). */
    setGravity(x: number, y: number, z: number): void {
        this.assertAlive();
        if (![x, y, z].every((v) => Number.isFinite(v) && Math.abs(v) <= 1000)) throw new RangeError('gravity must be finite and within +-1000');
        if (!this.enqueue({ k: 'gravity', v: [x, y, z] })) throw new Error('CommandRing full');
    }

    /**
     * Turn the debug snapshot on or off through the command ring. While off the runtime copies nothing; read it with
     * `InspectorView`. Safe to call repeatedly.
     */
    setInspector(on: boolean): void {
        this.assertAlive();
        if (!this.enqueue({ k: 'inspect', on })) throw new Error('CommandRing full');
    }

    /** Valid until the next call to `readTransforms()` (and, in fallback mode, the next `step()`). */
    readTransforms(): Float32Array {
        if (this.fatal) throw new Error(`world unavailable: ${this.fatal}`);
        this.syncLayout();
        this.flush();
        const buf = this.transforms.read();
        if (this.transforms.epoch !== this.knownEpoch) this.applyEpoch();
        return buf;
    }

    get runtimeCounters(): { ticks: number; staleDropped: number; malformedDropped: number } {
        return this.transforms.counters();
    }

    bucketHistogram(): Uint32Array {
        return this.transforms.bucketCounts();
    }

    get runtimeStats(): { ringPending: number; droppedContacts: number; workerOverruns: number; contacts: number } {
        return this.transforms.stats();
    }

    get capacity(): number { return this.ids.capacityNow; }
    get ringSize(): number { return this.ringCapacity; }
    get pendingCommands(): number { return this.ring.pendingCount + this.deferred.length - this.deferredHead; }
    get deferredCommands(): number { return this.deferred.length - this.deferredHead; }
    get totalDeferred(): number { return this.deferredTotal; }
    get backpressureDrops(): number { return this.rejected; }
    get growthEvents(): number { return this.growths; }
    get allocatedEntities(): number { return this.ids.allocated; }
}
