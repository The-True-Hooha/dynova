import type { World } from './World.js';

/** A seeded generator: integer arithmetic only (mulberry32), so every engine draws the same numbers from the same seed. */
export interface Rng {
    /** In [0, 1). */
    next(): number;
    range(lo: number, hi: number): number;
    /** An integer in [0, n). */
    int(n: number): number;
}

export function mulberry32(seed: number): Rng {
    let a = seed >>> 0;
    const next = () => {
        a = (a + 0x6d2b79f5) >>> 0;
        let t = Math.imul(a ^ (a >>> 15), 1 | a);
        t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
        return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
    return { next, range: (lo, hi) => lo + (hi - lo) * next(), int: (n) => Math.floor(next() * n) };
}

/** The seed of one episode of one world: a different stream for every (world, episode) pair of one base seed. */
export function episodeSeed(base: number, world: number, episode: number): number {
    let h = Math.imul(base ^ 0x9e3779b9, 0x85ebca6b);
    h ^= h >>> 13;
    h = Math.imul(h ^ world, 0xc2b2ae35);
    h ^= h >>> 16;
    h = Math.imul(h ^ episode, 0x27d4eb2f);
    h ^= h >>> 15;
    return h >>> 0;
}

/**
 * What an environment is. `C` is whatever `build` returns for a world (entity handles, joint handles, parameters), handed back to every
 * other hook for that world. Hooks read worlds through their bulk methods (`readTransforms`, `readJointStates`) and write through commands.
 */
export interface EnvSpec<C> {
    obsSize: number;
    actionSize: number;
    /** An episode that has not terminated after this many steps is truncated. */
    maxSteps: number;
    /** Spawn the scene. Runs once per world; the state it leaves is what every episode starts from. */
    build(world: World, index: number, count: number): C;
    /** Randomise the start of an episode with commands (teleports, forces, gravity, motors). The world has just returned to its checkpoint. */
    reset?(world: World, index: number, ctx: C, rng: Rng): void;
    /** Turn an action into commands for the next step. */
    act(world: World, index: number, ctx: C, action: Float32Array): void;
    /** Write this world's observation into `out`, which is `obsSize` long. */
    observe(world: World, index: number, ctx: C, out: Float32Array): void;
    reward(world: World, index: number, ctx: C, obs: Float32Array): number;
    terminated(world: World, index: number, ctx: C, obs: Float32Array): boolean;
}

/** Steps all worlds once. A `WorldBatch` steps synchronously; a `ParallelBatch` answers when every worker has. */
export interface EnvStepper {
    readonly worlds: readonly World[];
    step(): void | Promise<unknown>;
}

/**
 * The buffers of one step, one row per world. They are the environment's own and are overwritten by the next `reset` or `step`; copy what
 * must outlive it. A world whose `resetting` flag is set was reset this step: its `obs` is the first observation of a new episode, its
 * `reward` is 0, and the action given for it was ignored (Gymnasium's NEXT_STEP autoreset).
 */
export interface EnvResult {
    obs: Float32Array;
    reward: Float32Array;
    terminated: Uint8Array;
    truncated: Uint8Array;
    resetting: Uint8Array;
}

export interface EnvOptions {
    /** An integer in [0, 2^32). */
    seed?: number;
}

const MAX_DRAIN_STEPS = 10_000;

function checkSeed(seed: number): number {
    if (!Number.isInteger(seed) || seed < 0 || seed > 0xffffffff) throw new RangeError(`seed must be an integer in [0, 2^32), got ${seed}`);
    return seed;
}

const message = (err: unknown) => (err instanceof Error ? err.message : String(err));

/**
 * Many copies of one task, one per world of a batch, behind `reset` and `step` on flat typed arrays. Each world is independent: world `i`
 * of an N-world environment sees exactly what a one-world environment would for the same seed and index, whatever N is and however the
 * worlds are spread over workers.
 *
 * Once a call fails after it has queued commands (a hook threw, a value was not finite, a world failed), the environment is unusable and every
 * later call throws: the worlds hold half of a step that cannot be told apart from a whole one. A call that fails its own argument checks changes nothing.
 */
export class BatchEnv<C> {
    readonly size: number;
    /** The buffers `reset` and `step` fill and return; after `create` they hold the first observation. */
    readonly result: EnvResult;
    private readonly steps: Uint32Array;
    private readonly episode: Uint32Array;
    private readonly pendingReset: Uint8Array;
    /** Per world, the refused-command count when its reset was queued, or -1 when none is waiting. */
    private readonly refusedBefore: Int32Array;
    private seed: number;
    private busy = false;
    private broken: string | null = null;

    private constructor(private readonly stepper: EnvStepper, private readonly spec: EnvSpec<C>, seed: number, private readonly ctx: C[]) {
        const n = stepper.worlds.length;
        this.size = n;
        this.seed = seed;
        this.result = {
            obs: new Float32Array(n * spec.obsSize), reward: new Float32Array(n),
            terminated: new Uint8Array(n), truncated: new Uint8Array(n), resetting: new Uint8Array(n),
        };
        this.steps = new Uint32Array(n);
        this.episode = new Uint32Array(n);
        this.pendingReset = new Uint8Array(n);
        this.refusedBefore = new Int32Array(n).fill(-1);
    }

    get obsSize(): number { return this.spec.obsSize; }
    get actionSize(): number { return this.spec.actionSize; }

    /**
     * Builds every world, takes its checkpoint, and resets all of them. The worlds are paused while they are built, so the checkpoint is the
     * scene exactly as `build` left it however many commands that took; `build` may therefore queue more commands than a ring holds.
     */
    static async create<C>(stepper: EnvStepper, spec: EnvSpec<C>, options: EnvOptions = {}): Promise<BatchEnv<C>> {
        for (const [name, v] of [['obsSize', spec.obsSize], ['actionSize', spec.actionSize], ['maxSteps', spec.maxSteps]] as const) {
            if (!Number.isInteger(v) || v < 1) throw new RangeError(`${name} must be a positive integer, got ${v}`);
        }
        const seed = checkSeed(options.seed ?? 0);
        const { worlds } = stepper;
        if (worlds.length < 1) throw new RangeError('an environment needs at least one world');
        const refused = worlds.map((w) => w.runtimeCounters.malformedDropped);
        for (const w of worlds) w.pause(true);
        const ctx = worlds.map((w, i) => at(i, 'build', () => spec.build(w, i, worlds.length)));
        for (const w of worlds) w.checkpoint();
        await drain(stepper);
        for (const w of worlds) w.pause(false);
        await drain(stepper);
        worlds.forEach((w, i) => {
            const n = w.runtimeCounters.malformedDropped - refused[i];
            if (n > 0) throw new Error(`world ${i}: ${n} command(s) were refused while it was built (is physics enabled, is a recording running, was a spawn malformed?)`);
        });
        const env = new BatchEnv(stepper, spec, seed, ctx);
        await env.reset();
        return env;
    }

    /** Starts every world's episode again; `seed` replaces the base seed, and either way the episode counters restart so a seed replays. */
    async reset(seed?: number): Promise<EnvResult> {
        const next = seed === undefined ? this.seed : checkSeed(seed);
        this.begin();
        try {
            this.seed = next;
            this.episode.fill(0);
            this.steps.fill(0);
            this.pendingReset.fill(0);
            for (let i = 0; i < this.size; i++) this.startEpisode(i);
            await this.advance();
            this.collect(true);
            return this.result;
        } catch (err) {
            this.broken = message(err);
            throw err;
        } finally {
            this.busy = false;
        }
    }

    /** `actions` holds `actionSize` numbers per world, in world order. */
    async step(actions: Float32Array): Promise<EnvResult> {
        const { actionSize } = this.spec;
        if (actions.length !== this.size * actionSize) throw new RangeError(`expected ${this.size * actionSize} action values (${this.size} worlds x ${actionSize}), got ${actions.length}`);
        for (let k = 0; k < actions.length; k++) {
            if (!Number.isFinite(actions[k])) throw new RangeError(`action ${k} (world ${Math.floor(k / actionSize)}) is not finite: ${actions[k]}`);
        }
        this.begin();
        try {
            for (let i = 0; i < this.size; i++) {
                if (this.pendingReset[i]) this.startEpisode(i);
                else at(i, 'act', () => this.spec.act(this.stepper.worlds[i], i, this.ctx[i], actions.subarray(i * actionSize, (i + 1) * actionSize)));
            }
            await this.advance();
            this.collect(false);
            return this.result;
        } catch (err) {
            this.broken = message(err);
            throw err;
        } finally {
            this.busy = false;
        }
    }

    private begin(): void {
        if (this.broken !== null) throw new Error(`the environment is unusable after an earlier failure: ${this.broken}`);
        if (this.busy) throw new Error('a reset or step is already running');
        this.busy = true;
    }

    /**
     * Queues the reset of world `i` and runs the user's reset hook, which may only use handles `build` returned: the SDK re-syncs its entity tables
     * after the step that applies the reset, so a hook that spawns, destroys or creates joints works against tables from before it.
     */
    private startEpisode(i: number): void {
        const world = this.stepper.worlds[i];
        this.refusedBefore[i] = world.runtimeCounters.malformedDropped;
        at(i, 'reset', () => {
            world.reset();
            this.spec.reset?.(world, i, this.ctx[i], mulberry32(episodeSeed(this.seed, i, this.episode[i]++)));
        });
    }

    /** Steps the worlds, then checks that every command queued for this step was delivered, none of a reset was refused, and no world failed. */
    private async advance(): Promise<void> {
        for (const [i, w] of this.stepper.worlds.entries()) {
            if (w.deferredCommands > 0) throw new Error(`world ${i} queued more commands than its ring of ${w.ringSize} slots takes in one step; raise ringCapacity or send fewer commands`);
        }
        await this.stepper.step();
        failures(this.stepper);
        for (const [i, w] of this.stepper.worlds.entries()) {
            const before = this.refusedBefore[i];
            if (before < 0) continue;
            this.refusedBefore[i] = -1;
            const n = w.runtimeCounters.malformedDropped - before;
            if (n > 0) throw new Error(`world ${i}: ${n} command(s) were refused while it was reset (is a recording running, or was a hook command malformed?)`);
        }
    }

    private collect(allReset: boolean): void {
        const { obsSize, maxSteps } = this.spec;
        const { obs, reward, terminated, truncated, resetting } = this.result;
        for (let i = 0; i < this.size; i++) {
            const world = this.stepper.worlds[i];
            const row = obs.subarray(i * obsSize, (i + 1) * obsSize);
            at(i, 'observe', () => this.spec.observe(world, i, this.ctx[i], row));
            for (let k = 0; k < obsSize; k++) {
                if (!Number.isFinite(row[k])) throw new Error(`world ${i} observed a non-finite value at index ${k}: ${row[k]}`);
            }
            if (allReset || this.pendingReset[i]) {
                reward[i] = 0; terminated[i] = 0; truncated[i] = 0; resetting[i] = 1;
                this.steps[i] = 0;
                this.pendingReset[i] = 0;
                continue;
            }
            resetting[i] = 0;
            this.steps[i]++;
            const r = at(i, 'reward', () => this.spec.reward(world, i, this.ctx[i], row));
            if (!Number.isFinite(r)) throw new Error(`world ${i} gave a non-finite reward: ${r}`);
            reward[i] = r;
            const done = at(i, 'terminated', () => this.spec.terminated(world, i, this.ctx[i], row));
            terminated[i] = done ? 1 : 0;
            truncated[i] = !done && this.steps[i] >= maxSteps ? 1 : 0;
            this.pendingReset[i] = done || truncated[i] === 1 ? 1 : 0;
        }
    }
}

/** Runs a hook and says which world it was for. */
function at<T>(world: number, hook: string, fn: () => T): T {
    try {
        return fn();
    } catch (err) {
        throw new Error(`world ${world} ${hook}: ${message(err)}`, { cause: err });
    }
}

function failures(stepper: EnvStepper): void {
    const failed = stepper.worlds.flatMap((w, i) => (w.failure === null ? [] : [`${i}: ${w.failure}`]));
    if (failed.length) throw new Error(`worlds failed: ${failed.join('; ')}`);
}

/** Steps until every world has taken every command queued so far. */
async function drain(stepper: EnvStepper): Promise<void> {
    for (let k = 0; k < MAX_DRAIN_STEPS; k++) {
        await stepper.step();
        failures(stepper);
        if (stepper.worlds.every((w) => w.pendingCommands === 0)) return;
    }
    throw new Error(`the worlds still hold queued commands after ${MAX_DRAIN_STEPS} steps`);
}
