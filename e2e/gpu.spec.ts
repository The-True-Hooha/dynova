import { test, expect } from '@playwright/test';

// Chromium reaches WebGPU only through SwiftShader (a CPU rasteriser) behind this flag; WebKit uses the machine's GPU.
test.use({ launchOptions: { args: ['--enable-unsafe-webgpu'] } });

// The GPU path is not bit-identical to the CPU one (the CPU path is the reference and its hashes do not move): it is held to these bounds. They are absolute, so they
// hold for scenes within a few metres of the origin; far from it f32 cancellation in the anchor error grows them (a pivot at 50 m reached 5e-4 m in 400 ticks).
const TOLERANCE = { angle: 1e-4, velocity: 1e-3, position: 1e-4, rotation: 1e-4 };

for (const [name, query] of [['a driven arm', 'n=64&amp=3'], ['an arm driven into its limits', 'n=64&amp=9'], ['an arm kicked at 100x and driven at 30 rad/s (anchor errors far past the 4 m/s bias clamp)', 'n=64&amp=30&kick=100'], ['a slider driven into its limits (offset in metres, bodies held in orientation)', 'n=64&joint=slider&amp=6&kick=10'], ['a slider with the anchor off the link centre (every orientation row carries torque)', 'n=64&joint=slider&amp=4&kick=10&lever=1'], ['an arm with no motor that swings itself to rest and falls asleep when the core says so', 'n=64&sleep=1&ticks=600&kick=1'], ['a skewed arm that sleeps', 'n=64&sleep=1&ticks=800&scene=skew&kick=1'], ['a slider that sleeps', 'n=64&sleep=1&ticks=600&joint=slider&kick=3'], ['a slider coasting at 0.05 to 0.4 m/s that falls asleep once slower than the sleep speed', 'n=64&sleep=1&ticks=300&coast=1'], ['a skewed slider (off-origin pivot, tilted axis, bent gravity, heavier link, asymmetric limits)', 'n=64&joint=slider&scene=skew&amp=4&kick=5'], ['a skewed arm (off-origin pivot, tilted axis, bent gravity, heavier link, asymmetric limits)', 'n=64&amp=3&scene=skew'],
    ['a driven arm whose tip meets a floor at an angle (contact against the motor)', 'n=64&floor=tip&amp=3'],
    ['a slider driven onto a floor flat (four contact points)', 'n=64&floor=flat&joint=slider&amp=6&kick=10'],
    ['an arm kicked at up to 40 m/s into a floor (swept test)', 'n=64&floor=tip&amp=3&kick=10'],
    ['a slider driven at up to 40 m/s at a floor thinner than one substep of travel (the swept test)', 'n=64&floor=thin&joint=slider&amp=3&kick=10'],
    ['a skewed arm against a floor rotated about two axes', 'n=64&floor=twist&scene=skew&amp=3'],
    ['an arm bouncing off a tilted, bouncy floor', 'n=64&floor=slope&amp=3&kick=3&bounce=1'],
    ['an arm that comes to rest on a floor and sleeps', 'n=64&floor=tip&sleep=1&ticks=600&kick=1'],
    ['an arm that comes to rest on a tilted floor and sleeps', 'n=64&floor=slope&sleep=1&ticks=600&kick=2'],
    ['an arm driven against a post', 'n=64&floor=block&amp=3'],
    ['a rod from the pivot to a point off the link centre, kicked so the link spins and swings', 'n=64&joint=rod&kick=2'],
    ['a rope that goes slack, hits its short stop and snaps taut', 'n=64&joint=rope&kick=1'],
    ['a rod whose link turns about its anchor point and dips a corner into a floor', 'n=64&joint=rod&floor=deck&kick=2'],
    ['a rope whose slack link falls back onto a floor', 'n=64&joint=rope&floor=deck&kick=0.85']]) {
    test(`${name} on the GPU stays within tolerance of the CPU worlds`, async ({ page }, info) => {
        test.setTimeout(180_000);
        await page.goto(`/gpu.html?${query}`);
        await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 150_000 });
        const r = await page.evaluate(() => (window as any).__result);
        expect(r.error).toBeUndefined();
        test.skip(r.skipped !== undefined, r.skipped);
        info.annotations.push({ type: 'gpu', description: `${r.adapter}: ${JSON.stringify(r.checks[r.checks.length - 1])}` });
        const sleeps = query.includes('sleep=1');
        const floored = query.includes('floor=');
        // The floor scenes have to reach the contact path: a link corner inside the floor in some world.
        if (floored) expect(r.touched, 'worlds that touched the floor').toBeGreaterThan(16);
        expect(r.checks.length).toBe(sleeps ? 6 : 5);
        for (const c of r.checks) {
            for (const key of ['angle', 'velocity', 'position', 'rotation'] as const) expect(c[key], `${key} at tick ${c.tick}`).toBeLessThan(TOLERANCE[key]);
        }
        if (sleeps) {
            const last = r.checks[r.checks.length - 1];
            expect(last.asleep, 'every link is asleep at the end').toBe(64);
            expect(r.sleepTickMisses, 'each GPU link falls asleep in the tick the CPU one stops moving, or the one after').toBe(0);
            expect(r.checks.every((c: any) => c.gpuAsleepCpuMoving === 0), 'the GPU never sleeps a link the CPU still moves').toBe(true);
            for (const c of r.checks.slice(-2)) expect(c.cpuFrozenGpuAwake, 'and wakes none the CPU has put to sleep').toBe(0);
            // Links kicked differently fall asleep at different ticks: a sleep rule that is off by a substep shows as a position gap.
            if (query.includes('kick=1') && !query.includes('skew') && !floored) expect(r.checks.some((c: any) => c.asleep > 0 && c.asleep < 64)).toBe(true);
        } else if (query.includes('joint=r')) {
            // A rod or rope has no motor, so a link that settles on the floor may sleep: the GPU must agree with the CPU on which ones, tick by tick.
            expect(r.checks.every((c: any) => c.gpuAsleepCpuMoving === 0), 'the GPU never sleeps a link the CPU still moves').toBe(true);
        } else {
            expect(r.checks.every((c: any) => c.asleep === 0), 'a driven link never sleeps').toBe(true);
        }
        // The scene has to reach what the kernel must get right.
        if (query.includes('joint=rope')) {
            expect(r.widest, 'the rope went slack').toBeGreaterThan(0.1);
            expect(r.atLimit, 'and reached its short stop in some worlds').toBeGreaterThan(20);
        }
        if (!floored && !query.includes('joint=r')) expect(r.widest).toBeGreaterThan(sleeps ? 0.1 : query.includes('slider') ? 0.35 : query.includes('skew') ? 0.5 : 1.5);
        expect(r.guards).toEqual({ concurrentReadsAgree: true, readIsASnapshot: true, nanMotor: true, hugeMotor: true, nanKick: true, staticLink: true, resetObs: true, usedAfterDestroy: true });
        if (!sleeps && !floored && (query.includes('amp=9') || query.includes('kick=100') || query.includes('slider'))) expect(r.atLimit).toBeGreaterThan(20);
    });
}
