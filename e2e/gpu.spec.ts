import { test, expect } from '@playwright/test';

// Chromium reaches WebGPU only through SwiftShader (a CPU rasteriser) behind this flag; WebKit uses the machine's GPU.
test.use({ launchOptions: { args: ['--enable-unsafe-webgpu'] } });

// The GPU path is not bit-identical to the CPU one (the CPU path is the reference and its hashes do not move): it is held to these bounds. They are absolute, so they
// hold for scenes within a few metres of the origin; far from it f32 cancellation in the anchor error grows them (a pivot at 50 m reached 5e-4 m in 400 ticks).
const TOLERANCE = { angle: 1e-4, velocity: 1e-3, position: 1e-4, rotation: 1e-4 };

for (const [name, query] of [['a driven arm', 'n=64&amp=3'], ['an arm driven into its limits', 'n=64&amp=9'], ['an arm kicked at 100x and driven at 30 rad/s (anchor errors far past the 4 m/s bias clamp)', 'n=64&amp=30&kick=100'], ['a skewed arm (off-origin pivot, tilted axis, bent gravity, heavier link, asymmetric limits)', 'n=64&amp=3&scene=skew']]) {
    test(`${name} on the GPU stays within tolerance of the CPU worlds`, async ({ page }, info) => {
        test.setTimeout(180_000);
        await page.goto(`/gpu.html?${query}`);
        await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 150_000 });
        const r = await page.evaluate(() => (window as any).__result);
        expect(r.error).toBeUndefined();
        test.skip(r.skipped !== undefined, r.skipped);
        info.annotations.push({ type: 'gpu', description: `${r.adapter}: ${JSON.stringify(r.checks[r.checks.length - 1])}` });
        expect(r.checks.length).toBe(5);
        for (const c of r.checks) {
            for (const key of ['angle', 'velocity', 'position', 'rotation'] as const) expect(c[key], `${key} at tick ${c.tick}`).toBeLessThan(TOLERANCE[key]);
        }
        // The scene has to reach what the kernel must get right.
        expect(r.widest).toBeGreaterThan(query.includes('skew') ? 0.5 : 1.5);
        expect(r.guards).toEqual({ concurrentReadsAgree: true, readIsASnapshot: true, nanMotor: true, hugeMotor: true, nanKick: true, staticLink: true, usedAfterDestroy: true });
        if (query.includes('amp=9') || query.includes('kick=100')) expect(r.atLimit).toBeGreaterThan(20);
    });
}
