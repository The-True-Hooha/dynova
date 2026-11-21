import { test, expect } from '@playwright/test';

test('water in a basin holds a floating cube at about its analytic draft while sand rests outside, all in the browser', async ({ page }, info) => {
    test.setTimeout(180_000);
    await page.goto('/fluid.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 150_000 });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error).toBeUndefined();
    expect(r.particles).toBe(3072 + 512);
    expect(r.malformed).toBe(0);
    expect(r.leaked).toBe(0);
    // A cube at half the density of water floats with half its height under the surface (0.15 m); sunk to the floor it would read 0.3 and resting on top 0.
    expect(r.draft).toBeGreaterThan(0.12);
    expect(r.draft).toBeLessThan(0.17);
    expect(r.sandLowest).toBeGreaterThan(0.02);
    expect(r.sandWidth).toBeLessThan(0.5);
    info.annotations.push({ type: 'timing', description: `${r.particles} particles, median step ${r.medianStepMs.toFixed(2)} ms, p95 ${r.p95StepMs.toFixed(2)} ms, draft ${r.draft.toFixed(3)}, sand width ${r.sandWidth.toFixed(2)}` });
});
