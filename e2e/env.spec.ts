import { test, expect } from '@playwright/test';

test('an environment over four real workers replays an environment on one thread bit for bit, resets included, and draws the same random numbers', async ({ page }, info) => {
    await page.goto('/env.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 120_000 });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error).toBeUndefined();
    expect(r.firstDifferent, 'index of the first step where the two environments differ').toBe(-1);
    expect(r.terminated, 'some episodes end by terminating').toBeGreaterThan(0);
    expect(r.truncated, 'and some by running out of steps').toBeGreaterThan(0);
    expect(r.resets, 'every ended episode but the last of each world is followed by a reset').toBeGreaterThanOrEqual(r.terminated + r.truncated - 12);
    expect(r.remoteResets).toBe(r.resets);
    expect(r.rng, 'mulberry32 pins from the Node tests').toEqual([0.6270739405881613, 0.26642920868471265]);
    info.annotations.push({ type: 'timing', description: `256 worlds, 60 steps: 1 worker ${Math.round(r.rate1Ms)} ms, 4 workers ${Math.round(r.rate4Ms)} ms, ${r.cores} cores` });
});
