import { test, expect } from '@playwright/test';

test('a rope, a cloth and a soft block run in the browser and one read returns every particle', async ({ page }, info) => {
    await page.goto('/particles.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 90_000 });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error).toBeUndefined();
    expect(r.particles).toBe(11 + 441 + 64);
    expect(r.pin).toEqual([5, 5]);
    // Released horizontally, the tip is still swinging: it stays within a rope's length of the pin and below it.
    expect(r.ropeTip).toBeGreaterThan(2.9);
    expect(r.ropeTip).toBeLessThan(5);
    expect(r.clothMiddle).toBeGreaterThan(1.1);
    expect(r.clothMiddle).toBeLessThan(1.25);
    expect(r.blockLowest).toBeGreaterThan(0.1);
    expect(r.blockLowest).toBeLessThan(0.17);
    info.annotations.push({ type: 'timing', description: `516 particles, median step ${r.medianStepMs.toFixed(2)} ms` });
});
