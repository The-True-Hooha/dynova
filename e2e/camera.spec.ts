import { test, expect } from '@playwright/test';

test('64 worlds render 64x64 depth and id images, the same bits from one thread and from four workers, and a wall reads its distance', async ({ page }, info) => {
    await page.goto('/camera.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 90_000 });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error).toBeUndefined();
    expect(r.wallError, 'a flat wall reads 6.5 m to within a millimetre at every pixel').toBeLessThan(1e-3);
    expect(r.wallIdsOk).toBe(true);
    expect(r.hits, 'the camera sees bodies in the worlds').toBeGreaterThan(64 * 64 * 64 * 0.2);
    expect(r.mismatched).toEqual([]);
    info.annotations.push({ type: 'timing', description: `${r.worlds} worlds x ${r.pixels} px: one thread ${r.oneThreadMs.toFixed(1)} ms, four workers ${r.fourWorkersMs.toFixed(1)} ms, ${r.cores} cores` });
});
