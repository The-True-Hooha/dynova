import { test, expect } from '@playwright/test';

test('scene meshes become colliders: hull placed at its volume centroid, then dropped bodies rest on a box floor', async ({ page }) => {
    await page.goto('/colliders.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 60_000 });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error).toBeUndefined();
    expect(r.kinds).toEqual(['hull']);
    expect(r.levelKinds).toEqual(['floor:box', 'cone:hull', 'ball:sphere', 'ramp:box']);
    expect(r.failed).toEqual([]);
    expect(r.skipped).toEqual([]);
    expect(r.malformed).toBe(0);
    expect(r.centroidError).toBeLessThan(2e-3);
    expect(r.ball[1]).toBeGreaterThan(0.45);
    expect(r.ball[1]).toBeLessThan(0.56);
    expect(r.cone[1]).toBeGreaterThan(0.05);
    expect(r.cone[1]).toBeLessThan(2.0);
    expect(r.cone.every(Number.isFinite)).toBe(true);
    expect(r.sleeping).toBe(2);
});
