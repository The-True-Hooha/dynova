import { test, expect } from '@playwright/test';

test('convex hull bodies uploaded through the ring settle, a degenerate hull is rejected and its spawn dropped', async ({ page }) => {
    await page.goto('/hull.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 60_000 });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error).toBeUndefined();
    expect(r.finite).toBe(true);
    expect(r.topY).toBeGreaterThan(3.3);
    expect(r.topY).toBeLessThan(3.6);
    expect(Math.abs(r.topX)).toBeLessThan(0.1);
    expect(r.tetra[1]).toBeGreaterThan(0.4);
    expect(r.tetra[1]).toBeLessThan(1.2);
    expect(r.sleeping).toBe(11);
    expect(r.malformed).toBe(2);
    expect(r.rejected[6]).toBe(0);
});
