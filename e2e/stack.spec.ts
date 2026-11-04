import { test, expect } from '@playwright/test';

test('50-box stack settles in the browser without NaN and inside the 60fps step budget', async ({ page }) => {
    await page.goto('/stack.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 60_000 });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error).toBeUndefined();
    expect(r.finite).toBe(true);
    expect(r.minY).toBeGreaterThan(0.4);
    expect(r.topY).toBeGreaterThan(44);
    expect(r.topY).toBeLessThan(52);
    expect(r.maxXZ).toBeLessThan(0.5);
    expect(r.sleeping).toBe(50);
    expect(r.avgMs).toBeLessThan(8);
    expect(r.p99Ms).toBeLessThan(16);
});
