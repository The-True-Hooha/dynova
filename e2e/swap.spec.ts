import { test, expect } from '@playwright/test';

test('adapter swap: one World drives Three.js, then Canvas2D, then Three.js again with no change to the runtime', async ({ page }) => {
    const pageErrors: string[] = [];
    page.on('pageerror', (e) => pageErrors.push(String(e)));
    await page.goto('/swap.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 90_000 });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error).toBeUndefined();
    expect(pageErrors).toEqual([]);
    expect(r.sequence).toEqual(['three', 'canvas2d', 'three']);
    expect(r.ticks).toBe(1200);
    expect(r.malformed).toBe(0);
    expect(r.finite).toBe(true);
    expect(r.glLive).toBe(r.props);
    expect(r.flatLive).toBe(r.props);
    expect(r.mismatchBefore).toBeLessThan(1e-4);
    expect(r.mismatchAfter).toBeLessThan(1e-4);
    expect(r.inBounds).toBeGreaterThanOrEqual(20);
    expect(r.painted).toBe(r.inBounds);
    expect(r.largestJump).toBeLessThan(0.5);
});
