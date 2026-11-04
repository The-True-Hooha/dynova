import { test, expect } from '@playwright/test';

test('the three adapter mirrors the transform buffer for a settled 50-box wall, with no per-entity scene objects', async ({ page }) => {
    await page.goto('/three.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 60_000 });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error).toBeUndefined();
    expect(r.worst).toBeLessThan(1e-4);
    expect(r.topY).toBeGreaterThan(9.2);
    expect(r.topY).toBeLessThan(10.6);
    expect(r.children).toBe(1);
    expect(r.meshChildren).toBe(0);
});
