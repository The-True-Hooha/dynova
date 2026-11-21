import { test, expect } from '@playwright/test';

test('wasm memory growth during a step leaves the world and a pre-existing TransformView readable and writable', async ({ page }) => {
    await page.goto('/growth.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 120_000 });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error).toBeUndefined();
    expect(r.grew).toBe(true);
    expect(r.contacts).toBeGreaterThan(16384);
    expect(r.droppedContacts).toBe(0);
    expect(r.worldLen).toBe(r.expectedLen);
    expect(r.directLen).toBe(r.expectedLen);
    expect(r.finite).toBe(true);
    expect(r.malformed).toBe(0);
    expect(r.lateSpawnY).toBeLessThan(500);
    expect(r.lateSpawnY).toBeGreaterThan(400);
});
