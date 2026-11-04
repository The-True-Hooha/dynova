import { test, expect } from '@playwright/test';

// Pinned from the native host run of `physics_scene_matches_golden_hash` (core/src/runtime.rs).
const HOST_HASH = '7356864531754076060';

test('solver output is bit-identical to the native host build in this browser', async ({ page }) => {
    await page.goto('/determinism.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 30_000 });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error).toBeUndefined();
    expect(r.hash).toBe(HOST_HASH);
});
