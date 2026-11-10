import { test, expect } from '@playwright/test';

// Pinned from the native host run of `physics_scene_matches_golden_hash` (core/src/runtime.rs).
const HOST_HASH = '4599892428385582693';

test('solver output is bit-identical to the native host build in this browser', async ({ page }) => {
    await page.goto('/determinism.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 30_000 });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error).toBeUndefined();
    expect(r.hash).toBe(HOST_HASH);
});

// Pinned from core/src/lib.rs `determinism_probe_is_stable_and_finite_math_only`.
const HOST_PROBE = '7496710723231958662';

test('sin/cos/atan2/sqrt integration is bit-identical to the native build in this browser', async ({ page }) => {
    await page.goto('/determinism.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 30_000 });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error).toBeUndefined();
    expect(r.probe).toBe(HOST_PROBE);
});

// Pinned from core/src/determinism.rs `PINNED_END` and `PINNED_MIDDLE`; the four values are the live end, the live middle tick,
// the middle tick after a seek and the end after seeking back (a seek must reproduce the live run in every browser too).
const HOST_SCENE = ['1177776368450856606', '15081626903481916999', '15081626903481916999', '1177776368450856606'];

test('a 700-tick mixed scene (all four shapes, sleeping, forces, teleport, respawn) and its record/seek round trip match the native build in this browser', async ({ page }) => {
    await page.goto('/determinism.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 30_000 });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error).toBeUndefined();
    expect(r.scene).toEqual(HOST_SCENE);
});
