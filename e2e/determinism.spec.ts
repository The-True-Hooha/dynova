import { test, expect } from '@playwright/test';

// Pinned from the native host run of `physics_scene_matches_golden_hash` (core/src/runtime.rs).
const HOST_HASH = '2762124975456161600';

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
const HOST_SCENE = ['13348242057567185977', '8317036014868065234', '8317036014868065234', '13348242057567185977'];

test('a 700-tick mixed scene (all four shapes, sleeping, forces, teleport, respawn) and its record/seek round trip match the native build in this browser', async ({ page }) => {
    await page.goto('/determinism.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 30_000 });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error).toBeUndefined();
    expect(r.scene).toEqual(HOST_SCENE);
});

// Pinned from core/src/determinism.rs `JOINT_PINNED_END` and `JOINT_PINNED_MIDDLE`.
const HOST_JOINT_SCENE = ['13978249971055882505', '309544726468439355', '309544726468439355', '13978249971055882505'];

test('a 1230-tick joint scene (chain, limited door, motor wheel and slider, weld, joint and body destroy) and its seek round trip match the native build in this browser', async ({ page }) => {
    await page.goto('/determinism.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 30_000 });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error).toBeUndefined();
    expect(r.jointScene).toEqual(HOST_JOINT_SCENE);
});

// Pinned from core/src/determinism.rs `KIN_PINNED_END` and `KIN_PINNED_MIDDLE`.
const HOST_KINEMATIC_SCENE = ['1104800618632004284', '6926923596733983700', '6926923596733983700', '1104800618632004284'];

test('a 700-tick kinematic scene (platform, wall and joint anchor fed by samples, rejected samples) and its seek round trip match the native build in this browser', async ({ page }) => {
    await page.goto('/determinism.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 30_000 });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error).toBeUndefined();
    expect(r.kinematicScene).toEqual(HOST_KINEMATIC_SCENE);
});

// Pinned from core/src/determinism.rs `SENSOR_PINNED`.
test('raycast and overlap results against a settled mixed scene match the native build in this browser', async ({ page }) => {
    await page.goto('/determinism.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 30_000 });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error).toBeUndefined();
    expect(r.sensorScene).toBe('2460514860714154088');
});
