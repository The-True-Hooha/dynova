import { test, expect } from '@playwright/test';

test('stage 3A: scene meshes become colliders, props fall and settle on the level, instanced render matches the buffer', async ({ page }) => {
    const pageErrors: string[] = [];
    page.on('pageerror', (e) => pageErrors.push(String(e)));
    await page.goto('/stage3a.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 90_000 });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error).toBeUndefined();
    expect(pageErrors).toEqual([]);
    expect(r.staticKinds).toEqual(['floor:box', 'ramp:box', 'pillar:hull']);
    expect(r.propKinds).toEqual(['box', 'hull', 'sphere']);
    expect(r.propCount).toBe(r.props);
    expect(r.skipped).toEqual([]);
    expect(r.failed).toEqual([]);
    expect(r.malformed).toBe(0);
    expect(r.finite).toBe(true);
    expect(r.adapterMismatch).toBeLessThan(1e-4);
    expect(r.overRamp).toBeGreaterThan(50);
    expect(r.rampSink).toBeLessThan(0.08);
    expect(r.minHeight).toBeGreaterThan(0.2);
    expect(r.crate0[1]).toBeGreaterThan(0.25);
    expect(r.ball0[1]).toBeGreaterThan(0.2);
    expect(r.cone0[1]).toBeGreaterThan(0.1);
    expect(r.sleeping).toBeGreaterThanOrEqual(r.props - 6);
});
