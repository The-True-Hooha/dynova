import { test, expect } from '@playwright/test';

test('physics demo runs in the worker, settles on the floor and stays finite', async ({ page }) => {
    const errors: string[] = [];
    page.on('pageerror', (e) => errors.push(String(e)));
    await page.goto('/');
    await page.waitForFunction(() => (window as any).__demo !== undefined, null, { timeout: 20_000 });
    await page.waitForTimeout(4000);

    const snap = await page.evaluate(() => {
        const { world, boxes, spheres } = (window as any).__demo;
        const buf: Float32Array = world.readTransforms();
        const bodies = [...boxes, ...spheres].map((h: { id: number }) => Array.from(buf.slice(h.id * 7, h.id * 7 + 7)));
        return { mode: world.mode, bodies };
    });

    expect(errors).toEqual([]);
    expect(snap.mode).toBe('shared');
    expect(snap.bodies).toHaveLength(50);
    for (const b of snap.bodies as number[][]) {
        expect(b.every(Number.isFinite)).toBe(true);
        expect(b[6]).not.toBe(0);
        expect(b[1]).toBeGreaterThan(-14.6);
        expect(b[1]).toBeLessThan(-8);
        expect(Math.abs(b[0])).toBeLessThan(18);
    }
});
