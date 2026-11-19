import { test, expect } from '@playwright/test';

test('every world of a wasm batch ends bit-identical to the same world run alone, in this browser', async ({ page }) => {
    await page.goto('/batch.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 30_000 });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error).toBeUndefined();
    expect(r.mismatched).toEqual([]);
    expect(r.distinct, 'the worlds differ from each other, so a mix-up between worlds would show').toBeGreaterThan(3);
});
