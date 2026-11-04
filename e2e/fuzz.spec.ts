import { test, expect } from '@playwright/test';

for (const seed of [1, 2, 3]) {
    test(`fuzz seed ${seed}: 10k ticks of spawn/destroy/grow churn with a concurrent reader`, async ({ page, browserName }) => {
        test.setTimeout(150_000);
        await page.goto(`/fuzz.html?seed=${seed}`);
        await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 120_000 });
        const r = await page.evaluate(() => (window as any).__result);
        expect(r.error, `${browserName} seed ${seed}`).toBeUndefined();
        expect(r.ticks).toBeGreaterThanOrEqual(10_000);
        expect(r.pendingAtEnd).toBe(0);
        expect(r.tornReads).toBe(0);
        expect(r.finalTorn).toBe(0);
        expect(r.staleDropped).toBe(0);
        expect(r.malformedDropped).toBe(0);
        expect(r.backpressureDrops).toBe(0);
        expect(r.mismatched).toEqual([]);
        expect(r.actualLive).toBe(r.expectedLive);
        expect(r.observedGrows).toBe(r.requestedGrows);
        expect(r.observedGrows).toBeGreaterThan(0);
        expect(r.deferredTotal).toBeGreaterThan(0);
        expect(r.reads).toBeGreaterThan(50);
    });
}
