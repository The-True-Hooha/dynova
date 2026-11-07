import { test, expect } from '@playwright/test';

test.describe.configure({ mode: 'serial' });

for (const n of [1000, 5000, 10000]) {
    test(`bench ${n} bodies: pile settles without NaN or dropped commands, timings reported`, async ({ page, browserName }, info) => {
        test.skip(browserName !== 'chromium', 'results are deterministic and identical in WebKit; one browser keeps the gate short');
        test.setTimeout(240_000);
        await page.goto(`/bench.html?n=${n}`);
        await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 220_000 });
        const r = await page.evaluate(() => (window as any).__result);
        expect(r.error).toBeUndefined();
        expect(r.finite).toBe(true);
        expect(r.malformedDropped).toBe(0);
        expect(r.pending).toBe(0);
        expect(r.sleeping).toBeLessThanOrEqual(n);
        expect(r.buckets.used).toBeGreaterThan(0);
        info.annotations.push({ type: 'bench', description: JSON.stringify(r) });
        console.log(`[${info.project.name}] ${JSON.stringify(r)}`);
    });
}
