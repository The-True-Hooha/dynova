import { test, expect } from '@playwright/test';

test('worlds spread over four real workers end bit-identical to the same worlds run alone', async ({ page }, info) => {
    await page.goto('/parallel.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 90_000 });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error).toBeUndefined();
    expect(r.workers).toBe(4);
    expect(r.failed).toEqual([]);
    expect(r.mismatched).toEqual([]);
    expect(r.distinct, 'the worlds differ from each other').toBeGreaterThan(3);
    info.annotations.push({ type: 'timing', description: `64 worlds x 300 bodies, 40 ticks: 1 worker ${Math.round(r.oneWorkerMs)} ms, 4 workers ${Math.round(r.fourWorkersMs)} ms, ${r.cores} cores` });
});
