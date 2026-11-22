import { test, expect } from '@playwright/test';

test('water in a basin holds a floating cube at about its analytic draft while sand rests outside, all in the browser', async ({ page }, info) => {
    test.setTimeout(180_000);
    // A failure seen twice in Chromium and never reproduced (cold and loaded dev server, combined runs) left no message: keep what the page said so the next one does.
    const pageLog: string[] = [];
    page.on('pageerror', (e) => pageLog.push(`pageerror: ${e.message}`));
    page.on('console', (m) => { if (m.type() === 'error' || m.type() === 'warning') pageLog.push(`${m.type()}: ${m.text()}`); });
    page.on('requestfailed', (r) => pageLog.push(`requestfailed: ${r.url()} ${r.failure()?.errorText}`));
    page.on('framenavigated', (f) => pageLog.push(`navigated: ${f.url()}`));
    await page.goto('/fluid.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 150_000 }).catch((e) => { throw new Error(`${e.message}\n${pageLog.join("\n")}`); });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error, pageLog.join('\n')).toBeUndefined();
    expect(r.particles, pageLog.join('\n')).toBe(3072 + 512);
    expect(r.malformed).toBe(0);
    expect(r.leaked).toBe(0);
    // A cube at half the density of water floats with half its height under the surface (0.15 m); sunk to the floor it would read 0.3 and resting on top 0.
    expect(r.draft).toBeGreaterThan(0.12);
    expect(r.draft).toBeLessThan(0.17);
    expect(r.sandLowest).toBeGreaterThan(0.02);
    expect(r.sandWidth).toBeLessThan(0.5);
    info.annotations.push({ type: 'timing', description: `${r.particles} particles, median step ${r.medianStepMs.toFixed(2)} ms, p95 ${r.p95StepMs.toFixed(2)} ms, draft ${r.draft.toFixed(3)}, sand width ${r.sandWidth.toFixed(2)}` });
});
