import { test, expect, type Page } from '@playwright/test';

const ready = (page: Page) => page.waitForFunction(() => (window as any).__ready === true || (window as any).__error !== undefined, null, { timeout: 60_000 });

// Phase 7 exit: 5000 tracked entities at 60 fps in the browser. Measured on the demo's own control tick (feed build on every 12th tick,
// world step, Canvas2D draw of all entities, inspector snapshot and velocity overlay), not just the step.
test('the telemetry demo with 5000 entities and the overlay holds a 60 fps frame budget', async ({ page }, info) => {
    test.setTimeout(120_000);
    const errors: string[] = [];
    page.on('pageerror', (e) => errors.push(e.message));
    await page.goto('/telemetry.html?paused=1&n=5000&panel=0');
    await ready(page);
    expect(await page.evaluate(() => (window as any).__error)).toBeUndefined();
    const r = await page.evaluate(() => {
        const t = (window as any).__telemetry;
        t.step(24);
        const times: number[] = [];
        let overlay = 0;
        for (let i = 0; i < 240; i++) {
            const t0 = performance.now();
            t.step(1);
            times.push(performance.now() - t0);
            overlay = Math.max(overlay, t.overlayLines());
        }
        times.sort((a, b) => a - b);
        const at = (q: number) => times[Math.min(times.length - 1, Math.floor(q * times.length))];
        return { mean: times.reduce((a, b) => a + b, 0) / times.length, p50: at(0.5), p95: at(0.95), p99: at(0.99), max: times[times.length - 1], overlay, entities: t.entities };
    });
    console.log(`[${info.project.name}] frame work at ${r.entities} entities: mean ${r.mean.toFixed(2)} ms, p50 ${r.p50.toFixed(2)}, p95 ${r.p95.toFixed(2)}, p99 ${r.p99.toFixed(2)}, max ${r.max.toFixed(2)}; overlay lines ${r.overlay}`);
    expect(r.entities).toBe(5000);
    expect(r.overlay).toBeGreaterThan(1000);
    // Two stalls of a second or more (WebKit's first frames) are in the max and the mean, so the budget is judged on the median and p95.
    expect(r.p50, 'a typical frame of work fits a 60 fps budget with room for rendering').toBeLessThan(16.6);
    expect(r.p95, '19 frames in 20 do').toBeLessThan(16.6);
    expect(errors).toEqual([]);
});

test('with the real animation loop the demo paints at about 60 fps for 5000 entities', async ({ page }, info) => {
    test.setTimeout(120_000);
    await page.goto('/telemetry.html?n=5000&panel=0');
    await ready(page);
    const fps = await page.evaluate(() => new Promise<{ fps: number; frames: number }>((resolve) => {
        let frames = 0;
        const start = performance.now();
        const count = () => {
            frames++;
            if (performance.now() - start < 4000) requestAnimationFrame(count);
            else resolve({ frames, fps: (frames * 1000) / (performance.now() - start) });
        };
        requestAnimationFrame(count);
    }));
    console.log(`[${info.project.name}] rAF rate with the demo running: ${fps.fps.toFixed(1)} fps over ${fps.frames} frames`);
    expect(fps.fps).toBeGreaterThan(50);
});
