import { test, expect, type Page } from '@playwright/test';

const ready = (page: Page) => page.waitForFunction(() => (window as any).__ready === true || (window as any).__error !== undefined, null, { timeout: 60_000 });

async function open(page: Page) {
    const errors: string[] = [];
    page.on('pageerror', (e) => errors.push(e.message));
    await page.goto('/inspector.html?paused=1&n=0&panel=0&capacity=5100&ring=4096');
    await ready(page);
    expect(await page.evaluate(() => (window as any).__error)).toBeUndefined();
    return errors;
}

test('5000 tracked entities follow a recorded telemetry stream, step in under a frame, and a seek replays the stream bit-identically', async ({ page }) => {
    test.setTimeout(120_000);
    const errors = await open(page);
    const r = await page.evaluate(() => {
        const w = (window as any).__inspector.world;
        const N = 5000, SIDE = 71, EVERY = 12, TICKS = 360;
        w.setGravity(0, 0, 0);
        w.record(true);
        const home = (i: number): [number, number, number] => [(i % SIDE) * 3, 20, Math.floor(i / SIDE) * 3 + 10];
        const handles = Array.from({ length: N }, (_, i) => {
            const [x, y, z] = home(i);
            return w.spawn({ x, y, z, hx: 0.5, hy: 0.5, hz: 0.5, invMass: 0 });
        });
        // A fixed stream stands in for a recorded one: sample k of entity i is a pure function of (k, i).
        const sample = (k: number, i: number): [number, number, number] => {
            const [x, y, z] = home(i);
            return [x + Math.sin(k * 0.7 + i) * 1.2, y + Math.cos(k * 0.5 + i * 0.3) * 1.2, z + Math.sin(k * 0.3 + i * 0.1)];
        };
        w.step();
        const live: Record<number, string> = {};
        let worst = 0, total = 0;
        for (let t = 0; t < TICKS; t++) {
            if (t % EVERY === 0) {
                const k = t / EVERY;
                w.moveKinematic(handles.map((handle: unknown, i: number) => { const [x, y, z] = sample(k, i); return { handle, x, y, z, ticks: EVERY }; }));
            }
            const t0 = performance.now();
            w.step();
            const ms = performance.now() - t0;
            worst = Math.max(worst, ms);
            total += ms;
            live[w.recording.tick] = w.recording.hash.toString();
        }
        const f = w.readTransforms();
        const last = (TICKS / EVERY - 1);
        let wrong = 0;
        for (let i = 0; i < N; i += 97) {
            const [x, y, z] = sample(last, i);
            if (Math.hypot(f[handles[i].id * 7] - x, f[handles[i].id * 7 + 1] - y, f[handles[i].id * 7 + 2] - z) > 1e-3) wrong++;
        }
        const first = w.recording.start, end = w.recording.end;
        w.pause(true);
        const mismatches: string[] = [];
        for (const t of [end, first + 100, first + 181, end - 1, first + 7, end]) {
            w.seek(t);
            w.step();
            if (w.recording.hash.toString() !== live[t]) mismatches.push(`hash differs at ${t}`);
        }
        return { mean: total / TICKS, worst, wrong, mismatches, malformed: w.runtimeCounters.malformedDropped, stale: w.runtimeCounters.staleDropped };
    });
    expect(r.mismatches).toEqual([]);
    expect(r.wrong).toBe(0);
    expect(r).toMatchObject({ malformed: 0, stale: 0 });
    expect(r.mean).toBeLessThan(16);
    expect(errors).toEqual([]);
    console.log(`telemetry: mean step ${r.mean.toFixed(2)} ms, worst ${r.worst.toFixed(2)} ms`);
});
