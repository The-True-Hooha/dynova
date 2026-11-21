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

async function openDemo(page: Page, query: string) {
    const errors: string[] = [];
    page.on('pageerror', (e) => errors.push(e.message));
    await page.goto(`/telemetry.html?paused=1&${query}`);
    await ready(page);
    expect(await page.evaluate(() => (window as any).__error)).toBeUndefined();
    return errors;
}

test('the telemetry demo feeds every tracked entity from the recorded stream on the sample cadence and draws velocity overlays from the inspector', async ({ page }) => {
    test.setTimeout(120_000);
    const errors = await openDemo(page, 'n=1500&panel=0');
    const r = await page.evaluate(() => {
        const t = (window as any).__telemetry;
        const w = t.world;
        t.step(60);
        const feedsAfter60 = t.feeds();
        const f = w.readTransforms();
        let worst = 0;
        // The sample fed at tick 48 (frame 4) arrived at tick 60; entity ids follow spawn order.
        for (const i of [0, 3, 77, 400, 999, 1499]) {
            const [hx, hy] = t.home(i);
            const [ox, oy] = t.feedOffset(4, i);
            worst = Math.max(worst, Math.hypot(f[i * 7] - (hx + ox), f[i * 7 + 1] - (hy + oy)));
        }
        t.step(6);
        const c = document.getElementById('view') as HTMLCanvasElement;
        const px = c.getContext('2d')!.getImageData(0, 0, c.width, c.height).data;
        let green = 0;
        for (let p = 0; p < px.length; p += 4) if (px[p] === 0x7f && px[p + 1] === 0xe0 && px[p + 2] === 0xa0) green++;
        return { feedsAfter60, worst, green, lines: t.overlayLines(), entities: t.entities, stats: t.stats() };
    });
    expect(r.feedsAfter60).toBe(5);
    expect(r.worst).toBeLessThan(2e-3);
    expect(r.lines).toBeGreaterThan(1000);
    expect(r.lines).toBeLessThanOrEqual(r.entities);
    expect(r.green).toBeGreaterThan(500);
    expect(r.stats).toMatchObject({ malformed: 0, stale: 0 });
    expect(errors).toEqual([]);
});

test('a scrub replays the stream bit-identically, and live stepping after a seek feeds the same samples as the first run', async ({ page }) => {
    test.setTimeout(120_000);
    const errors = await openDemo(page, 'n=1500&panel=0');
    const r = await page.evaluate(() => {
        const t = (window as any).__telemetry;
        const w = t.world;
        const live: Record<number, string> = {};
        for (let i = 0; i < 150; i++) {
            t.step(1);
            live[w.recording.tick] = w.recording.hash.toString();
        }
        const end = w.recording.end, first = w.recording.start;
        const mismatches: string[] = [];
        w.pause(true);
        for (const tick of [end, first + 40, first + 97, end - 1, first + 13]) {
            w.seek(tick);
            w.step();
            if (w.recording.hash.toString() !== live[tick]) mismatches.push(`scrub to ${tick}`);
        }
        w.seek(first + 40);
        w.step();
        w.pause(false);
        const replayed: string[] = [];
        for (let i = 0; i < 100; i++) {
            t.step(1);
            const tick = w.recording.tick;
            if (live[tick] !== undefined && w.recording.hash.toString() !== live[tick]) replayed.push(`live tick ${tick}`);
        }
        return { mismatches, replayed, ticks: w.recording.tick };
    });
    expect(r.mismatches).toEqual([]);
    expect(r.replayed).toEqual([]);
    expect(r.ticks).toBeGreaterThan(100);
    expect(errors).toEqual([]);
});

test('a paused world does not queue the same telemetry batch on every frame, and a bad entity count falls back instead of killing the page', async ({ page }) => {
    const errors = await openDemo(page, 'n=300&panel=0');
    const r = await page.evaluate(() => {
        const t = (window as any).__telemetry;
        const w = t.world;
        t.step(36);
        w.pause(true);
        w.step();
        const before = t.feeds();
        t.step(40);
        return { grew: t.feeds() - before, ticks: w.runtimeCounters.ticks };
    });
    expect(r.ticks % 12).toBe(0);
    expect(r.grew).toBeLessThanOrEqual(1);
    expect(errors).toEqual([]);
    const garbage = await openDemo(page, 'n=abc&panel=0');
    expect(await page.evaluate(() => (window as any).__telemetry.entities)).toBe(2000);
    await openDemo(page, 'n=10.7&panel=0');
    expect(await page.evaluate(() => (window as any).__telemetry.entities)).toBe(10);
    await openDemo(page, 'n=99999999&panel=0');
    expect(await page.evaluate(() => (window as any).__telemetry.entities)).toBe(10000);
    expect(garbage).toEqual([]);
});

test('the telemetry demo mounts the inspector with the timeline controls and lists the tracked entities as static bodies', async ({ page }) => {
    const errors = await openDemo(page, 'n=300');
    await page.evaluate(() => (window as any).__telemetry.step(30));
    await expect(page.locator('[data-testid="scrub"]')).toHaveCount(1);
    await expect(page.locator('[data-testid="record"]')).toHaveCount(1);
    await page.waitForFunction(() => document.querySelector('[data-testid="counts"]')?.textContent?.includes('300 static'), null, { timeout: 5000 });
    expect(errors).toEqual([]);
});
