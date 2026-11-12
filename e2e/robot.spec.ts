import { test, expect, type Page } from '@playwright/test';

const ready = (page: Page) => page.waitForFunction(() => (window as any).__ready === true || (window as any).__error !== undefined, null, { timeout: 60_000 });

async function open(page: Page, query: string) {
    const errors: string[] = [];
    page.on('pageerror', (e) => errors.push(e.message));
    await page.goto(`/robot.html?paused=1&${query}`);
    await ready(page);
    expect(await page.evaluate(() => (window as any).__error)).toBeUndefined();
    return errors;
}

// A run, then seeks back to several ticks: every seek must reproduce the live hash, so the recorded motor commands replay exactly.
const runAndScrub = (page: Page, ticks: number, scrubs: number[]) => page.evaluate(({ ticks, scrubs }) => {
    const t = (window as any).__robot;
    const w = t.world;
    const live: Record<number, string> = {};
    const trace: number[] = [];
    for (let i = 0; i < ticks; i++) {
        t.step(1);
        live[w.recording.tick] = w.recording.hash.toString();
        if (i % 60 === 0) trace.push(t.scene === 'arm' ? t.error() : t.distanceToEnd());
    }
    const end = w.recording.end, start = w.recording.start;
    const finalHash = w.recording.hash.toString();
    w.pause(true);
    const mismatches: string[] = [];
    for (const s of [end, ...scrubs.map((k: number) => start + k), end - 1, end]) {
        w.seek(s);
        w.step();
        if (w.recording.hash.toString() !== live[s]) mismatches.push(`hash differs at ${s}`);
    }
    return { mismatches, finalHash, trace, counters: w.runtimeCounters };
}, { ticks, scrubs });

// Pinned: the same run gives this hash in Chromium and WebKit, which is the cross-engine check for the controller loop.
const ARM_HASH = '2826087665793585159';
const CAR_HASH = '14230732875360184978';

test('a 3-joint arm reaches its target, holds it, and reaches a second one, with an exact scrub of the recorded motor commands', async ({ page }) => {
    const errors = await open(page, 'scene=arm');
    const first = await page.evaluate(() => {
        const t = (window as any).__robot;
        t.step(240);
        return { error: t.error(), tip: t.tip(), target: t.target() };
    });
    expect(first.error).toBeLessThan(0.06);
    const second = await page.evaluate(() => {
        const t = (window as any).__robot;
        const start = t.tip();
        t.setTarget(0.5, 1.6);
        t.step(300);
        return { start, error: t.error(), tip: t.tip() };
    });
    expect(second.error).toBeLessThan(0.08);
    expect(Math.hypot(second.tip[0] - second.start[0], second.tip[1] - second.start[1])).toBeGreaterThan(0.8);
    const run = await runAndScrub(await freshArm(page, errors), 360, [30, 97, 150, 211]);
    expect(run.mismatches).toEqual([]);
    expect(run.counters).toMatchObject({ malformedDropped: 0, staleDropped: 0 });
    expect(run.trace[run.trace.length - 1]).toBeLessThan(0.06);
    expect(run.finalHash).toBe(ARM_HASH);
    expect(errors).toEqual([]);
});

async function freshArm(page: Page, errors: string[]) {
    page.removeAllListeners('pageerror');
    page.on('pageerror', (e) => errors.push(e.message));
    await page.goto('/robot.html?paused=1&scene=arm');
    await ready(page);
    return page;
}

test('a differential-drive robot follows a path to its end without tipping over, and an exact scrub replays its wheel commands', async ({ page }) => {
    const errors = await open(page, 'scene=car');
    const r = await page.evaluate(() => {
        const t = (window as any).__robot;
        let tallest = 0;
        let arrivedAt = -1;
        for (let i = 0; i < 1400; i++) {
            t.step(1);
            tallest = Math.max(tallest, t.world.readTransforms()[7 + 2]);
            if (arrivedAt < 0 && t.done()) arrivedAt = i;
        }
        return { tallest, arrivedAt, left: t.distanceToEnd(), pose: t.pose(), index: t.index(), maxSplit: t.maxSplit() };
    });
    expect(r.arrivedAt, 'the robot reached the last waypoint').toBeGreaterThan(0);
    expect(r.arrivedAt).toBeLessThan(1200);
    expect(r.maxSplit, 'the path asked for wheel speeds that differ by at least half').toBeGreaterThanOrEqual(0.5);
    expect(r.left).toBeLessThan(0.6);
    expect(r.tallest, 'the chassis stayed on the floor').toBeLessThan(0.2);
    const run = await runAndScrub(await freshCar(page, errors), 1000, [100, 333, 500, 777]);
    expect(run.mismatches).toEqual([]);
    expect(run.counters).toMatchObject({ malformedDropped: 0, staleDropped: 0 });
    expect(run.finalHash).toBe(CAR_HASH);
    expect(errors).toEqual([]);
});

async function freshCar(page: Page, errors: string[]) {
    page.removeAllListeners('pageerror');
    page.on('pageerror', (e) => errors.push(e.message));
    await page.goto('/robot.html?paused=1&scene=car');
    await ready(page);
    return page;
}

for (const scene of ['arm', 'car'] as const) {
    test(`the ${scene} demo's controller stays quiet while paused, keeps the recording after a scrub, and carries on after resuming`, async ({ page }) => {
        const errors = await open(page, `scene=${scene}`);
        const r = await page.evaluate(() => {
            const t = (window as any).__robot;
            const w = t.world;
            t.step(200);
            const start = w.recording.start;
            w.pause(true);
            w.step();
            w.seek(start + 50);
            w.step();
            const metric = () => (t.scene === 'arm' ? t.error() : t.distanceToEnd());
            const leftAtSeek = metric();
            const endBefore = w.recording.end, hashBefore = w.recording.hash.toString(), keyframesBefore = w.recording.keyframes;
            t.step(8);
            const stayed = { end: w.recording.end === endBefore, hash: w.recording.hash.toString() === hashBefore, keyframes: w.recording.keyframes === keyframesBefore };
            w.pause(false);
            t.step(t.scene === 'arm' ? 400 : 500);
            return { stayed, endBefore, endAfter: w.recording.end, left: metric(), leftAtSeek, counters: w.runtimeCounters };
        });
        expect(r.stayed).toEqual({ end: true, hash: true, keyframes: true });
        expect(r.endAfter).toBeGreaterThan(r.endBefore);
        // The arm settles on its target; the robot, restarted from tick 50 of its run, closes on the end of its path (it may not stop on it).
        if (scene === 'arm') expect(r.left).toBeLessThan(0.08);
        else expect(r.left).toBeLessThan(r.leftAtSeek - 3);
        expect(r.counters).toMatchObject({ malformedDropped: 0, staleDropped: 0 });
        expect(errors).toEqual([]);
    });
}

test('after a seek the robot demo picks its waypoint again from where the robot now is, not from where the run ended', async ({ page }) => {
    const errors = await open(page, 'scene=car');
    const r = await page.evaluate(() => {
        const t = (window as any).__robot;
        const w = t.world;
        t.step(900);
        const indexAtEnd = t.index();
        w.pause(true);
        w.step();
        w.seek(w.recording.start + 40);
        w.step();
        w.pause(false);
        t.step(1);
        return { indexAtEnd, indexAfter: t.index(), x: t.pose().x };
    });
    expect(r.indexAtEnd).toBeGreaterThanOrEqual(3);
    expect(r.x).toBeLessThan(1.5);
    expect(r.indexAfter).toBeLessThanOrEqual(1);
    expect(errors).toEqual([]);
});
