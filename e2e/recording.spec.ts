import { test, expect, type Page } from '@playwright/test';

const ready = (page: Page) => page.waitForFunction(() => (window as any).__ready === true || (window as any).__error !== undefined, null, { timeout: 60_000 });

async function open(page: Page, query = 'n=40&panel=0') {
    const errors: string[] = [];
    page.on('pageerror', (e) => errors.push(e.message));
    await page.goto(`/inspector.html?paused=1&${query}`);
    await ready(page);
    expect(await page.evaluate(() => (window as any).__error)).toBeUndefined();
    return errors;
}

// Hash of the published frame, so the checks compare what a renderer would see, not only the runtime's own hash.
const FRAME_HASH = `(w) => {
    const buf = w.readTransforms();
    const u = new Uint32Array(buf.buffer, buf.byteOffset, buf.length);
    let h = 2166136261;
    for (const v of u) h = Math.imul(h ^ v, 16777619) >>> 0;
    return h;
}`;

test('seeking anywhere in a recording through the SDK reproduces the live state hash and frame', async ({ page }) => {
    const errors = await open(page);
    const result = await page.evaluate((frameHash) => {
        const w = (window as any).__inspector.world;
        const frame = eval(frameHash);
        w.record(true);
        w.step();
        const live: Record<number, { hash: string; frame: number }> = {};
        const first = w.recording.tick;
        for (let i = 0; i < 150; i++) {
            if (i === 40) w.setGravity(0, -3, 0);
            w.step();
            const r = w.recording;
            live[r.tick] = { hash: r.hash.toString(), frame: frame(w) };
        }
        w.pause(true);
        w.step();
        const mismatches: string[] = [];
        const end = w.recording.end;
        for (const t of [end, first + 70, first, first + 71, end - 1, first + 3]) {
            w.seek(t);
            w.step();
            const r = w.recording;
            const want = live[t];
            if (r.tick !== t) mismatches.push(`tick ${r.tick} != ${t}`);
            if (want && (r.hash.toString() !== want.hash || frame(w) !== want.frame)) mismatches.push(`state differs at ${t}`);
        }
        return { mismatches, first, end, paused: w.recording.paused, seeks: w.recording.seeks, malformed: w.runtimeCounters.malformedDropped };
    }, FRAME_HASH);
    expect(result.mismatches).toEqual([]);
    expect(result.end - result.first).toBe(150);
    expect(result).toMatchObject({ paused: true, seeks: 6, malformed: 0 });
    expect(errors).toEqual([]);
});

test('a paused world does not advance and resuming forks the recorded future', async ({ page }) => {
    await open(page);
    const r = await page.evaluate(() => {
        const w = (window as any).__inspector.world;
        w.record(true);
        for (let i = 0; i < 30; i++) w.step();
        w.pause(true);
        w.step();
        const t = w.recording.tick;
        for (let i = 0; i < 10; i++) w.step();
        const held = w.recording.tick;
        w.seek(t - 10);
        w.step();
        const back = { tick: w.recording.tick, end: w.recording.end };
        w.pause(false);
        w.step();
        return { t, held, back, after: { tick: w.recording.tick, end: w.recording.end, paused: w.recording.paused } };
    });
    expect(r.held).toBe(r.t);
    expect(r.back).toEqual({ tick: r.t - 10, end: r.t });
    expect(r.after).toEqual({ tick: r.t - 9, end: r.t - 9, paused: false });
});

test('handles for entities that did not exist at the seeked tick go stale and their ids are reissued', async ({ page }) => {
    await open(page);
    const r = await page.evaluate(() => {
        const w = (window as any).__inspector.world;
        w.record(true);
        for (let i = 0; i < 20; i++) w.step();
        const early = w.recording.tick;
        const late = w.spawn({ x: 0, y: 30, z: 0 });
        for (let i = 0; i < 20; i++) w.step();
        w.pause(true);
        w.step();
        const aliveBefore = (() => { try { w.force(late, 0, 1, 0); return true; } catch { return false; } })();
        w.seek(early);
        w.step();
        w.readTransforms();
        const aliveAfter = (() => { try { w.force(late, 0, 1, 0); return true; } catch (e) { return String(e); } })();
        const reissued = w.spawn({ x: 1, y: 30, z: 0 });
        w.seek(w.recording.end);
        w.step();
        return { aliveBefore, aliveAfter, reissued, late };
    });
    expect(r.aliveBefore).toBe(true);
    expect(String(r.aliveAfter)).toMatch(/stale handle/);
    expect(r.reissued).toEqual(r.late);
});

test('teleport moves a body through the command ring and the new position is published while paused', async ({ page }) => {
    await open(page);
    const r = await page.evaluate(() => {
        const w = (window as any).__inspector.world;
        for (let i = 0; i < 5; i++) w.step();
        w.pause(true);
        w.step();
        const h = { id: 10, gen: 0 };
        w.teleport(h, 3, 40, -2);
        w.step();
        const buf: Float32Array = w.readTransforms();
        return { pos: [buf[10 * 7], buf[10 * 7 + 1], buf[10 * 7 + 2]], ticks: w.recording.tick };
    });
    expect(r.pos).toEqual([3, 40, -2]);
    expect(r.ticks).toBeGreaterThan(0);
});

test('a paused edit is discarded by a seek to its tick and kept once a live step commits it', async ({ page }) => {
    await open(page);
    const r = await page.evaluate(() => {
        const w = (window as any).__inspector.world;
        w.record(true);
        for (let i = 0; i < 20; i++) w.step();
        w.pause(true);
        w.step();
        const tick = w.recording.tick;
        const recorded = w.recording.hash;
        w.teleport({ id: 10, gen: 0 }, 3, 40, -2);
        w.step();
        const edited = w.recording.hash;
        const sameTickWhileEdited = w.recording.tick === tick;
        w.seek(tick);
        w.step();
        const afterSeek = w.recording.hash;
        w.teleport({ id: 10, gen: 0 }, 3, 40, -2);
        w.step();
        w.pause(false);
        w.step();
        const live = w.recording.hash;
        w.pause(true);
        w.step();
        const liveTick = w.recording.tick;
        w.seek(tick);
        w.step();
        w.seek(liveTick);
        w.step();
        return { recorded: recorded.toString(), edited: edited.toString(), afterSeek: afterSeek.toString(), sameTickWhileEdited, replayed: w.recording.hash.toString(), live: live.toString(), liveTick, tick };
    });
    expect(r.sameTickWhileEdited).toBe(true);
    expect(r.edited).not.toBe(r.recorded);
    expect(r.afterSeek).toBe(r.recorded);
    expect(r.liveTick).toBe(r.tick + 1);
    expect(r.replayed).toBe(r.live);
});
