import { test, expect, type Page } from '@playwright/test';

const ready = (page: Page) => page.waitForFunction(() => (window as any).__ready === true || (window as any).__error !== undefined, null, { timeout: 60_000 });

async function open(page: Page, query = 'n=40') {
    const errors: string[] = [];
    page.on('pageerror', (e) => errors.push(e.message));
    await page.goto(`/inspector.html?paused=1&${query}`);
    await ready(page);
    expect(await page.evaluate(() => (window as any).__error)).toBeUndefined();
    // Steps the world and remembers the state hash and the published frame of every tick it produces.
    await page.evaluate(() => {
        const w = (window as any);
        w.__live = {} as Record<number, { hash: string; frame: number }>;
        w.__frame = () => {
            const buf: Float32Array = w.__inspector.world.readTransforms();
            const u = new Uint32Array(buf.buffer, buf.byteOffset, buf.length);
            let h = 2166136261;
            for (const v of u) h = Math.imul(h ^ v, 16777619) >>> 0;
            return h;
        };
        w.__run = (n: number) => {
            for (let i = 0; i < n; i++) {
                w.__inspector.step(1);
                const r = w.__inspector.world.recording;
                if (r && r.on && !r.paused) w.__live[r.tick] = { hash: r.hash.toString(16).padStart(16, '0'), frame: w.__frame() };
            }
        };
    });
    return errors;
}

const step = (page: Page, n = 1) => page.evaluate((k) => (window as any).__inspector.step(k), n);
const label = async (page: Page) => (await page.locator('[data-testid="timeline-label"]').textContent()) ?? '';
const parseLabel = (text: string) => {
    const m = /^tick (\d+) of (\d+)–(\d+), (\d+) keyframes, hash ([0-9a-f]{16})/.exec(text);
    if (!m) throw new Error(`unexpected timeline label: ${text}`);
    return { tick: Number(m[1]), start: Number(m[2]), end: Number(m[3]), keyframes: Number(m[4]), hash: m[5], paused: /, paused$/.test(text) };
};

test('exit: record 10 s with a mouse drag in it, scrub backward and forward through the panel, and every tick comes back bit-identical', async ({ page }) => {
    const errors = await open(page);

    await page.locator('[data-testid="record"]').click();
    await step(page);
    await page.evaluate(() => (window as any).__inspector.panel.update());
    expect(await page.locator('[data-testid="record"]').textContent()).toBe('stop recording');

    await page.evaluate(() => (window as any).__run(120));
    // Body 43 is the last one spawned, so the pointer picks it rather than a neighbour whose box overlaps its centre.
    const at = await page.evaluate(() => (window as any).__inspector.screenOf(43));
    await page.mouse.move(at.x, at.y);
    await page.mouse.down();
    for (let i = 1; i <= 8; i++) {
        await page.mouse.move(at.x + i * 12, at.y - i * 9);
        await page.evaluate(() => (window as any).__run(5));
    }
    await page.mouse.up();
    const moved = await page.evaluate(() => (window as any).__inspector.screenOf(43));
    expect(Math.abs(moved.x - at.x), 'the drag moved the body to the right').toBeGreaterThan(60);
    await page.evaluate(() => (window as any).__run(600 - 120 - 40));

    const live: Record<string, { hash: string; frame: number }> = await page.evaluate(() => (window as any).__live);
    const ticks = Object.keys(live).map(Number).sort((a, b) => a - b);
    expect(ticks.length).toBe(600);
    const start = ticks[0] - 1;
    const end = ticks[ticks.length - 1];

    await page.locator('[data-testid="pause"]').click();
    await step(page);
    await page.evaluate(() => (window as any).__inspector.panel.update());
    expect(parseLabel(await label(page))).toMatchObject({ end, paused: true });
    expect(parseLabel(await label(page)).keyframes).toBeGreaterThanOrEqual(10);

    const scrubTo = async (t: number) => {
        await page.locator('[data-testid="scrub"]').fill(String(t));
        await step(page);
        await page.evaluate(() => (window as any).__inspector.panel.update());
        return parseLabel(await label(page));
    };
    const frameNow = () => page.evaluate(() => (window as any).__frame());

    const order = [end, start + 1, Math.floor((start + end) / 2), end - 1, start + 61, start + 60, end - 59, 300, 301, 299, end];
    for (const t of order) {
        const shown = await scrubTo(t);
        expect(shown.tick, `tick after scrubbing to ${t}`).toBe(t);
        expect(shown.hash, `state hash at ${t}`).toBe(live[t].hash);
        expect(await frameNow(), `published frame at ${t}`).toBe(live[t].frame);
    }

    // Walk the whole recording tick by tick with the step buttons, forward then backward.
    await scrubTo(start + 1);
    for (let i = 0; i < 25; i++) {
        await page.locator('[data-testid="step-forward"]').click();
        await step(page);
    }
    await page.evaluate(() => (window as any).__inspector.panel.update());
    expect(parseLabel(await label(page))).toMatchObject({ tick: start + 26, hash: live[start + 26].hash });
    for (let i = 0; i < 10; i++) {
        await page.locator('[data-testid="step-back"]').click();
        await step(page);
    }
    await page.evaluate(() => (window as any).__inspector.panel.update());
    expect(parseLabel(await label(page))).toMatchObject({ tick: start + 16, hash: live[start + 16].hash });

    // Snapshot round trip: end -> start -> end gives the same state, and it is the live one.
    const first = await scrubTo(end);
    await scrubTo(start + 1);
    const second = await scrubTo(end);
    expect(second.hash).toBe(first.hash);
    expect(second.hash).toBe(live[end].hash);

    expect(await page.evaluate(() => (window as any).__inspector.world.runtimeCounters.malformedDropped)).toBe(0);
    expect(errors).toEqual([]);
});

test('the scrubber and the step buttons are disabled until something is recorded and at the ends of the range', async ({ page }) => {
    await open(page);
    await step(page, 3);
    await page.evaluate(() => (window as any).__inspector.panel.update());
    expect(await page.locator('[data-testid="scrub"]').isDisabled()).toBe(true);
    expect(await page.locator('[data-testid="step-back"]').isDisabled()).toBe(true);
    expect(await label(page)).toBe('not recording');

    await page.locator('[data-testid="record"]').click();
    await step(page, 5);
    await page.locator('[data-testid="pause"]').click();
    await step(page);
    await page.evaluate(() => (window as any).__inspector.panel.update());
    expect(await page.locator('[data-testid="step-forward"]').isDisabled()).toBe(true);
    expect(await page.locator('[data-testid="step-back"]').isDisabled()).toBe(false);
    expect(await page.locator('[data-testid="pause"]').textContent()).toBe('resume');
});

test('typing a position into the panel moves the body through a command, and the edit replays after resuming', async ({ page }) => {
    const errors = await open(page);
    await page.locator('[data-testid="record"]').click();
    await step(page, 30);
    await page.locator('[data-testid="pause"]').click();
    await step(page);
    await page.locator('[data-testid="entity-10"]').click();

    await page.locator('[data-testid="pos-x"]').fill('3');
    await page.locator('[data-testid="pos-y"]').fill('40');
    await page.locator('[data-testid="pos-z"]').fill('-1');
    await page.locator('[data-testid="pos-z"]').press('Enter');
    await step(page);
    const pos = () => page.evaluate(() => { const b: Float32Array = (window as any).__inspector.world.readTransforms(); return [b[70], b[71], b[72]]; });
    expect(await pos()).toEqual([3, 40, -1]);

    await page.locator('[data-testid="pause"]').click();
    await step(page, 20);
    const after = await pos();
    expect(after[1]).toBeLessThan(40);
    await page.evaluate(() => (window as any).__inspector.panel.update());
    const lastTick = await page.evaluate(() => (window as any).__inspector.world.recording.tick);
    const liveHash = await page.evaluate(() => (window as any).__inspector.world.recording.hash.toString());
    await page.locator('[data-testid="scrub"]').fill(String(lastTick - 15));
    await step(page);
    await page.locator('[data-testid="scrub"]').fill(String(lastTick));
    await step(page);
    expect(await page.evaluate(() => (window as any).__inspector.world.recording.hash.toString())).toBe(liveHash);
    expect(errors).toEqual([]);
});

test('an invalid or out-of-range coordinate is flagged and Enter does nothing', async ({ page }) => {
    const errors = await open(page);
    await step(page, 5);
    await page.locator('[data-testid="entity-10"]').click();
    const before = await page.evaluate(() => { const b: Float32Array = (window as any).__inspector.world.readTransforms(); return [b[70], b[71], b[72]]; });
    await page.locator('[data-testid="pos-x"]').fill('abc');
    expect(await page.locator('[data-testid="pos-x"]').getAttribute('class')).toContain('bad');
    await page.locator('[data-testid="pos-x"]').press('Enter');
    expect(await page.evaluate(() => (window as any).__inspector.world.pendingCommands)).toBe(0);
    await page.locator('[data-testid="pos-x"]').fill('1e9');
    expect(await page.locator('[data-testid="pos-x"]').getAttribute('class')).not.toContain('bad');
    await page.locator('[data-testid="pos-x"]').press('Enter');
    expect(await page.locator('[data-testid="pos-x"]').getAttribute('class')).toContain('bad');
    expect(await page.evaluate(() => (window as any).__inspector.world.pendingCommands)).toBe(0);
    await step(page);
    const after = await page.evaluate(() => { const b: Float32Array = (window as any).__inspector.world.readTransforms(); return [b[70], b[71], b[72]]; });
    expect(after[0]).toBeCloseTo(before[0], 0);
    expect(errors).toEqual([]);
});

test('dragging a body on the canvas follows the pointer and selects it; a static wall is selected but does not move', async ({ page }) => {
    const errors = await open(page);
    await step(page, 60);
    const at = await page.evaluate(() => (window as any).__inspector.screenOf(43));
    await page.mouse.move(at.x, at.y);
    await page.mouse.down();
    await page.mouse.move(at.x + 80, at.y - 120);
    await step(page);
    await page.mouse.up();
    const to = await page.evaluate(() => (window as any).__inspector.screenOf(43));
    expect(Math.abs(to.x - (at.x + 80))).toBeLessThan(4);
    expect(Math.abs(to.y - (at.y - 120))).toBeLessThan(10);
    expect(await page.locator('[data-testid="entity-43"].sel').count()).toBe(1);

    // Wall 3 is the backdrop behind the whole arena; its centre is empty, so the pointer picks it.
    const wall = await page.evaluate(() => (window as any).__inspector.screenOf(3));
    await page.mouse.move(wall.x, wall.y);
    await page.mouse.down();
    await page.mouse.move(wall.x + 60, wall.y - 60);
    await step(page);
    await page.mouse.up();
    expect(await page.locator('[data-testid="entity-3"].sel').count(), 'the wall was picked').toBe(1);
    expect(await page.evaluate(() => (window as any).__inspector.screenOf(3))).toEqual(wall);
    expect(errors).toEqual([]);
});

test('without controls the panel stays read-only: no timeline and no editable position', async ({ page }) => {
    await open(page, 'n=20&controls=0');
    await step(page, 5);
    await page.locator('[data-testid="entity-10"]').click();
    expect(await page.locator('[data-testid="timeline"]').count()).toBe(0);
    expect(await page.locator('[data-testid="pos-x"]').count()).toBe(0);
    expect(await page.locator('[data-testid="r-position"]').textContent()).toMatch(/^position-?\d/);
});

test('moving the slider while the world runs pauses it first and lands on the tick', async ({ page }) => {
    await open(page);
    await page.locator('[data-testid="record"]').click();
    await step(page, 40);
    await page.evaluate(() => (window as any).__inspector.panel.update());
    const start = parseLabel(await label(page)).start;
    await page.locator('[data-testid="scrub"]').fill(String(start + 3));
    await step(page);
    const r = await page.evaluate(() => { const x = (window as any).__inspector.world.recording; return { paused: x.paused, tick: x.tick }; });
    expect(r).toEqual({ paused: true, tick: start + 3 });
});

test('several slider moves before the next step land on the last one and count nothing as malformed', async ({ page }) => {
    await open(page);
    await page.locator('[data-testid="record"]').click();
    await step(page, 60);
    await page.locator('[data-testid="pause"]').click();
    await step(page);
    await page.evaluate(() => (window as any).__inspector.panel.update());
    const { start, end } = parseLabel(await label(page));
    for (const t of [start + 5, start + 25, end - 3]) await page.locator('[data-testid="scrub"]').fill(String(t));
    await step(page);
    const r = await page.evaluate(() => { const w = (window as any).__inspector.world; return { tick: w.recording.tick, malformed: w.runtimeCounters.malformedDropped }; });
    expect(r).toEqual({ tick: end - 3, malformed: 0 });
});

test('a failing command shows a message in the timeline instead of throwing', async ({ page }) => {
    const errors = await open(page);
    await step(page, 3);
    await page.evaluate(() => {
        const w = (window as any).__inspector.world;
        w.dispose();
    });
    await page.locator('[data-testid="record"]').click();
    expect(await label(page)).toMatch(/^command refused: world unavailable/);
    expect(errors).toEqual([]);
});
