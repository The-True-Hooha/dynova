import { test, expect, type Page } from '@playwright/test';

const ready = (page: Page) => page.waitForFunction(() => (window as any).__ready === true || (window as any).__error !== undefined, null, { timeout: 60_000 });

async function open(page: Page, query: string) {
    const errors: string[] = [];
    page.on('pageerror', (e) => errors.push(e.message));
    await page.goto(`/inspector.html?paused=1&${query}`);
    await ready(page);
    expect(await page.evaluate(() => (window as any).__error)).toBeUndefined();
    return errors;
}

const step = (page: Page, n: number) => page.evaluate((k) => (window as any).__inspector.step(k), n);
const text = (page: Page, testid: string) => page.locator(`[data-testid="${testid}"]`).first().textContent();

test('the panel shows profiler counters, entity counts and a selected entity with its contacts', async ({ page }) => {
    const errors = await open(page, 'n=60');
    await step(page, 200);
    await page.evaluate(() => (window as any).__inspector.panel.update());

    expect(Number(((await text(page, 'p-ticks')) ?? '').replace('tick', ''))).toBeGreaterThanOrEqual(200);
    expect(await text(page, 'counts')).toMatch(/^\d+ awake, \d+ sleeping, 4 static$/);
    expect(await text(page, 'list-note')).toBe('64 entities');
    expect(await page.locator('[data-testid^="entity-"]').count()).toBe(64);
    expect(await text(page, 'p-queue')).toBe('queue depth0');
    expect(await text(page, 'p-memory')).toMatch(/MiB$/);

    await page.locator('[data-testid="entity-10"]').click();
    expect(await text(page, 'r-state')).toMatch(/^id \/ state10 \/ (awake|sleeping)$/);
    expect(await text(page, 'r-position')).toMatch(/^position-?\d/);
    const touching = await page.locator('[data-testid="contact"]').count();
    const none = await page.locator('[data-testid="no-contacts"]').count();
    expect(touching + none).toBeGreaterThan(0);
    expect(errors).toEqual([]);
});

test('the state filter narrows the list and the note reports the shown total', async ({ page }) => {
    await open(page, 'n=60');
    await step(page, 400);
    await page.evaluate(() => (window as any).__inspector.panel.update());
    await page.locator('[data-testid="filter"]').selectOption('static');
    expect(await page.locator('[data-testid^="entity-"]').count()).toBe(4);
    expect(await text(page, 'list-note')).toBe('4 entities');
    await page.locator('[data-testid="filter"]').selectOption('all');
    expect(await page.locator('[data-testid^="entity-"]').count()).toBe(64);
});

test('selecting a missing entity says so instead of showing stale numbers', async ({ page }) => {
    await open(page, 'n=10');
    await step(page, 5);
    await page.evaluate(() => (window as any).__inspector.panel.select(7000));
    expect(await text(page, 'readout-empty')).toBe('entity 7000 does not exist');
});

test('the inspector never changes the simulation: the same scene gives identical transforms with and without the panel', async ({ page }) => {
    const hashAfter = async (query: string) => {
        await open(page, query);
        await step(page, 300);
        return page.evaluate(() => {
            const w = (window as any).__inspector.world;
            const buf: Float32Array = w.readTransforms();
            let h = 2166136261;
            for (const v of buf) h = Math.imul(h ^ new Uint32Array(new Float32Array([v]).buffer)[0], 16777619) >>> 0;
            return h;
        });
    };
    const closed = await hashAfter('n=120&panel=0');
    const open_ = await hashAfter('n=120&panel=1');
    expect(open_).toBe(closed);
});

test('disposing the panel removes it and stops the runtime from publishing snapshots', async ({ page }) => {
    await open(page, 'n=30');
    await step(page, 5);
    expect(await page.locator('[data-testid="inspector"]').count()).toBe(1);
    const before = await page.evaluate(async () => {
        const i = (window as any).__inspector;
        i.panel.dispose();
        i.step(3);
        const first = (document.querySelectorAll('[data-testid="inspector"]').length);
        return first;
    });
    expect(before).toBe(0);
    await page.evaluate(() => (window as any).__inspector.step(40));
    const after = await page.evaluate(() => {
        const i = (window as any).__inspector;
        return { age: i.view.profile().snapshotAgeTicks, snapshot: i.view.snapshot() };
    });
    expect(after).toEqual({ age: null, snapshot: null });
});

test('a growth during a run keeps the panel live and counts it', async ({ page }) => {
    await open(page, 'n=20');
    await step(page, 5);
    await page.evaluate(() => {
        const i = (window as any).__inspector;
        i.world.requestGrow(200);
        i.step(3);
        i.panel.update();
    });
    expect(await text(page, 'p-growths')).toBe('buffer growths1');
    await page.evaluate(() => (window as any).__inspector.step(5));
    await page.evaluate(() => (window as any).__inspector.panel.update());
    expect(await page.locator('[data-testid^="entity-"]').count()).toBe(24);
});

test('a second panel on the same view is refused, and disposing after the world is gone still removes the panel', async ({ page }) => {
    await open(page, 'n=10');
    const result = await page.evaluate(() => {
        const i = (window as any).__inspector;
        let refused = '';
        try {
            i.mountInspector(document.body, { view: i.view, setEnabled: () => {} });
        } catch (e) {
            refused = String(e);
        }
        i.world.dispose();
        let threw = false;
        try {
            i.panel.dispose();
        } catch {
            threw = true;
        }
        return { refused, threw, panels: document.querySelectorAll('[data-testid="inspector"]').length };
    });
    expect(result.refused).toContain('already mounted');
    expect(result.threw).toBe(true);
    expect(result.panels).toBe(0);
});

test('opening the inspector does not slow the simulation (5000 bodies, step time with and without)', async ({ page, browserName }) => {
    test.skip(browserName !== 'chromium', 'timing check runs once');
    test.setTimeout(400_000);
    const run = async (on: boolean) => {
        await page.goto(`/bench.html?n=5000&inspector=${on ? 1 : 0}`);
        await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 220_000 });
        const r = await page.evaluate(() => (window as any).__result);
        expect(r.error).toBeUndefined();
        expect(r.finite).toBe(true);
        return r;
    };
    // Alternate and keep the best of each so a slow first page load or a busy moment cannot decide the result.
    const closed: number[] = [];
    const opened: number[] = [];
    let last: any;
    for (let round = 0; round < 2; round++) {
        closed.push((await run(false)).late.p50Ms);
        last = await run(true);
        opened.push(last.late.p50Ms);
    }
    const bestClosed = Math.min(...closed);
    const bestOpen = Math.min(...opened);
    console.log(`inspector closed best p50 ${bestClosed.toFixed(2)} ms, open best p50 ${bestOpen.toFixed(2)} ms, snapshot read ${last.snapshotMs?.toFixed(2)} ms x${last.snapshotsTaken}, missed ${last.snapshotMissed}`);
    expect(last.snapshotsTaken).toBeGreaterThan(10);
    expect(last.snapshotsOk).toBe(last.snapshotsTaken);
    expect(last.lastSnapshotContacts).toBeGreaterThan(1000);
    expect(last.snapshotMissed).toBe(0);
    expect(bestOpen).toBeLessThan(bestClosed * 1.15);
    expect(last.snapshotMs).toBeLessThan(25);
});
