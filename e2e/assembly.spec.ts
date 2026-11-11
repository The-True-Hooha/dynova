import { test, expect, type Page } from '@playwright/test';

const ready = (page: Page) => page.waitForFunction(() => (window as any).__ready === true || (window as any).__error !== undefined, null, { timeout: 60_000 });

async function open(page: Page, query: string) {
    const errors: string[] = [];
    page.on('pageerror', (e) => errors.push(e.message));
    await page.goto(`/assembly.html?paused=1&${query}`);
    await ready(page);
    expect(await page.evaluate(() => (window as any).__error)).toBeUndefined();
    return errors;
}

// Sweeps the crank through its range three times and reports, at the end of each leg, how far the mates are from where kinematics says they should be.
const sweep = (page: Page) => page.evaluate(() => {
    const t = (window as any).__asm;
    const legs: Array<{ target: number; angle: number; carriage: number; expected: number; stretch: number }> = [];
    const w = t.world;
    const hashes: string[] = [];
    for (let cycle = 0; cycle < 3; cycle++) {
        for (const target of [0.9, -0.9, 0]) {
            t.setTarget(target);
            t.step(150);
            const s = t.asm.states();
            legs.push({ target, angle: s.crank.position, carriage: t.carriageGlobalX(), expected: t.carriageX(s.crank.position), stretch: s.rod.position });
            hashes.push(w.recording.hash.toString());
        }
    }
    return { legs, finalHash: w.recording.hash.toString(), counters: w.runtimeCounters };
});

// Pinned: the same run gives this hash in Chromium and WebKit.
const ASSEMBLY_HASH = '17604264783152797242';

test('the crank-slider assembly moves through its range and returns to where it started, with the rod holding its length', async ({ page }) => {
    const errors = await open(page, 'offset=0');
    const r = await sweep(page);
    for (const leg of r.legs) {
        expect(Math.abs(leg.angle - leg.target), `crank at ${leg.target}`).toBeLessThan(0.04);
        expect(Math.abs(leg.carriage - leg.expected), `carriage at ${leg.target}`).toBeLessThan(0.01);
        expect(Math.abs(leg.stretch), `rod at ${leg.target}`).toBeLessThan(0.01);
    }
    const home = r.legs.filter((l) => l.target === 0);
    expect(Math.abs(home[2].carriage - home[0].carriage), 'no drift over three cycles').toBeLessThan(0.005);
    expect(r.counters).toMatchObject({ malformedDropped: 0, staleDropped: 0 });
    expect(r.finalHash).toBe(ASSEMBLY_HASH);
    expect(errors).toEqual([]);
});

test('the same assembly 500 km from the origin behaves bit-for-bit like the one at the origin when the assembly removes its origin', async ({ page }) => {
    const near = await sweepAt(page, 'offset=0');
    const far = await sweepAt(page, 'offset=500000');
    expect(far.finalHash).toBe(near.finalHash);
    expect(far.legs).toEqual(near.legs);
});

test('without the floating origin the raw coordinates 500 km out leave the carriage up to 6 cm off where the crank puts it and the rod stretching', async ({ page }) => {
    const raw = await sweepAt(page, 'offset=500000&float=0');
    const carriageError = Math.max(...raw.legs.map((l) => Math.abs(l.carriage - l.expected)));
    const stretch = Math.max(...raw.legs.map((l) => Math.abs(l.stretch)));
    expect(carriageError, 'the carriage sits on a coarse grid instead of where the crank puts it').toBeGreaterThan(0.04);
    expect(stretch, 'the rod no longer holds its length').toBeGreaterThan(0.015);
});

async function sweepAt(page: Page, query: string) {
    const errors = await open(page, query);
    const r = await sweep(page);
    expect(errors).toEqual([]);
    return r;
}
