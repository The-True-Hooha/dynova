import { test, expect, type Page } from '@playwright/test';

const ready = (page: Page) => page.waitForFunction(() => (window as any).__ready === true || (window as any).__error !== undefined, null, { timeout: 60_000 });

async function run(page: Page, query: string) {
    const errors: string[] = [];
    page.on('pageerror', (e) => errors.push(e.message));
    await page.goto(`/assemblyImport.html?paused=1&${query}`);
    await ready(page);
    expect(await page.evaluate(() => (window as any).__error)).toBeUndefined();
    const r = await page.evaluate(() => {
        const t = (window as any).__imp;
        const legs: Array<{ target: number; angle: number; carriage: number; expected: number; stretch: number }> = [];
        for (let cycle = 0; cycle < 3; cycle++) {
            for (const target of [0.9, -0.9, 0]) {
                t.setTarget(target);
                t.step(150);
                const s = t.assembly.states();
                legs.push({ target, angle: s.crank.position, carriage: t.carriageGlobalX(), expected: t.carriageX(s.crank.position), stretch: s.rod?.position ?? s['carriage#1']?.position });
            }
        }
        return { imported: t.imported, legs, hash: t.world.recording.hash.toString(), counters: t.world.runtimeCounters, mates: t.assembly.mateNames };
    });
    expect(errors).toEqual([]);
    return r;
}

test('a crank-slider read from a glTF file imports its parts and mates, moves through its range and holds its loop', async ({ page }) => {
    const r = await run(page, 'offset=0');
    expect(r.imported.errors).toEqual([]);
    expect(r.imported.skipped).toEqual([]);
    expect(r.imported.parts).toEqual(['base', 'crank', 'carriage']);
    expect(r.mates).toEqual(['crank', 'carriage#0', 'carriage#1']);
    for (const leg of r.legs) {
        expect(Math.abs(leg.angle - leg.target), `crank at ${leg.target}`).toBeLessThan(0.04);
        expect(Math.abs(leg.carriage - leg.expected), `carriage at ${leg.target}`).toBeLessThan(0.01);
        expect(Math.abs(leg.stretch), `rod at ${leg.target}`).toBeLessThan(0.01);
    }
    expect(r.counters).toMatchObject({ malformedDropped: 0, staleDropped: 0 });
});

test('the same file modelled 500 km from the origin imports and runs bit-for-bit like the one at the origin', async ({ page }) => {
    const near = await run(page, 'offset=0');
    const far = await run(page, 'offset=500000');
    expect(far.imported.errors).toEqual([]);
    expect(far.hash).toBe(near.hash);
    // The state hash is exact. The carriage readout adds the origin back in 64 bits and subtracts the offset, so it differs by rounding in the last digits.
    far.legs.forEach((l, i) => {
        expect(l.carriage).toBeCloseTo(near.legs[i].carriage, 8);
        expect([l.angle, l.stretch]).toEqual([near.legs[i].angle, near.legs[i].stretch]);
    });
});
