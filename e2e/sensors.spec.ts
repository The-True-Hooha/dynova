import { test, expect, type Page } from '@playwright/test';

const ready = (page: Page) => page.waitForFunction(() => (window as any).__ready === true || (window as any).__error !== undefined, null, { timeout: 60_000 });

async function open(page: Page) {
    const errors: string[] = [];
    page.on('pageerror', (e) => errors.push(e.message));
    await page.goto('/inspector.html?paused=1&n=0&panel=0');
    await ready(page);
    expect(await page.evaluate(() => (window as any).__error)).toBeUndefined();
    return errors;
}

test('raycasts and sphere overlaps answer from the runtime with hits, misses, ignores and entity ids, and change nothing', async ({ page }) => {
    const errors = await open(page);
    const r = await page.evaluate(() => {
        const w = (window as any).__inspector.world;
        w.setGravity(0, 0, 0);
        w.record(true);
        const sphere = w.spawn({ x: 10, y: 20, z: 3, shape: 1, hx: 1, invMass: 0 });
        const box = w.spawn({ x: 20, y: 20, z: 3, hx: 1, hy: 2, hz: 3, invMass: 0 });
        const behind = w.spawn({ x: 30, y: 20, z: 3, shape: 1, hx: 1, invMass: 0 });
        w.step();
        const hash = w.recording.hash.toString();
        const ids = {
            hit: w.raycast({ origin: [0, 20, 3], direction: [4, 0, 0], maxDistance: 100 }),
            skip: w.raycast({ origin: [0, 20, 3], direction: [1, 0, 0], maxDistance: 100, ignore: sphere }),
            short: w.raycast({ origin: [0, 20, 3], direction: [1, 0, 0], maxDistance: 5 }),
            up: w.raycast({ origin: [0, 20, 3], direction: [0, 1, 0], maxDistance: 100 }),
            two: w.overlapSphere({ center: [15, 20, 3], radius: 6 }),
            none: w.overlapSphere({ center: [0, 100, 0], radius: 1 }),
        };
        const queued = w.readQueryResults();
        w.step();
        const { results, lost } = w.readQueryResults();
        const by = (q: number) => results.filter((x: { query: number }) => x.query === q);
        return { ids, queued, lost, hit: by(ids.hit), skip: by(ids.skip), short: by(ids.short), up: by(ids.up), two: by(ids.two), none: by(ids.none),
            entities: { sphere: sphere.id, box: box.id, behind: behind.id }, same: w.recording.hash.toString() === hash, stats: w.runtimeCounters };
    });
    expect(r.queued.results).toEqual([]);
    expect(r.lost).toBe(0);
    expect(r.hit).toHaveLength(1);
    expect(r.hit[0]).toMatchObject({ kind: 'hit', entity: r.entities.sphere });
    expect(r.hit[0].distance).toBeCloseTo(9, 4);
    expect(r.hit[0].normal[0]).toBeCloseTo(-1, 4);
    expect(r.skip[0]).toMatchObject({ kind: 'hit', entity: r.entities.box });
    expect(r.skip[0].distance).toBeCloseTo(19, 4);
    expect(r.short).toEqual([{ query: r.ids.short, kind: 'miss' }]);
    expect(r.up).toEqual([{ query: r.ids.up, kind: 'miss' }]);
    // The page's own arena walls (ids 0 to 3) are in range too and come first, lowest id first.
    const mine = r.two.map((x: { entity: number }) => x.entity);
    expect(mine).toEqual([...mine].sort((a, b) => a - b));
    expect(mine.filter((id: number) => id >= r.entities.sphere)).toEqual([r.entities.sphere, r.entities.box]);
    expect(r.none).toEqual([{ query: r.ids.none, kind: 'overlap-none' }]);
    expect(r.same, 'the queries left the state hash alone').toBe(true);
    expect(r.stats).toMatchObject({ malformedDropped: 0, staleDropped: 0 });
    expect(errors).toEqual([]);
});

const scenario = (page: Page, withQueries: boolean) => page.evaluate((queries) => {
    const w = (window as any).__inspector.world;
    w.setGravity(0, -9.81, 0);
    w.record(true);
    w.spawn({ x: 10, y: 20, z: 3, shape: 1, hx: 1, invMass: 1 });
    w.spawn({ x: 14, y: 22, z: 3, hx: 1, hy: 1, hz: 1, invMass: 1 });
    const hashes: string[] = [];
    let answered = 0;
    for (let i = 0; i < 60; i++) {
        if (queries && i % 7 === 0) {
            w.raycast({ origin: [0, 20, 3], direction: [1, 0, 0], maxDistance: 100 });
            w.overlapSphere({ center: [12, 20, 3], radius: 5 });
        }
        w.step();
        hashes.push(w.recording.hash.toString());
        answered += w.readQueryResults().results.length;
    }
    w.pause(true);
    w.raycast({ origin: [0, 20, 3], direction: [1, 0, 0], maxDistance: 100 });
    w.step();
    const paused = w.readQueryResults().results.length;
    const start = w.recording.start;
    w.seek(start + 5);
    w.step();
    const afterSeek = w.readQueryResults();
    return { hashes, answered, paused, afterSeek: afterSeek.results.length, lost: afterSeek.lost, seekHash: w.recording.hash.toString(), liveAtSeek: hashes[5] };
}, withQueries);

test('queries are not recorded: asking them does not change the state hash, a paused world still answers, and a scrub does not answer them again', async ({ page }) => {
    const errors = await open(page);
    const asked = await scenario(page, true);
    const quietPage = await page.context().newPage();
    const quietErrors: string[] = [];
    quietPage.on('pageerror', (e) => quietErrors.push(e.message));
    await quietPage.goto('/inspector.html?paused=1&n=0&panel=0');
    await ready(quietPage);
    const quiet = await scenario(quietPage, false);
    expect(asked.hashes).toEqual(quiet.hashes);
    expect(asked.answered).toBeGreaterThan(10);
    expect(quiet.answered).toBe(0);
    expect(asked.paused).toBe(1);
    expect(asked.afterSeek).toBe(0);
    expect(asked.lost).toBe(0);
    expect(asked.seekHash).toBe(asked.liveAtSeek);
    expect(errors).toEqual([]);
    expect(quietErrors).toEqual([]);
});
