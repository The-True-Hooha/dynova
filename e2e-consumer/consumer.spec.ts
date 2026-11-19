import { test, expect, type Page } from '@playwright/test';

const ISOLATED = 'http://localhost:5191/';
const PLAIN = 'http://localhost:5192/';
const DEV = 'http://localhost:5193/';

const ready = (page: Page) => page.waitForFunction(() => (window as any).__ready === true || (window as any).__error !== undefined, null, { timeout: 45_000 });

async function open(page: Page, url: string) {
    const errors: string[] = [];
    page.on('pageerror', (e) => errors.push(e.message));
    await page.goto(url);
    await ready(page);
    return errors;
}

const settle = async (page: Page) => {
    await page.waitForFunction(() => {
        const a = (window as any).__app;
        return a && a.world.runtimeCounters.ticks > 150;
    }, null, { timeout: 30_000 });
};

async function inspect(page: Page) {
    return page.evaluate(() => {
        const a = (window as any).__app;
        const t = a.world.readTransforms();
        const ballY = t[a.ball.id * 7 + 1];
        let boxesOnFloor = 0, highest = 0;
        for (let id = a.floor.id + 1; id <= 20; id++) { const y = t[id * 7 + 1]; if (y > 0 && y < 6) boxesOnFloor++; highest = Math.max(highest, y); }
        const m = new Float32Array(16);
        a.mesh.instanceMatrix.array.slice(a.ball.id * 16, a.ball.id * 16 + 16).forEach((v: number, i: number) => { m[i] = v; });
        const rows = document.querySelectorAll('[data-testid^="entity-"]').length;
        const joints = a.world.readJointStates();
        return { highest, drawnBall: a.drawnAt(a.ball.id), drawnBox: a.drawnAt(1), drawnFloor: a.drawnAt(a.floor.id), mode: a.world.mode, ticks: a.world.runtimeCounters.ticks, ballY, boxesOnFloor, meshBallY: m[13], pixels: a.pixels(), rows, joints: joints.length, isolated: a.isolated, imports: a.imports, motor: a.motor().velocity, counters: a.world.runtimeCounters, snapshot: a.inspector.snapshot() !== null };
    });
}

test('on an isolated page the packaged world runs in a worker over shared memory and every adapter reads it', async ({ page }) => {
    const errors = await open(page, ISOLATED);
    expect(await page.evaluate(() => (window as any).__error)).toBeUndefined();
    await settle(page);
    const r = await inspect(page);
    expect(r.isolated).toBe(true);
    expect(r.mode).toBe('shared');
    expect(r.ticks).toBeGreaterThan(150);
    expect(r.ballY, 'the ball fell onto the boxes').toBeLessThan(8);
    expect(r.ballY).toBeGreaterThan(0);
    expect(r.boxesOnFloor).toBe(20);
    expect(r.meshBallY, 'the Three.js instance matches the buffer').toBeCloseTo(r.ballY, 3);
    expect([r.drawnFloor, r.drawnBox, r.drawnBall], 'the Canvas2D view painted the floor, a box and the ball where they are').toEqual([true, true, true]);
    expect(r.highest, 'the stack settled: the top row came down from its spawn height').toBeLessThan(4.2);
    expect(r.rows, 'the inspector listed every entity read from shared memory the worker writes').toBe(24);
    expect(r.snapshot).toBe(true);
    expect(r.joints, 'joint states cross the worker boundary').toBe(1);
    expect(r.imports).toEqual({ importAssembly: 'function', collidersFromScene: 'function', loadModelFiles: 'function' });
    expect(r.motor).toBe(1);
    expect(r.counters).toMatchObject({ malformedDropped: 0, staleDropped: 0 });
    expect(errors).toEqual([]);
});

test('on an isolated page the packaged batch worker steps worlds on two workers', async ({ page }) => {
    await open(page, ISOLATED);
    const r = await page.evaluate(() => (window as any).__app.runBatch());
    expect(r.failed).toEqual([]);
    expect(r.ys).toHaveLength(4);
    r.ys.forEach((y: number, i: number) => {
        expect(y, `world ${i} fell from ${4 + i}`).toBeLessThan(4 + i);
        expect(y).toBeGreaterThan(0);
    });
    expect(new Set(r.ys).size, 'each world ran its own scene').toBe(4);
});

test('without SharedArrayBuffer the same code runs the world on the calling thread', async ({ page }) => {
    await page.addInitScript(() => { delete (window as any).SharedArrayBuffer; });
    const errors = await open(page, ISOLATED);
    expect(await page.evaluate(() => (window as any).__error)).toBeUndefined();
    await settle(page);
    const r = await inspect(page);
    expect(r.mode).toBe('fallback');
    expect(r.ballY, 'physics ran on the calling thread: the ball fell').toBeLessThan(7.5);
    expect(r.highest, 'and the stack settled').toBeLessThan(4.2);
    expect(r.boxesOnFloor).toBe(20);
    expect([r.drawnFloor, r.drawnBox, r.drawnBall]).toEqual([true, true, true]);
    expect(r.meshBallY).toBeCloseTo(r.ballY, 3);
    expect(r.rows).toBe(24);
    expect(r.joints).toBe(1);
    expect(errors).toEqual([]);
});

test('on a page that supports shared memory but lacks the isolation headers the error says what to serve', async ({ page }) => {
    await open(page, PLAIN);
    const err = await page.evaluate(() => (window as any).__error);
    expect(err.name).toBe('SharedMemoryUnavailableError');
    expect(err.message).toContain('Cross-Origin-Opener-Policy');
    expect(err.message).toContain('Cross-Origin-Embedder-Policy');
});

test('the Vite dev server (unbundled, with the documented optimizeDeps exclusion) runs the packaged world over shared memory', async ({ page }) => {
    const errors = await open(page, DEV);
    expect(await page.evaluate(() => (window as any).__error)).toBeUndefined();
    await settle(page);
    const r = await inspect(page);
    expect(r.mode).toBe('shared');
    expect(r.ballY).toBeLessThan(8);
    expect(r.boxesOnFloor).toBe(20);
    expect([r.drawnFloor, r.drawnBox, r.drawnBall]).toEqual([true, true, true]);
    expect(r.rows).toBe(24);
    expect(r.joints).toBe(1);
    expect(errors).toEqual([]);
});
