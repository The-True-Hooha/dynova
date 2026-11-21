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

test('a hinged arm swings on its pivot, a seek replays the joints bit-identically, and the runtime counts nothing as dropped', async ({ page }) => {
    const errors = await open(page);
    const r = await page.evaluate(() => {
        const w = (window as any).__inspector.world;
        const J = { ball: 0, hinge: 1, fixed: 2, slider: 3 };
        w.setGravity(0, -9.81, 0);
        w.record(true);
        const anchor = w.spawn({ x: 0, y: 8, z: 2, invMass: 0, hx: 0.25, hy: 0.25, hz: 0.25 });
        const arm = w.spawn({ x: 1.5, y: 8, z: 5, hx: 1.5, hy: 0.1, hz: 0.1, invMass: 1 });
        const joint = w.createJoint({ kind: J.hinge, a: anchor, b: arm, anchor: [0, 8, 5], axis: [0, 0, 1], limits: [-0.6, 0.6] });
        w.step();
        const live: Record<number, string> = {};
        let lowest = 99;
        let reach = 0;
        for (let i = 0; i < 200; i++) {
            w.step();
            const f = w.readTransforms();
            const x = f[arm.id * 7], y = f[arm.id * 7 + 1];
            lowest = Math.min(lowest, y);
            reach = Math.max(reach, Math.hypot(x, y - 8));
            live[w.recording.tick] = w.recording.hash.toString();
            if (i === 120) w.setJointMotor(joint, 2, 50);
        }
        const end = w.recording.end;
        const first = w.recording.start;
        const mismatches: string[] = [];
        w.pause(true);
        for (const t of [end, first + 60, first + 121, end - 1, first + 2]) {
            w.seek(t);
            w.step();
            if (w.recording.hash.toString() !== live[t]) mismatches.push(`hash differs at ${t}`);
        }
        return { lowest, reach, mismatches, malformed: w.runtimeCounters.malformedDropped, stale: w.runtimeCounters.staleDropped };
    });
    expect(r.mismatches).toEqual([]);
    expect(r.lowest).toBeLessThan(7.3);
    expect(r.lowest).toBeGreaterThan(7.0);
    expect(r.reach).toBeGreaterThan(1.4);
    expect(r.reach).toBeLessThan(1.6);
    expect(r).toMatchObject({ malformed: 0, stale: 0 });
    expect(errors).toEqual([]);
});

test('destroying the arm removes the joint and a later motor command is counted stale instead of crashing', async ({ page }) => {
    const errors = await open(page);
    const r = await page.evaluate(() => {
        const w = (window as any).__inspector.world;
        const anchor = w.spawn({ x: 0, y: 8, z: 2, invMass: 0 });
        const arm = w.spawn({ x: 1, y: 8, z: 5, invMass: 1 });
        const joint = w.createJoint({ kind: 0, a: anchor, b: arm, anchor: [0, 8, 5] });
        w.step();
        w.destroy(arm);
        w.step();
        w.destroyJoint(joint);
        w.step();
        return { stale: w.runtimeCounters.staleDropped, malformed: w.runtimeCounters.malformedDropped };
    });
    expect(r).toEqual({ stale: 1, malformed: 0 });
    expect(errors).toEqual([]);
});
