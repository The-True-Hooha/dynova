import { test, expect, type Page } from '@playwright/test';
import { box, buildGlb, cube, plane, rampGrid, uvSphere } from '../renderer/three/src/testing/gltfFixture';

async function stats(page: Page, ticks = 1500) {
    await page.waitForFunction(() => (window as any).__ready === true, null, { timeout: 60_000 });
    await page.evaluate((n) => (window as any).__step(n), ticks);
    return page.evaluate(() => (window as any).__stats());
}

const arena = () => Buffer.from(buildGlb([
    { name: 'ground', ...plane(30, 30) },
    { name: 'dome', ...uvSphere(2, 16, 12), translation: [6, 0, 0] },
    { name: 'tilt', ...rampGrid(5, 0.4), translation: [-8, 0, -4] },
]));

const pageErrors: string[] = [];
test.beforeEach(({ page }) => {
    pageErrors.length = 0;
    page.on('pageerror', (e) => pageErrors.push(String(e)));
});
test.afterEach(() => {
    expect(pageErrors).toEqual([]);
});

const glbBytes = (b: ArrayBuffer) => Buffer.from(b);

test('built-in GLB level loads through GLTFLoader, colliders match, props stay inside the walls', async ({ page }) => {
    await page.goto('/gltf.html?paused=1');
    const r = await stats(page);
    expect(r.levelKinds).toEqual([
        'floor:box', 'wall-n:box', 'wall-s:box', 'wall-w:box', 'wall-e:box', 'ramp:box', 'pillar:hull', 'crate-static:box',
    ]);
    expect(r.skipped).toEqual([]);
    expect(r.failed).toEqual([]);
    expect(r.malformed).toBe(0);
    expect(r.finite).toBe(true);
    expect(r.propCount).toBe(26);
    expect(r.propsInside).toBe(26);
    expect(r.minPropY).toBeGreaterThan(0.05);
    expect(r.sleeping).toBeGreaterThanOrEqual(r.propCount - 6);
    expect(r.coneInstanceScale).toBeCloseTo(r.propScale, 3);
});

test('?url= loads a GLB over HTTP and builds its colliders', async ({ page }) => {
    await page.route((u) => u.pathname === '/arena.glb', (route) => route.fulfill({ status: 200, contentType: 'model/gltf-binary', body: arena() }));
    await page.goto('/gltf.html?paused=1&url=/arena.glb');
    const r = await stats(page);
    expect(r.levelKinds).toEqual(['ground:box', 'dome:sphere', 'tilt:box']);
    expect(r.failed).toEqual([]);
    expect(r.malformed).toBe(0);
    expect(r.minPropY).toBeGreaterThan(r.floorY - 0.1);
    expect(r.finite).toBe(true);
    expect(r.coneInstanceScale).toBeCloseTo(r.propScale, 3);
});

test('choosing a file replaces the scene and runtime; a bad file reports an error and keeps the old scene', async ({ page }) => {
    await page.goto('/gltf.html?paused=1');
    await stats(page, 10);
    await page.setInputFiles('#file', { name: 'bowl.glb', mimeType: 'model/gltf-binary', buffer: Buffer.from(buildGlb([
        { name: 'slab', ...box(8, 0.3, 8), translation: [0, -0.3, 0] },
        { name: 'block', ...cube(1), translation: [0, 1, 0] },
    ])) });
    await page.waitForFunction(() => (window as any).__stats()?.label === 'bowl.glb', null, { timeout: 30_000 });
    await page.evaluate(() => (window as any).__step(1200));
    const loaded = await page.evaluate(() => (window as any).__stats());
    expect(loaded.levelKinds).toEqual(['slab:box', 'block:box']);
    expect(loaded.malformed).toBe(0);
    expect(loaded.finite).toBe(true);
    expect(loaded.ticks).toBe(1200);

    await page.setInputFiles('#file', { name: 'broken.glb', mimeType: 'model/gltf-binary', buffer: Buffer.from('this is not a glb file at all') });
    await page.waitForFunction(() => (window as any).__error !== undefined, null, { timeout: 30_000 });
    const after = await page.evaluate(() => (window as any).__stats());
    expect(after.label).toBe('bowl.glb');
    expect(await page.locator('#msg').textContent()).toContain('could not load broken.glb');
});

test('dropping a .glb on the page loads it, and a drop with no file is ignored', async ({ page }) => {
    await page.goto('/gltf.html?paused=1');
    await stats(page, 10);
    const bytes = Array.from(new Uint8Array(buildGlb([
        { name: 'pad', ...box(6, 0.2, 6), translation: [0, -0.2, 0] },
        { name: 'ball-static', ...uvSphere(0.7, 12, 8), translation: [0, 0.7, 0] },
    ])));
    await page.evaluate(() => addEventListener('drop', () => undefined));
    await page.evaluate(() => { window.dispatchEvent(new DragEvent('drop', { dataTransfer: new DataTransfer(), cancelable: true })); });
    expect(await page.evaluate(() => (window as any).__stats().label)).toContain('built-in');
    await page.evaluate((b) => {
        const dt = new DataTransfer();
        dt.items.add(new File([new Uint8Array(b)], 'dropped.glb', { type: 'model/gltf-binary' }));
        window.dispatchEvent(new DragEvent('drop', { dataTransfer: dt, cancelable: true }));
    }, bytes);
    await page.waitForFunction(() => (window as any).__stats()?.label === 'dropped.glb', null, { timeout: 30_000 });
    await page.evaluate(() => (window as any).__step(900));
    const r = await page.evaluate(() => (window as any).__stats());
    expect(r.levelKinds).toEqual(['pad:box', 'ball-static:sphere']);
    expect(r.malformed).toBe(0);
    expect(r.finite).toBe(true);
});

test('a GLB with nothing to collide with is rejected with a message and the running scene is kept', async ({ page }) => {
    await page.goto('/gltf.html?paused=1');
    await stats(page, 10);
    await page.setInputFiles('#file', { name: 'empty.glb', mimeType: 'model/gltf-binary', buffer: glbBytes(buildGlb([])) });
    await page.waitForFunction(() => (window as any).__error !== undefined, null, { timeout: 30_000 });
    expect(await page.locator('#msg').textContent()).toContain('no meshes to collide with');
    expect(await page.evaluate(() => (window as any).__stats().label)).toContain('built-in');
});

test('an error is cleared by the next successful load and choosing the same bad file twice reports twice', async ({ page }) => {
    await page.goto('/gltf.html?paused=1');
    await stats(page, 10);
    const bad = { name: 'broken.glb', mimeType: 'model/gltf-binary', buffer: Buffer.from('not a glb') };
    await page.setInputFiles('#file', bad);
    await page.waitForFunction(() => (window as any).__errorCount === 1, null, { timeout: 30_000 });
    expect(await page.evaluate(() => (document.getElementById('file') as HTMLInputElement).files?.length)).toBe(0);
    await page.setInputFiles('#file', bad);
    await page.waitForFunction(() => (window as any).__errorCount === 2, null, { timeout: 30_000 });
    await page.setInputFiles('#file', { name: 'ok.glb', mimeType: 'model/gltf-binary', buffer: glbBytes(buildGlb([{ name: 'slab', ...box(5, 0.2, 5), translation: [0, -0.2, 0] }])) });
    await page.waitForFunction(() => (window as any).__stats()?.label === 'ok.glb', null, { timeout: 30_000 });
    expect(await page.evaluate(() => (window as any).__error)).toBeUndefined();
});

test('with two overlapping loads the last requested file wins even if the earlier one finishes later', async ({ page }) => {
    const slow = glbBytes(buildGlb([{ name: 'slow-slab', ...box(5, 0.2, 5), translation: [0, -0.2, 0] }]));
    await page.route((u) => u.pathname === '/slow.glb', async (route) => {
        await new Promise((r) => setTimeout(r, 1500));
        await route.fulfill({ status: 200, contentType: 'model/gltf-binary', body: slow });
    });
    await page.goto('/gltf.html?paused=1&url=/slow.glb', { waitUntil: 'commit' });
    await page.waitForFunction(() => typeof (window as any).__step === 'function', null, { timeout: 30_000 });
    await page.setInputFiles('#file', { name: 'fast.glb', mimeType: 'model/gltf-binary', buffer: glbBytes(buildGlb([{ name: 'fast-slab', ...box(6, 0.2, 6), translation: [0, -0.2, 0] }])) });
    await page.waitForFunction(() => (window as any).__stats()?.label === 'fast.glb', null, { timeout: 30_000 });
    await page.waitForFunction(() => (window as any).__ready === true, null, { timeout: 30_000 });
    const r = await page.evaluate(() => (window as any).__stats());
    expect(r.label).toBe('fast.glb');
    expect(r.levelKinds).toEqual(['fast-slab:box']);
});
