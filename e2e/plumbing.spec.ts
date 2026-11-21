import { test, expect } from '@playwright/test';

async function result(page: import('@playwright/test').Page) {
    await page.goto('/plumbing.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 40_000 });
    return page.evaluate(() => (window as any).__result);
}

test('10k spawns reach the main thread through the worker in shared mode', async ({ page }) => {
    const r = await result(page);
    expect(r.error).toBeUndefined();
    expect(r).toMatchObject({ mode: 'shared', crossOriginIsolated: true, spawned: 10_000, allocated: 10_000, firstBadEntity: -1 });
});

test('10k spawns work through the sync fallback when SharedArrayBuffer is missing', async ({ page }) => {
    await page.addInitScript(() => { delete (window as any).SharedArrayBuffer; });
    const r = await result(page);
    expect(r.error).toBeUndefined();
    expect(r).toMatchObject({ mode: 'fallback', spawned: 10_000, allocated: 10_000, firstBadEntity: -1 });
});

test('shared mode never instantiates wasm on the main thread', async ({ page }) => {
    await page.addInitScript(() => {
        const orig = WebAssembly.instantiate;
        (window as any).__wasmCalls = 0;
        WebAssembly.instantiate = function (...a: any[]) { (window as any).__wasmCalls++; return (orig as any).apply(this, a); } as any;
        const origStream = WebAssembly.instantiateStreaming;
        if (origStream) WebAssembly.instantiateStreaming = function (...a: any[]) { (window as any).__wasmCalls++; return (origStream as any).apply(this, a); } as any;
    });
    const r = await result(page);
    expect(r.mode).toBe('shared');
    expect(await page.evaluate(() => (window as any).__wasmCalls)).toBe(0);
});
