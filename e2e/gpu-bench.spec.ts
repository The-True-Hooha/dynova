import { test, expect } from '@playwright/test';

test.use({ launchOptions: { args: ['--enable-unsafe-webgpu'] } });

// The exit asks for a speedup "on a measured GPU": Chromium here only has SwiftShader (a CPU rasteriser), so the speedup is asserted for a hardware adapter only (the page reports whether the adapter is a fallback)
// and measured for the record otherwise.
test('4096 arm worlds: the GPU batch is timed against the CPU batch', async ({ page, browserName }, info) => {
    test.setTimeout(300_000);
    await page.goto('/gpu-bench.html?n=4096&ticks=100');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 280_000 });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error).toBeUndefined();
    test.skip(r.skipped !== undefined, r.skipped);
    info.annotations.push({ type: 'bench', description: `${browserName} ${r.adapter}: cpu ${r.cpuMsPerTick.toFixed(2)} ms, gpu ${r.gpuMsPerTick.toFixed(2)} ms, gpu open loop ${r.gpuOpenLoopMsPerTick.toFixed(2)} ms per tick` });
    console.log(JSON.stringify(r));
    expect(r.finiteObs).toBe(true);
    expect(r.moved, 'the arms moved').toBeGreaterThan(0.05);
    expect(r.angleError, `step(${r.agreeTicks}) in one submission against ${r.agreeTicks} CPU ticks`).toBeLessThan(1e-4);
    expect(r.velocityError).toBeLessThan(1e-3);
    expect(r.worlds).toBe(4096);
    if (r.hardware) expect(r.speedup).toBeGreaterThan(1);
});
