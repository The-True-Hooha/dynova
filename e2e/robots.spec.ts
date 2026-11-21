import { test, expect } from '@playwright/test';

test('the shipped URDF assets load in the browser: an arm matches forward kinematics, the quadruped and the humanoid stand', async ({ page }) => {
    await page.goto('/robots.html');
    await page.waitForFunction(() => (window as any).__result !== undefined, null, { timeout: 90_000 });
    const r = await page.evaluate(() => (window as any).__result);
    expect(r.error).toBeUndefined();
    expect(r.arm.worstError, 'every link of the arm within 1 cm of its forward-kinematics pose').toBeLessThan(0.01);
    expect(r.quadruped.tilt).toBeLessThan(0.05);
    expect(r.quadruped.y).toBeGreaterThan(0.48);
    expect(r.quadruped.y).toBeLessThan(0.53);
    expect(r.quadruped.drift).toBeLessThan(0.05);
    expect(r.humanoid.tilt).toBeLessThan(0.15);
    expect(r.humanoid.y).toBeGreaterThan(0.9);
    expect(r.humanoid.y).toBeLessThan(0.97);
    expect(r.humanoid.drift).toBeLessThan(0.1);
});
