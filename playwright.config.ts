import { defineConfig, devices } from '@playwright/test';

export default defineConfig({
    testDir: 'e2e',
    timeout: 60_000,
    workers: 1,
    webServer: {
        command: 'pnpm --filter @dynova/examples dev --port 5173 --strictPort',
        url: 'http://localhost:5173/plumbing.html',
        reuseExistingServer: true,
        timeout: 60_000,
    },
    use: { baseURL: 'http://localhost:5173' },
    projects: [
        { name: 'chromium', use: { ...devices['Desktop Chrome'] } },
        { name: 'firefox', use: { ...devices['Desktop Firefox'] } },
        { name: 'webkit', use: { ...devices['Desktop Safari'] } },
    ],
});
