import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { defineConfig, devices } from '@playwright/test';

const app = join(process.env.CONSUMER_DIR ?? join(tmpdir(), 'dynova-consumer'), 'app');

export default defineConfig({
    testDir: 'e2e-consumer',
    timeout: 60_000,
    workers: 1,
    webServer: [
        { command: 'npx vite preview --port 5191 --strictPort', cwd: app, url: 'http://localhost:5191/', reuseExistingServer: false, timeout: 30_000 },
        { command: 'npx vite preview --port 5192 --strictPort', cwd: app, url: 'http://localhost:5192/', env: { NO_ISOLATION: '1' }, reuseExistingServer: false, timeout: 30_000 },
        { command: 'npx vite --port 5193 --strictPort', cwd: app, url: 'http://localhost:5193/', reuseExistingServer: false, timeout: 60_000 },
    ],
    projects: [
        { name: 'chromium', use: { ...devices['Desktop Chrome'] } },
        { name: 'webkit', use: { ...devices['Desktop Safari'] } },
    ],
});
