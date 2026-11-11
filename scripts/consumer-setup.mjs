// Packs the four packages, installs the tarballs into a clean app outside the workspace, typechecks and bundles it.
// `make consumer-test` runs this and then the Playwright specs in e2e-consumer/.
import { execFileSync, spawnSync } from 'node:child_process';
import { cpSync, mkdirSync, readFileSync, readdirSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const repo = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const dir = process.env.CONSUMER_DIR ?? join(tmpdir(), 'dynova-consumer');
if (!dir.includes('dynova-consumer')) throw new Error(`CONSUMER_DIR must contain 'dynova-consumer' (it is deleted first), got ${dir}`);
const version = JSON.parse(readFileSync(join(repo, 'sdk/typescript/package.json'), 'utf8')).version;
const packages = { sdk: 'sdk/typescript', three: 'renderer/three', canvas2d: 'renderer/canvas2d', inspector: 'inspector' };
const pm = process.env.CONSUMER_PM ?? 'npm';
if (!['npm', 'pnpm'].includes(pm)) throw new Error(`CONSUMER_PM must be npm or pnpm, got ${pm}`);
const run = (cmd, args, cwd) => execFileSync(cmd, args, { cwd, stdio: 'inherit' });
const exec = (args, cwd) => (pm === 'pnpm' ? run('pnpm', ['exec', ...args], cwd) : run('npx', args, cwd));

rmSync(dir, { recursive: true, force: true });
mkdirSync(join(dir, 'tarballs'), { recursive: true });
for (const path of Object.values(packages)) run('pnpm', ['pack', '--pack-destination', join(dir, 'tarballs')], join(repo, path));
const tarball = (name) => {
    const file = readdirSync(join(dir, 'tarballs')).find((f) => f === `dynova-${name}-${version}.tgz`);
    if (!file) throw new Error(`no tarball for ${name}`);
    return `file:../tarballs/${file}`;
};

const app = join(dir, 'app');
cpSync(join(repo, 'consumer-fixture'), app, { recursive: true });
const three = JSON.parse(readFileSync(join(repo, 'renderer/three/package.json'), 'utf8')).devDependencies.three;
const vite = JSON.parse(readFileSync(join(repo, 'package.json'), 'utf8')).devDependencies.vite;
writeFileSync(join(app, 'package.json'), JSON.stringify({
    name: 'dynova-consumer', private: true, type: 'module',
    dependencies: {
        '@dynova/sdk': tarball('sdk'), '@dynova/three': tarball('three'), '@dynova/canvas2d': tarball('canvas2d'), '@dynova/inspector': tarball('inspector'), three,
    },
    devDependencies: { vite, typescript: '^7.0', '@types/three': '^0.170.0' },
    // The packed adapters depend on @dynova/sdk by version, which is not on a registry: point every copy at the tarball.
    overrides: { '@dynova/sdk': tarball('sdk') },
}, null, 2));

if (pm === 'pnpm') {
    // Outside the workspace: an empty workspace file keeps pnpm from walking up to the repo's.
    writeFileSync(join(app, 'pnpm-workspace.yaml'), `packages: []\noverrides:\n  '@dynova/sdk': ${tarball('sdk')}\n`);
    run('pnpm', ['install'], app);
} else {
    run('npm', ['install', '--no-audit', '--no-fund'], app);
}
exec(['tsc', '--noEmit'], app);
// Third-party declarations (@webgpu/types against TypeScript 7's DOM lib) fail a full check, so the app skips library checks;
// check our own declarations on their own: any error inside @dynova/* fails.
const full = pm === 'pnpm'
    ? spawnSync('pnpm', ['exec', 'tsc', '--noEmit', '--skipLibCheck', 'false'], { cwd: app, encoding: 'utf8' })
    : spawnSync('npx', ['tsc', '--noEmit', '--skipLibCheck', 'false'], { cwd: app, encoding: 'utf8' });
const ours = `${full.stdout}${full.stderr}`.split('\n').filter((l) => l.includes('node_modules/@dynova/'));
if (ours.length) throw new Error(`errors in the packed declarations:\n${ours.join('\n')}`);
exec(['vite', 'build'], app);
console.log(`consumer app ready in ${app}`);
