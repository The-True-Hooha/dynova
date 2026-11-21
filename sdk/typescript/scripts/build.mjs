// Builds the publishable SDK into dist/: TypeScript to ESM plus declarations, and the two wasm builds beside it.
// The wasm must already be built in release mode (`make build-release build-st`, or `make package`).
import { execFileSync } from 'node:child_process';
import { cpSync, existsSync, mkdirSync, readFileSync, readdirSync, rmSync, statSync, writeFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const dist = join(root, 'dist');
if (process.env.DYNOVA_RELEASE !== '1') {
    throw new Error('refusing to package a possibly debug wasm: run `make package`, which builds the release wasm first');
}
const WASM = ['wasm', 'wasm-st'];
const SHIPPED = ['dynova_core.js', 'dynova_core.d.ts', 'dynova_core_bg.wasm', 'dynova_core_bg.wasm.d.ts'];

for (const dir of WASM) {
    for (const file of SHIPPED) {
        if (!existsSync(join(root, dir, file))) throw new Error(`${dir}/${file} is missing: build the wasm first (make build-release build-st)`);
    }
}

rmSync(dist, { recursive: true, force: true });
execFileSync('pnpm', ['exec', 'tsc', '-p', 'tsconfig.build.json'], { cwd: root, stdio: 'inherit' });

for (const dir of WASM) {
    mkdirSync(join(dist, dir), { recursive: true });
    for (const file of SHIPPED) cpSync(join(root, dir, file), join(dist, dir, file));
}

// The sources reach the wasm by `../wasm`; inside dist/ it sits next to the code.
const rewrites = [
    ['sim.worker.js', [["'../wasm/dynova_core.js'", "'./wasm/dynova_core.js'"], ["'../wasm/dynova_core_bg.wasm'", "'./wasm/dynova_core_bg.wasm'"]]],
    ['batch.worker.js', [["'../wasm/dynova_core.js'", "'./wasm/dynova_core.js'"], ["'../wasm/dynova_core_bg.wasm'", "'./wasm/dynova_core_bg.wasm'"]]],
    ['web.js', [["'../wasm-st/dynova_core.js'", "'./wasm-st/dynova_core.js'"]]],
];
for (const [file, pairs] of rewrites) {
    const path = join(dist, file);
    let text = readFileSync(path, 'utf8');
    for (const [from, to] of pairs) {
        const count = text.split(from).length - 1;
        if (count !== 1) throw new Error(`${file}: expected exactly one ${from}, found ${count}`);
        text = text.replace(from, to);
    }
    writeFileSync(path, text);
}

// Nothing in dist may still point outside it.
const stray = [];
const scan = (dir) => {
    for (const name of readdirSync(dir)) {
        const path = join(dir, name);
        if (statSync(path).isDirectory()) { scan(path); continue; }
        if (!name.endsWith('.js') || path.includes(`${join(dist, 'wasm')}`)) continue;
        if (/['"]\.\.\/wasm/.test(readFileSync(path, 'utf8'))) stray.push(path);
    }
};
scan(dist);
if (stray.length) throw new Error(`still references ../wasm: ${stray.join(', ')}`);
writeFileSync(join(dist, 'RELEASE'), 'built from the release wasm by make package\n');
console.log(`built ${dist}`);
