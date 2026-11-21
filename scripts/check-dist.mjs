// Runs before `pnpm pack`: refuses to pack a package whose published entry points do not exist, or an SDK without its wasm.
import { existsSync, readFileSync } from 'node:fs';
import { join } from 'node:path';

const dir = process.cwd();
const pkg = JSON.parse(readFileSync(join(dir, 'package.json'), 'utf8'));
const missing = [];
const walk = (v) => {
    if (typeof v === 'string') { if (v.startsWith('./') && v !== './package.json' && !existsSync(join(dir, v))) missing.push(v); }
    else if (v && typeof v === 'object') Object.values(v).forEach(walk);
};
walk(pkg.publishConfig?.exports);
if (pkg.name === '@dynova/sdk') {
    for (const d of ['wasm', 'wasm-st']) for (const f of ['dynova_core.js', 'dynova_core_bg.wasm']) if (!existsSync(join(dir, 'dist', d, f))) missing.push(`dist/${d}/${f}`);
    if (!existsSync(join(dir, 'dist', 'RELEASE'))) missing.push('dist/RELEASE (built without make package)');
}
if (missing.length) {
    console.error(`${pkg.name}: not built (${missing.join(', ')}). Run \`make package\` first.`);
    process.exit(1);
}
