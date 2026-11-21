import { describe, it, expect } from 'vitest';
import { readFileSync, readdirSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { parseUrdf } from '../robot.js';

const root = new URL('../../../../assets/', import.meta.url);
const manifest = JSON.parse(readFileSync(new URL('manifest.json', root), 'utf8')) as { assets: { file: string; name: string; licence: string; source: string; sha256: string }[] };

describe('the asset set', () => {
    it('lists every robot file, and each entry has a licence, a source and the hash of its file', () => {
        const onDisk = readdirSync(new URL('robots/', root)).filter((f) => !f.startsWith('.')).map((f) => `robots/${f}`).sort();
        expect(manifest.assets.map((a) => a.file).sort()).toEqual(onDisk);
        for (const a of manifest.assets) {
            expect(a.licence, a.file).toMatch(/^[A-Za-z0-9.+-]+$/);
            expect(a.source.length, a.file).toBeGreaterThan(0);
            expect(createHash('sha256').update(readFileSync(new URL(a.file, root))).digest('hex'), a.file).toBe(a.sha256);
        }
    });

    it('every file names its licence in a header comment and loads without a warning other than the known ones', () => {
        for (const a of manifest.assets) {
            const text = readFileSync(new URL(a.file, root), 'utf8');
            expect(text, a.file).toContain(a.licence.replace('CC0-1.0', 'CC0 1.0'));
            const d = parseUrdf(text);
            expect(d.links.length, a.file).toBeGreaterThan(1);
            expect(d.warnings.filter((w) => !/cylinder is a capsule/.test(w)), a.file).toEqual([]);
        }
    });

    it('the quadruped has four legs of three joints and 10.6 kilograms', () => {
        const d = parseUrdf(readFileSync(new URL('robots/quadruped.urdf', root), 'utf8'));
        expect(d.joints).toHaveLength(12);
        expect(d.links.reduce((s, l) => s + (l.mass ?? 0), 0)).toBeCloseTo(3 + 4 * (0.5 + 0.8 + 0.6), 9);
    });
});
