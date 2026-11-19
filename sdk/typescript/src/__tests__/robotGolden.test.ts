import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { parseUrdf, forwardKinematics } from '../robot.js';

// python/tests/golden_robot_ts.json is read by python/tests/test_robot.py: both languages must give these frames for these assets and joint values.
const GOLD = JSON.parse(readFileSync(new URL('../../../../python/tests/golden_robot_ts.json', import.meta.url), 'utf8')) as { cases: { asset: string; values: Record<string, number>; frames: Record<string, number[]> }[] };

describe('robot frames shared with the Python package', () => {
    it('forward kinematics of the shipped assets equals the pinned frames to 1e-12', () => {
        expect(GOLD.cases.length).toBeGreaterThan(2);
        for (const c of GOLD.cases) {
            const d = parseUrdf(readFileSync(new URL(`../../../../assets/robots/${c.asset}.urdf`, import.meta.url), 'utf8'));
            const f = forwardKinematics(d, c.values);
            expect(Object.keys(c.frames).sort()).toEqual(d.links.map((l) => l.name).sort());
            for (const [name, want] of Object.entries(c.frames)) {
                const fr = f.get(name)!;
                [...fr.p, ...fr.q].forEach((v, i) => expect(Math.abs(v - want[i]), `${c.asset} ${name}[${i}]`).toBeLessThan(1e-12));
            }
        }
    });
});
