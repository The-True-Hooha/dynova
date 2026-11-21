import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { World, layoutFromRuntime } from '../World.js';
import { parseMjcf } from '../mjcf.js';
import { forwardKinematics, spawnRobot, compose, qrot, axisAngle, RobotError } from '../robot.js';

const glue = await import('../../wasm-st/dynova_core.js');
const wasm = glue.initSync({ module: readFileSync(new URL('../../wasm-st/dynova_core_bg.wasm', import.meta.url)) });
const config = { capacity: 32, ringCapacity: 128 };
const dist3 = (a: readonly number[], b: readonly number[]) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

function world(): World {
    const rt = new glue.Runtime(config.capacity, config.ringCapacity);
    rt.enable_physics();
    const w = World.attachSync(config, wasm.memory, 'fallback', layoutFromRuntime(rt), rt);
    w.setGravity(0, 0, 0);
    return w;
}

// Degrees by default. The shoulder turns about a point 0.1 above its body origin, and the forearm is a capsule given by fromto.
const ARM = `<mujoco model="arm">
  <compiler angle="degree"/>
  <worldbody>
    <geom type="plane" size="5 5 0.1"/>
    <body name="base" pos="1 0 0.5"><geom type="box" size="0.2 0.2 0.1" mass="5"/>
      <body name="upper" pos="0 0 0.1" euler="0 0 90">
        <joint name="shoulder" type="hinge" axis="0 1 0" pos="0 0 0.1" range="-60 60"/>
        <geom type="capsule" fromto="0 0 0 0 0 0.5" size="0.05" mass="2"/>
        <body name="fore" pos="0.1 0 0.5"><joint name="slide" type="slide" axis="0 0 1" range="0 0.2"/><geom type="sphere" size="0.06" mass="0.5"/></body>
      </body>
    </body>
  </worldbody>
</mujoco>`;

describe('parseMjcf', () => {
    it('reads bodies, shapes, degrees, masses and the plane, with a frame root', () => {
        const d = parseMjcf(ARM);
        expect(d.root).toBe('world');
        expect(d.links.map((l) => [l.name, l.shape?.kind, l.mass, l.static ?? false])).toEqual([
            ['world', undefined, null, false], ['world_geom0', 'box', null, true], ['base', 'box', 5, true], ['upper', 'capsule', 2, false], ['fore', 'sphere', 0.5, false],
        ]);
        const shoulder = d.joints.find((j) => j.name === 'shoulder')!;
        expect(shoulder.type).toBe('revolute');
        expect(shoulder.limit!.lower).toBeCloseTo(-Math.PI / 3, 12);
        expect(d.joints.find((j) => j.name === 'slide')!.limit).toEqual({ lower: 0, upper: 0.2 });
        expect(d.links[1].shape!.half).toEqual([5, 5, 0.5]);
        expect(d.warnings).toEqual(["worldbody geom: MuJoCo's plane is infinite; here it is a 10 by 10 m slab (size 0 means 100 m)"]);
    });

    it('puts the link frame at the joint and shifts the body\'s contents back, so the geometry stays where it was written', () => {
        const d = parseMjcf(ARM);
        const f = forwardKinematics(d);
        // base body at (1,0,0.5); upper at base + (0,0,0.1), then its joint is 0.1 higher still.
        expect(dist3(f.get('upper')!.p, [1, 0, 0.7])).toBeLessThan(1e-12);
        // The capsule runs from the body origin (1,0,0.6) to 0.5 above it; its centre is at 1,0,0.85.
        const c = compose(f.get('upper')!, d.links[3].shape!.frame);
        expect(dist3(c.p, [1, 0, 0.85])).toBeLessThan(1e-12);
        // Its long axis is the engine capsule's local Y.
        expect(dist3(qrot(c.q, [0, 1, 0]), [0, 0, 1])).toBeLessThan(1e-12);
        expect(d.links[3].shape!.halfLength).toBeCloseTo(0.25, 12);
    });

    it('a nested body sits at its parent\'s body origin plus its own pos, not at the parent\'s joint', () => {
        const d = parseMjcf(ARM);
        // upper's body origin is (1,0,0.6) turned 90 degrees about z; (0.1,0,0.5) in that frame is (0,0.1,0.5) in the world.
        expect(dist3(forwardKinematics(d).get('fore')!.p, [1, 0.1, 1.1])).toBeLessThan(1e-12);
    });

    it('euler angles are intrinsic x then y then z, in degrees', () => {
        const rot = (axis: 'x' | 'y' | 'z', deg: number) => {
            const t = (deg * Math.PI) / 180, c = Math.cos(t), s = Math.sin(t);
            return axis === 'x' ? [[1, 0, 0], [0, c, -s], [0, s, c]] : axis === 'y' ? [[c, 0, s], [0, 1, 0], [-s, 0, c]] : [[c, -s, 0], [s, c, 0], [0, 0, 1]];
        };
        const mul = (a: number[][], b: number[][]) => a.map((row) => [0, 1, 2].map((j) => row[0] * b[0][j] + row[1] * b[1][j] + row[2] * b[2][j]));
        const R = mul(mul(rot('x', 30), rot('y', 40)), rot('z', 50));
        const v = [1, 2, 3];
        const want = R.map((row) => row[0] * v[0] + row[1] * v[1] + row[2] * v[2]);
        const d = parseMjcf('<mujoco><worldbody><body name="a" euler="30 40 50"><geom size="1"/></body></worldbody></mujoco>');
        // The body is welded at the top level, so its fixed joint carries the orientation.
        expect(dist3(qrot(d.joints[0].origin.q, v), want)).toBeLessThan(1e-12);
    });

    it('a hinge swings the geometry about the joint point, as the format defines it', () => {
        const d = parseMjcf(ARM);
        const theta = 0.5;
        const f = forwardKinematics(d, { shoulder: theta });
        const c = compose(f.get('upper')!, d.links[3].shape!.frame);
        // Hand computation: joint J = (1,0,0.7), capsule centre at rest g = (1,0,0.85), axis y (body is turned 90 degrees about z first, so the axis is body-y = world -x).
        const axisWorld = qrot(axisAngle([0, 0, 1], Math.PI / 2), [0, 1, 0]);
        const rel = [0, 0, 0.15];
        const turned = qrot(axisAngle(axisWorld, theta), rel);
        expect(dist3(c.p, [1 + turned[0], turned[1], 0.7 + turned[2]])).toBeLessThan(1e-12);
    });

    it('drives to joint values and matches forward kinematics on the real wasm', () => {
        const d = parseMjcf(ARM);
        const w = world();
        const bot = spawnRobot(w, d, { upAxis: 'z' });
        const target = { shoulder: 0.5, slide: 0.1 };
        const gains = { kp: 6, kd: 0.5, maxSpeed: 2, maxForce: 200 };
        const drives = (['shoulder', 'slide'] as const).map((n) => ({ joint: bot.joints.get(n)!, target: target[n], gains }));
        for (let t = 0; t < 900; t++) {
            w.driveJoints(drives);
            w.step();
        }
        const fk = forwardKinematics(d, target);
        const tr = w.readTransforms();
        for (const l of d.links.filter((k) => k.shape)) {
            const h = bot.links.get(l.name)!;
            const want = compose(fk.get(l.name)!, l.shape!.frame);
            expect(dist3(Array.from(tr.slice(h.id * 7, h.id * 7 + 3)), want.p), l.name).toBeLessThan(0.01);
        }
    });

    it('a top-level body with no joint is welded, a free joint leaves it free, a scenery geom never moves', () => {
        const text = `<mujoco><compiler angle="radian"/><worldbody><geom type="box" size="5 5 0.1"/>
          <body name="welded" pos="0 0 3"><geom type="sphere" size="0.1"/></body>
          <body name="loose" pos="2 0 3"><freejoint/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>`;
        const w = world();
        w.setGravity(0, -9.81, 0);
        const bot = spawnRobot(w, parseMjcf(text), { upAxis: 'y' });
        for (let t = 0; t < 60; t++) w.step();
        const y = (n: string) => w.readTransforms()[bot.links.get(n)!.id * 7 + 1];
        expect(y('welded')).toBeCloseTo(3, 3);
        expect(y('loose')).toBeLessThan(2.5);
        expect(y('world_geom0')).toBeCloseTo(0, 6);
    });

    const body = (inner: string) => `<mujoco><worldbody><body name="a">${inner}</body></worldbody></mujoco>`;
    it.each([
        ['a wrong root', '<robot/>', /not <mujoco>/],
        ['no worldbody', '<mujoco/>', /no <worldbody>/],
        ['an empty worldbody', '<mujoco><worldbody/></mujoco>', /no bodies or geoms/],
        ['an unknown element', '<mujoco><foo/></mujoco>', /not a MJCF element/],
        ['a bad angle unit', '<mujoco><compiler angle="turns"/><worldbody/></mujoco>', /degree or radian/],
        ['another euler sequence', '<mujoco><compiler eulerseq="zyx"/><worldbody/></mujoco>', /eulerseq/],
        ['two orientations', body('<geom size="1"/>').replace('name="a"', 'name="a" quat="1 0 0 0" euler="0 0 0"'), /twice/],
        ['an unsupported orientation', body('<geom size="1"/>').replace('name="a"', 'name="a" xyaxes="1 0 0 0 1 0"'), /not supported/],
        ['a zero quat', body('<geom size="1"/>').replace('name="a"', 'name="a" quat="0 0 0 0"'), /quat is zero/],
        ['two joints on one body', body('<joint type="hinge"/><joint type="slide" range="0 1"/><geom size="1"/>'), /one joint per body/],
        ['a ball joint', body('<joint type="ball"/><geom size="1"/>'), /not supported/],
        ['a limited joint with no range', body('<joint type="hinge" limited="true"/><geom size="1"/>'), /no range/],
        ['a bad limited value', body('<joint type="hinge" limited="yes"/><geom size="1"/>'), /limited/],
        ['a bad autolimits value', '<mujoco><compiler autolimits="maybe"/><worldbody/></mujoco>', /autolimits/],
        ['a reversed range', body('<joint type="hinge" range="2 1"/><geom size="1"/>'), /above its upper/],
        ['a zero axis', body('<joint type="hinge" axis="0 0 0"/><geom size="1"/>'), /zero axis/],
        ['a nested free joint', `<mujoco><worldbody><body name="a"><geom size="1"/><body name="b"><freejoint/><geom size="1"/></body></body></worldbody></mujoco>`, /top-level/],
        ['a negative size', body('<geom type="box" size="1 -1 1"/>'), /negative size|positive sizes/],
        ['a zero radius', body('<geom type="sphere" size="0"/>'), /positive sizes/],
        ['a bad number', body('<geom type="sphere" size="0x1"/>'), /finite numbers/],
        ['a negative mass', body('<geom size="1" mass="-1"/>'), /mass must be positive/],
        ['a duplicate body', `<mujoco><worldbody><body name="a"><geom size="1"/></body><body name="a"><geom size="1"/></body></worldbody></mujoco>`, /defined twice/],
        ['fromto on a box', body('<geom type="box" fromto="0 0 0 0 0 1" size="1"/>'), /only for capsule/],
        ['a zero-length fromto', body('<geom type="capsule" fromto="0 0 0 0 0 0" size="1"/>'), /no length/],
    ])('refuses %s', (_, text, message) => {
        expect(() => parseMjcf(text)).toThrow(RobotError);
        expect(() => parseMjcf(text)).toThrow(message);
    });

    it('density gives the mass of a shape, an inertial element overrides it, and unsupported geoms are dropped with a warning', () => {
        const d = parseMjcf(body('<geom type="box" size="0.5 0.5 0.5" density="500"/>'));
        expect(d.links[1].mass).toBeCloseTo(500, 9);
        const o = parseMjcf(body('<inertial mass="3" pos="0 0 0"/><geom type="box" size="0.5 0.5 0.5"/>'));
        expect(o.links[1].mass).toBe(3);
        const m = parseMjcf(body('<geom type="mesh" mesh="x"/><geom type="sphere" size="0.1"/>'));
        expect(m.links[1].shape!.kind).toBe('sphere');
        expect(m.warnings).toEqual(['body a geom: a mesh geom is not supported and was dropped']);
    });

    it('fromto along every direction (up, down, sideways) gives a capsule whose Y axis lies along the segment', () => {
        for (const [seg, dir] of [['0 0 0 0 0 1', [0, 0, 1]], ['0 0 1 0 0 0', [0, 0, -1]], ['0 0 0 1 0 0', [1, 0, 0]], ['0 0 0 0 1 0', [0, 1, 0]], ['0 0 0 1 1 1', [0.577350269, 0.577350269, 0.577350269]]] as const) {
            const d = parseMjcf(body(`<geom type="capsule" fromto="${seg}" size="0.1"/>`));
            const along = qrot(d.links[1].shape!.frame.q, [0, 1, 0]);
            const sign = Math.sign(along[0] * dir[0] + along[1] * dir[1] + along[2] * dir[2]);
            expect(dist3(along.map((v) => v * sign), dir), seg).toBeLessThan(1e-6);
        }
    });

    it('the mass of a cylinder is that of the cylinder, and masses of several geoms add up', () => {
        const c = parseMjcf(body('<geom type="cylinder" size="0.1 0.1"/>'));
        expect(c.links[1].mass).toBeCloseTo(1000 * Math.PI * 0.01 * 0.2, 9);
        const two = parseMjcf(body('<geom size="1" mass="1"/><geom type="box" size="1 1 1" mass="9"/>'));
        expect(two.links[1].mass).toBe(10);
        expect(two.warnings[0]).toMatch(/masses are summed/);
        expect(parseMjcf(body('<inertial mass="2" pos="0 0 1" diaginertia="1 1 1"/><geom size="1"/>')).warnings[0]).toMatch(/inertia are ignored/);
    });

    it('limited, autolimits and unlimited slides follow the format', () => {
        const hinge = (attrs: string, compiler = '') => parseMjcf(`<mujoco>${compiler}<worldbody><body name="a"><joint type="hinge" ${attrs}/><geom size="1"/></body></worldbody></mujoco>`).joints[0];
        expect(hinge('range="-90 90"').limit?.upper).toBeCloseTo(Math.PI / 2, 12);
        expect(hinge('range="-90 90" limited="false"').limit).toBeNull();
        expect(hinge('range="-90 90" limited="true"', '<compiler autolimits="false"/>').type).toBe('revolute');
        expect(hinge('range="-90 90"', '<compiler autolimits="false"/>').type).toBe('continuous');
        const slide = parseMjcf(body('<joint type="slide"/><geom size="1"/>')).joints[0];
        expect([slide.type, slide.limit]).toEqual(['prismatic', null]);
        expect(parseMjcf(body('<joint type="slide" range="1 2"/><geom size="1"/>')).joints[0].limit).toEqual({ lower: 1, upper: 2 });
    });

    it('made-up names never take a name the document states, and scenery counts toward the limit', () => {
        const d = parseMjcf('<mujoco><worldbody><body name="world_geom0"><geom size="1"/></body><geom size="1"/><body><geom size="1"/></body><body name="body3"><geom size="1"/></body></worldbody></mujoco>');
        expect(new Set(d.links.map((l) => l.name)).size).toBe(d.links.length);
        expect(new Set(d.joints.map((j) => j.name)).size).toBe(d.joints.length);
        expect(d.links.map((l) => l.name)).toContain('world_geom0');
        const named = parseMjcf('<mujoco><worldbody><body name="a"><geom size="1"/><body name="b"><joint name="a_joint"/><geom size="1"/></body></body></worldbody></mujoco>');
        expect(new Set(named.joints.map((j) => j.name)).size).toBe(2);
        expect(() => parseMjcf(`<mujoco><worldbody>${'<geom size="1"/>'.repeat(600)}</worldbody></mujoco>`)).toThrow(/more than 512/);
    });

    it('a pendulum hinged on the world stays pinned there instead of falling', () => {
        const text = '<mujoco><compiler angle="radian"/><worldbody><body name="p" pos="0 0 2"><joint name="pivot" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.5 0 0" size="0.05" mass="1"/></body></worldbody></mujoco>';
        const w = world();
        w.setGravity(0, -9.81, 0);
        const bot = spawnRobot(w, parseMjcf(text), { upAxis: 'y' });
        expect(bot.anchors).toHaveLength(1);
        expect(bot.warnings.some((m) => /static anchor was made at the joint/.test(m))).toBe(true);
        for (let t = 0; t < 600; t++) w.step();
        const tr = w.readTransforms();
        const p = bot.links.get('p')!;
        const anchor = bot.anchors[0];
        // The pivot is at (0, 2, 0) in the engine; the capsule hangs 0.25 m from it whatever the angle.
        const c = Array.from(tr.slice(p.id * 7, p.id * 7 + 3));
        expect(dist3(c, Array.from(tr.slice(anchor.id * 7, anchor.id * 7 + 3)))).toBeCloseTo(0.25, 1);
        expect(c[1]).toBeGreaterThan(1.5);
    });

    it('fixedBase does not freeze a body that hangs on a hinge', () => {
        const text = '<mujoco><compiler angle="radian"/><worldbody><body name="p" pos="0 0 2"><joint axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.5 0 0" size="0.05" mass="1"/></body></worldbody></mujoco>';
        const w = world();
        w.setGravity(0, -9.81, 0);
        const bot = spawnRobot(w, parseMjcf(text), { upAxis: 'y', fixedBase: true });
        for (let t = 0; t < 30; t++) w.step();
        expect(w.readTransforms()[bot.links.get('p')!.id * 7 + 1]).toBeLessThan(1.95);
    });

    it('a plane tilted in its body keeps its top face as the plane', () => {
        const d = parseMjcf('<mujoco><worldbody><geom type="plane" size="1 1 0.1" pos="0 0 1"/></worldbody></mujoco>');
        const s = d.links[1].shape!;
        // The slab is 1 m thick and its top face is at z = 1, facing +Z: centre at z = 0.5.
        expect(s.half).toEqual([1, 1, 0.5]);
        expect(dist3(s.frame.p, [0, 0, 0.5])).toBeLessThan(1e-12);
    });
});
