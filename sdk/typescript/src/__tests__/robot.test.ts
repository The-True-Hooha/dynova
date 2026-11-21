import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { World, layoutFromRuntime } from '../World.js';
import { parseXml, XmlError } from '../xml.js';
import { parseUrdf, forwardKinematics, spawnRobot, compose, qrot, rpyToQuat, RobotError, type RobotDesc } from '../robot.js';
import { JOINT_HINGE, JOINT_SLIDER } from '../CommandRing.js';

const glue = await import('../../wasm-st/dynova_core.js');
const wasm = glue.initSync({ module: readFileSync(new URL('../../wasm-st/dynova_core_bg.wasm', import.meta.url)) });
const config = { capacity: 32, ringCapacity: 128 };

// A bolted arm: a yawing base, a pitching shoulder, an elbow turned by rpy, and a sliding tool; every origin is off-axis.
const ARM = `<?xml version="1.0"?>
<robot name="arm">
  <!-- links -->
  <link name="base"><inertial><mass value="5"/></inertial><collision><origin xyz="0 0 0.1"/><geometry><box size="0.4 0.4 0.2"/></geometry></collision></link>
  <link name="upper"><inertial><mass value="2"/></inertial><collision><origin xyz="0 0 0.3"/><geometry><box size="0.1 0.1 0.6"/></geometry></collision></link>
  <link name="fore"><inertial><mass value="1"/></inertial><collision><origin xyz="0 0 0.25" rpy="0 0 0"/><geometry><cylinder radius="0.05" length="0.5"/></geometry></collision></link>
  <link name="tool"><inertial><mass value="0.5"/></inertial><collision><geometry><sphere radius="0.06"/></geometry></collision></link>
  <joint name="yaw" type="revolute"><parent link="base"/><child link="upper"/><origin xyz="0 0 0.2"/><axis xyz="0 0 1"/><limit lower="-1.5" upper="1.5" effort="50" velocity="3"/></joint>
  <joint name="elbow" type="revolute"><parent link="upper"/><child link="fore"/><origin xyz="0.05 0 0.6" rpy="0 0.3 0.2"/><axis xyz="0 1 0"/><limit lower="-1.2" upper="1.2" effort="50" velocity="3"/></joint>
  <joint name="slide" type="prismatic"><parent link="fore"/><child link="tool"/><origin xyz="0 0 0.5" rpy="0.1 0 0"/><axis xyz="0 0 1"/><limit lower="0" upper="0.2" effort="50" velocity="1"/></joint>
</robot>`;

const dist3 = (a: readonly number[], b: readonly number[]) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
const quatGap = (a: readonly number[], b: readonly number[]) => 1 - Math.abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);

function world(): World {
    const rt = new glue.Runtime(config.capacity, config.ringCapacity);
    rt.enable_physics();
    const w = World.attachSync(config, wasm.memory, 'fallback', layoutFromRuntime(rt), rt);
    w.setGravity(0, 0, 0);
    return w;
}

describe('xml', () => {
    it('reads elements, attributes, text, comments, CDATA and entities', () => {
        const root = parseXml(`<?xml version="1.0"?><!-- c --><a x="1" y='2&amp;3'><b/><c>t &lt; <![CDATA[<raw>]]></c></a>`);
        expect(root.attrs).toEqual({ x: '1', y: '2&3' });
        expect(root.children.map((c) => c.name)).toEqual(['b', 'c']);
        expect(root.children[1].text).toBe('t < <raw>');
    });

    it.each([
        ['a doctype', '<!DOCTYPE a><a/>', /DOCTYPE/],
        ['an entity declaration', '<!ENTITY x "y"><a/>', /ENTITY/],
        ['a mismatched close', '<a><b></a></b>', /does not match/],
        ['an unclosed element', '<a><b>', /never closed/],
        ['two roots', '<a/><b/>', /more than one root/],
        ['text outside the root', 'x<a/>', /outside the root/],
        ['an unterminated comment', '<a><!-- </a>', /unterminated comment/],
        ['a duplicate attribute', '<a x="1" x="2"/>', /duplicate attribute/],
        ['an unquoted attribute', '<a x=1/>', /malformed tag/],
        ['an empty document', '  ', /no root/],
        ['deep nesting', '<a>'.repeat(100) + '</a>'.repeat(100), /nesting deeper/],
    ])('refuses %s', (_, text, message) => {
        expect(() => parseXml(text)).toThrow(XmlError);
        expect(() => parseXml(text)).toThrow(message);
    });
});

describe('parseUrdf', () => {
    it('reads links, shapes, masses and joints, and orders joints parent first', () => {
        const d = parseUrdf(ARM);
        expect(d.name).toBe('arm');
        expect(d.root).toBe('base');
        expect(d.links.map((l) => [l.name, l.mass, l.shape?.kind])).toEqual([['base', 5, 'box'], ['upper', 2, 'box'], ['fore', 1, 'capsule'], ['tool', 0.5, 'sphere']]);
        expect(d.links[0].shape?.half).toEqual([0.2, 0.2, 0.1]);
        expect(d.joints.map((j) => [j.name, j.type, j.limit])).toEqual([['yaw', 'revolute', { lower: -1.5, upper: 1.5 }], ['elbow', 'revolute', { lower: -1.2, upper: 1.2 }], ['slide', 'prismatic', { lower: 0, upper: 0.2 }]]);
        expect(d.warnings).toEqual(['link fore: the cylinder is a capsule with rounded ends']);
    });

    it('a cylinder becomes a capsule of the same radius and overall length, lying along the description\'s Z', () => {
        const s = parseUrdf(ARM).links[2].shape!;
        expect(s.radius).toBe(0.05);
        expect(s.halfLength).toBeCloseTo(0.2, 12);
        const along = qrot(s.frame.q, [0, 1, 0]);
        expect(along[0]).toBeCloseTo(0, 12);
        expect(along[1]).toBeCloseTo(0, 12);
        expect(along[2]).toBeCloseTo(1, 12);
    });

    it('orders a child before its parent in the file correctly', () => {
        const text = `<robot name="r"><link name="c"><collision><geometry><sphere radius="1"/></geometry></collision></link><link name="a"><collision><geometry><sphere radius="1"/></geometry></collision></link><link name="b"><collision><geometry><sphere radius="1"/></geometry></collision></link>
          <joint name="j2" type="fixed"><parent link="b"/><child link="c"/></joint><joint name="j1" type="fixed"><parent link="a"/><child link="b"/></joint></robot>`;
        const d = parseUrdf(text);
        expect(d.root).toBe('a');
        expect(d.order.map((j) => j.name)).toEqual(['j1', 'j2']);
    });

    const link = (n: string) => `<link name="${n}"><collision><geometry><sphere radius="1"/></geometry></collision></link>`;
    const joint = (n: string, p: string, c: string, extra = '', type = 'fixed') => `<joint name="${n}" type="${type}"><parent link="${p}"/><child link="${c}"/>${extra}</joint>`;
    it.each([
        ['a wrong root element', '<model/>', /not <robot>/],
        ['no links', '<robot/>', /no links/],
        ['a nameless link', '<robot><link/></robot>', /no name/],
        ['a duplicate link', `<robot>${link('a')}${link('a')}</robot>`, /defined twice/],
        ['an unknown parent', `<robot>${link('a')}${joint('j', 'zz', 'a')}</robot>`, /unknown parent/],
        ['an unknown child', `<robot>${link('a')}${joint('j', 'a', 'zz')}</robot>`, /unknown child/],
        ['two parents', `<robot>${link('a')}${link('b')}${link('c')}${joint('j1', 'a', 'c')}${joint('j2', 'b', 'c')}</robot>`, /two parent joints/],
        ['two roots', `<robot>${link('a')}${link('b')}</robot>`, /one root/],
        ['a loop', `<robot>${link('a')}${link('b')}${joint('j1', 'a', 'b')}${joint('j2', 'b', 'a')}</robot>`, /loop/],
        ['an unsupported joint type', `<robot>${link('a')}${link('b')}${joint('j', 'a', 'b', '', 'floating')}</robot>`, /not supported/],
        ['a revolute joint with no limit', `<robot>${link('a')}${link('b')}${joint('j', 'a', 'b', '', 'revolute')}</robot>`, /needs a <limit>/],
        ['inverted limits', `<robot>${link('a')}${link('b')}${joint('j', 'a', 'b', '<limit lower="1" upper="0"/>', 'revolute')}</robot>`, /above upper/],
        ['a zero axis', `<robot>${link('a')}${link('b')}${joint('j', 'a', 'b', '<axis xyz="0 0 0"/><limit lower="0" upper="1"/>', 'revolute')}</robot>`, /zero axis/],
        ['a non-numeric origin', `<robot>${link('a')}${link('b')}${joint('j', 'a', 'b', '<origin xyz="1 x 0"/>')}</robot>`, /3 finite numbers/],
        ['a short vector', `<robot>${link('a')}${link('b')}${joint('j', 'a', 'b', '<origin xyz="1 2"/>')}</robot>`, /3 finite numbers/],
        ['an infinite number', `<robot>${link('a')}${link('b')}${joint('j', 'a', 'b', '<origin xyz="1 Infinity 0"/>')}</robot>`, /3 finite numbers/],
        ['a negative mass', '<robot><link name="a"><inertial><mass value="-1"/></inertial></link></robot>', /must be positive/],
        ['a zero-size box', '<robot><link name="a"><collision><geometry><box size="1 0 1"/></geometry></collision></link></robot>', /must be positive/],
        ['a collision with no geometry', '<robot><link name="a"><collision/></link></robot>', /no geometry/],
    ])('refuses %s', (_, text, message) => {
        expect(() => parseUrdf(text)).toThrow(RobotError);
        expect(() => parseUrdf(text)).toThrow(message);
    });

    it.each(['0x10', '0b1', '1,5', '--1', '1e', 'NaN'])('refuses the number %s', (n) => {
        expect(() => parseUrdf(`<robot><link name="a"><collision><geometry><sphere radius="${n}"/></geometry></collision></link></robot>`)).toThrow(/finite numbers/);
    });
    it('takes padded, signed and exponent numbers', () => {
        const d = parseUrdf('<robot><link name="a"><collision><origin xyz=" +1  2e-1 .5 "/><geometry><sphere radius="1e-1"/></geometry></collision></link></robot>');
        expect(d.links[0].shape?.frame.p).toEqual([1, 0.2, 0.5]);
    });
    it('a joint named like an Object method has value 0, not a function', () => {
        const text = '<robot><link name="a"><collision><geometry><sphere radius="1"/></geometry></collision></link><link name="b"><collision><geometry><sphere radius="1"/></geometry></collision></link><joint name="constructor" type="fixed"><parent link="a"/><child link="b"/></joint></robot>';
        expect(() => forwardKinematics(parseUrdf(text))).not.toThrow();
    });

    it('drops what it cannot honour and says so', () => {
        const d = parseUrdf(`<robot name="r"><link name="a"><collision><geometry><mesh filename="a.stl"/></geometry></collision><collision><geometry><sphere radius="1"/></geometry></collision><collision><geometry><box size="1 1 1"/></geometry></collision></link></robot>`);
        expect(d.links[0].shape?.kind).toBe('sphere');
        expect(d.warnings).toEqual(['link a: a mesh collision is not supported and was dropped', 'link a: only the first of 2 collision shapes is used (a body has one shape)']);
    });
});

describe('xml hostile and unusual input', () => {
    it('a comment or CDATA may mention a doctype', () => {
        expect(parseXml('<a><!-- <!DOCTYPE x> --></a>').name).toBe('a');
        expect(parseXml('<a><![CDATA[<!ENTITY x>]]></a>').text).toBe('<!ENTITY x>');
    });
    it('attributes with prototype names are ordinary', () => {
        const el = parseXml('<a constructor="1" toString="2" __proto__="3"/>');
        expect(Object.keys(el.attrs)).toEqual(['constructor', 'toString', '__proto__']);
        expect(el.attrs.__proto__).toBe('3');
    });
    it.each([
        ['a < in an attribute value', '<a x="<b>"/>', /malformed tag/],
        ['an attribute on a closing tag', '<a></a x="1">', /malformed tag|bare/],
        ['a self-closing closing tag', '<a></a/>', /malformed tag|bare/],
        ['an unknown entity', '<a>&foo;</a>', /unknown entity/],
        ['a NUL character reference', '<a>&#0;</a>', /invalid character reference/],
        ['too many elements', '<a>' + '<b/>'.repeat(200_001) + '</a>', /more than 200000 elements/],
    ])('refuses %s', (_, text, message) => expect(() => parseXml(text)).toThrow(message));
    it('decodes numeric character references and takes nesting of exactly 64', () => {
        expect(parseXml('<a>&#65;&#x42;</a>').text).toBe('AB');
        expect(parseXml('<a>'.repeat(64) + '</a>'.repeat(64)).name).toBe('a');
    });
});

describe('forwardKinematics', () => {
    it('is the identity at the root and composes each joint origin, axis and value', () => {
        const d = parseUrdf(ARM);
        const f0 = forwardKinematics(d);
        expect(f0.get('base')).toEqual({ p: [0, 0, 0], q: [0, 0, 0, 1] });
        expect(f0.get('upper')!.p).toEqual([0, 0, 0.2]);
        const elbowOrigin = compose({ p: [0, 0, 0.2], q: [0, 0, 0, 1] }, { p: [0.05, 0, 0.6], q: rpyToQuat(0, 0.3, 0.2) });
        expect(dist3(f0.get('fore')!.p, elbowOrigin.p)).toBeLessThan(1e-12);
        // A yaw of 90 degrees swings the elbow's off-axis offset from +x to +y.
        const f1 = forwardKinematics(d, { yaw: Math.PI / 2 });
        expect(dist3(f1.get('fore')!.p, [0, 0.05, 0.8])).toBeLessThan(1e-12);
        // Sliding moves the tool along the fore link's own axis, which the elbow has turned.
        const slid = forwardKinematics(d, { elbow: 0.5, slide: 0.2 }).get('tool')!;
        const rest = forwardKinematics(d, { elbow: 0.5 }).get('tool')!;
        expect(dist3(slid.p, rest.p)).toBeCloseTo(0.2, 12);
        expect(() => forwardKinematics(d, { yaw: Number.NaN })).toThrow(/not finite/);
    });
});

describe('spawnRobot on the real wasm', () => {
    const sphere = (n: string, extra = '') => `<link name="${n}">${extra}<collision><geometry><sphere radius="0.1"/></geometry></collision></link>`;

    it('clamps a size or mass the engine would drop, so the body exists', () => {
        const text = `<robot name="w"><link name="a"><inertial><mass value="1e-7"/></inertial><collision><geometry><cylinder radius="0.1" length="0.05"/></geometry></collision></link></robot>`;
        const w = world();
        const bot = spawnRobot(w, parseUrdf(text), { position: [1, 2, 3], upAxis: 'z' });
        w.step();
        const h = bot.links.get('a')!;
        expect(Array.from(w.readTransforms().slice(h.id * 7, h.id * 7 + 3))).toEqual([1, 2, 3]);
        expect(bot.warnings.some((m) => /half length 0 was clamped to 0.001/.test(m))).toBe(true);
        expect(bot.warnings.some((m) => /mass 1e-7 was clamped to 0.000001/.test(m))).toBe(true);
        const thin = spawnRobot(w, parseUrdf('<robot><link name="p"><collision><geometry><box size="1 1 0.0001"/></geometry></collision></link></robot>'), { upAxis: 'z', fixedBase: true });
        expect(thin.warnings.some((m) => /half z .* clamped to 0.001/.test(m))).toBe(true);
        const heavy = spawnRobot(w, parseUrdf('<robot><link name="h"><inertial><mass value="1e300"/></inertial><collision><geometry><sphere radius="1"/></geometry></collision></link></robot>'), { upAxis: 'z', position: [9, 9, 9] });
        w.step();
        expect(heavy.warnings.some((m) => /mass 1e\+300 was clamped to 1e\+30/.test(m))).toBe(true);
    });

    it('a frame-only root works: the first bodies below it are the base', () => {
        const text = `<robot name="r"><link name="world"/>${sphere('a', '<inertial><mass value="1"/></inertial>')}${sphere('b', '<inertial><mass value="1"/></inertial>')}
          <joint name="mount" type="fixed"><parent link="world"/><child link="a"/><origin xyz="0 0 1"/></joint>
          <joint name="j" type="revolute"><parent link="a"/><child link="b"/><origin xyz="0.5 0 0"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/></joint></robot>`;
        const w = world();
        w.setGravity(0, -9.81, 0);
        const bot = spawnRobot(w, parseUrdf(text), { fixedBase: true, upAxis: 'z' });
        for (let t = 0; t < 120; t++) w.step();
        const a = bot.links.get('a')!;
        expect(Array.from(w.readTransforms().slice(a.id * 7, a.id * 7 + 3))).toEqual([0, 0, 1]);
        expect([...bot.joints.keys()]).toEqual(['j']);
    });

    it('warns when a frame link loses a motion or a mass', () => {
        const text = `<robot name="r">${sphere('a', '<inertial><mass value="1"/></inertial>')}<link name="f"><inertial><mass value="9"/></inertial></link>${sphere('b', '<inertial><mass value="1"/></inertial>')}
          <joint name="j1" type="revolute"><parent link="a"/><child link="f"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/></joint>
          <joint name="j2" type="fixed"><parent link="f"/><child link="b"/><origin xyz="1 0 0"/></joint></robot>`;
        const bot = spawnRobot(world(), parseUrdf(text), { fixedBase: true, upAxis: 'z' });
        expect(bot.warnings).toContain('link f: it has a mass but no collision shape, so the mass is dropped');
        expect(bot.warnings).toContain('joint j1: its child link f has no body, so the motion of this revolute joint is lost');
    });

    it('a default upright robot has its description\'s +Z along the engine\'s +Y', () => {
        const text = `<robot name="r">${sphere('a', '<inertial><mass value="1"/></inertial>')}${sphere('b', '<inertial><mass value="1"/></inertial>')}<joint name="j" type="fixed"><parent link="a"/><child link="b"/><origin xyz="0 0 1"/></joint></robot>`;
        const w = world();
        const bot = spawnRobot(w, parseUrdf(text), { fixedBase: true });
        w.step();
        const b = bot.links.get('b')!;
        const p = Array.from(w.readTransforms().slice(b.id * 7, b.id * 7 + 3));
        expect(dist3(p, [0, 1, 0])).toBeLessThan(1e-3);
    });

    it('a spawn that cannot finish destroys what it made and rethrows', () => {
        const links = Array.from({ length: 20 }, (_, i) => sphere(`l${i}`, '<inertial><mass value="1"/></inertial>')).join('');
        const joints = Array.from({ length: 19 }, (_, i) => `<joint name="j${i}" type="fixed"><parent link="l${i}"/><child link="l${i + 1}"/><origin xyz="1 0 0"/></joint>`).join('');
        const rt = new glue.Runtime(8, 128);
        rt.enable_physics();
        const w = World.attachSync({ capacity: 8, ringCapacity: 128 }, wasm.memory, 'fallback', layoutFromRuntime(rt), rt);
        expect(() => spawnRobot(w, parseUrdf(`<robot name="r">${links}${joints}</robot>`), { upAxis: 'z' })).toThrow(/capacity/);
        w.step();
        expect(w.readTransforms().slice(0, 8 * 7).every((v) => v === 0)).toBe(true);
    });

    it('places every link at its description frame, makes the right joints, and holds still with gravity off', () => {
        const w = world();
        const d = parseUrdf(ARM);
        const bot = spawnRobot(w, d, { position: [1, 2, 3], rotation: rpyToQuat(0.2, -0.4, 0.9), fixedBase: true, upAxis: 'z' });
        for (let t = 0; t < 30; t++) w.step();
        const tr = w.readTransforms();
        for (const l of d.links) {
            const h = bot.links.get(l.name)!;
            const want = compose(bot.frames.get(l.name)!, l.shape!.frame);
            expect(dist3(Array.from(tr.slice(h.id * 7, h.id * 7 + 3)), want.p), l.name).toBeLessThan(2e-3);
            expect(quatGap(Array.from(tr.slice(h.id * 7 + 3, h.id * 7 + 7)), want.q), l.name).toBeLessThan(1e-4);
        }
        expect([...bot.joints.keys()]).toEqual(['yaw', 'elbow', 'slide']);
        const kinds = w.readJointStates().map((s) => s.kind);
        expect(kinds).toEqual([JOINT_HINGE, JOINT_HINGE, JOINT_SLIDER]);
        expect(w.readJointStates().every((s) => Math.abs(s.position) < 1e-3)).toBe(true);
    });

    it('driven to joint values it takes the pose forward kinematics gives for them', () => {
        const w = world();
        const d = parseUrdf(ARM);
        const bot = spawnRobot(w, d, { fixedBase: true, upAxis: 'z' });
        const target = { yaw: 0.7, elbow: -0.6, slide: 0.12 };
        const gains = { kp: 6, kd: 0.5, maxSpeed: 2, maxForce: 200 };
        const drives = d.joints.map((j) => ({ joint: bot.joints.get(j.name)!, target: target[j.name as keyof typeof target], gains }));
        for (let t = 0; t < 900; t++) {
            w.driveJoints(drives);
            w.step();
        }
        const states = new Map(w.readJointStates().map((s) => [s.joint, s.position]));
        for (const j of d.joints) expect(states.get(bot.joints.get(j.name)!.id), j.name).toBeCloseTo(target[j.name as keyof typeof target], 2);
        const fk = forwardKinematics(d, target);
        const tr = w.readTransforms();
        for (const l of d.links) {
            const h = bot.links.get(l.name)!;
            const want = compose(fk.get(l.name)!, l.shape!.frame);
            expect(dist3(Array.from(tr.slice(h.id * 7, h.id * 7 + 3)), want.p), l.name).toBeLessThan(0.01);
            expect(quatGap(Array.from(tr.slice(h.id * 7 + 3, h.id * 7 + 7)), want.q), l.name).toBeLessThan(1e-3);
        }
    });

    it('a link with no shape is a frame: joints through it reach the nearest body above it', () => {
        const text = `<robot name="r">
          <link name="a"><inertial><mass value="1"/></inertial><collision><geometry><box size="1 1 1"/></geometry></collision></link>
          <link name="frame"/>
          <link name="b"><inertial><mass value="1"/></inertial><collision><geometry><box size="1 1 1"/></geometry></collision></link>
          <joint name="j1" type="fixed"><parent link="a"/><child link="frame"/><origin xyz="1 0 0"/></joint>
          <joint name="j2" type="revolute"><parent link="frame"/><child link="b"/><origin xyz="1 0 0"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/></joint></robot>`;
        const w = world();
        const bot = spawnRobot(w, parseUrdf(text), { fixedBase: true, upAxis: 'z' });
        expect([...bot.links.keys()]).toEqual(['a', 'b']);
        expect([...bot.joints.keys()]).toEqual(['j2']);
        for (let t = 0; t < 20; t++) w.step();
        const b = bot.links.get('b')!;
        expect(dist3(Array.from(w.readTransforms().slice(b.id * 7, b.id * 7 + 3)), [2, 0, 0])).toBeLessThan(1e-3);
    });

    it('clamps limits the engine cannot take, defaults a missing mass, and says so', () => {
        const text = `<robot name="r">
          <link name="a"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
          <link name="b"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
          <joint name="j" type="revolute"><parent link="a"/><child link="b"/><origin xyz="1 0 0"/><axis xyz="0 0 1"/><limit lower="0.5" upper="3.14"/></joint></robot>`;
        const bot = spawnRobot(world(), parseUrdf(text), { fixedBase: true, defaultMass: 4, upAxis: 'z' });
        expect(bot.warnings).toEqual(['link b: no mass given, using 4 kg', 'joint j: limits [0.5, 3.14] were clamped to [0, 3.1] (zero must lie inside, and the engine\'s bound is 3.1)']);
    });

    it('refuses a root with no shape, and reports a fixed joint made as one', () => {
        expect(() => spawnRobot(world(), parseUrdf('<robot name="r"><link name="a"/></robot>'))).toThrow(/nothing to spawn/);
        const text = `<robot name="r"><link name="a"><collision><geometry><box size="1 1 1"/></geometry></collision></link><link name="b"><inertial><mass value="1"/></inertial><collision><geometry><box size="1 1 1"/></geometry></collision></link><joint name="j" type="fixed"><parent link="a"/><child link="b"/><origin xyz="1 0 0"/></joint></robot>`;
        const w = world();
        w.setGravity(0, -9.81, 0);
        const bot = spawnRobot(w, parseUrdf(text), { fixedBase: true, upAxis: 'z' });
        expect(bot.joints.has('j')).toBe(true);
        for (let t = 0; t < 240; t++) w.step();
        const b = bot.links.get('b')!;
        expect(dist3(Array.from(w.readTransforms().slice(b.id * 7, b.id * 7 + 3)), [1, 0, 0])).toBeLessThan(0.02);
    });
});

export type { RobotDesc };
