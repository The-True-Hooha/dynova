import { World, layoutFromRuntime, parseUrdf, spawnRobot, forwardKinematics, type RobotDesc, type SpawnedRobot } from '@dynova/sdk';
import armText from '../assets/robots/arm.urdf?raw';
import quadrupedText from '../assets/robots/quadruped.urdf?raw';
import humanoidText from '../assets/robots/humanoid.urdf?raw';

const gains = { kp: 10, kd: 0.3, maxSpeed: 6 };

async function setup() {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    const rt = new glue.Runtime(64, 1024);
    rt.enable_physics();
    return World.attachSync({ capacity: 64, ringCapacity: 1024 }, exports.memory, 'fallback', layoutFromRuntime(rt), rt);
}

function tilt(w: World, bot: SpawnedRobot, link: string): { y: number; tilt: number; drift: number } {
    const tr = w.readTransforms();
    const o = bot.links.get(link)!.id * 7;
    const [x, y, z, qx, qy, qz, qw] = Array.from(tr.slice(o, o + 7));
    // The y component of the body's local z axis in the world.
    const upY = 2 * (qy * qz - qw * qx);
    return { y, tilt: Math.acos(Math.max(-1, Math.min(1, upY))), drift: Math.hypot(x, z) };
}

async function stand(text: string, base: string, position: [number, number, number], maxForce: number, ticks: number) {
    const w = await setup();
    w.spawn({ y: -1, hx: 100, hy: 1, hz: 50, invMass: 0, friction: 1 });
    const desc: RobotDesc = parseUrdf(text);
    const bot = spawnRobot(w, desc, { position, friction: 1 });
    const driven = desc.joints.filter((j) => j.type !== 'fixed');
    for (let t = 0; t < ticks; t++) {
        w.driveJoints(driven.map((j) => ({ joint: bot.joints.get(j.name)!, target: 0, gains: { ...gains, maxForce } })));
        w.step();
    }
    return { ...tilt(w, bot, base), warnings: bot.warnings.length };
}

async function arm() {
    const w = await setup();
    w.setGravity(0, 0, 0);
    const desc = parseUrdf(armText);
    const bot = spawnRobot(w, desc, { fixedBase: true, upAxis: 'z' });
    const target: Record<string, number> = { yaw: 0.7, elbow: -0.6, slide: 0.12 };
    for (let t = 0; t < 900; t++) {
        w.driveJoints(desc.joints.map((j) => ({ joint: bot.joints.get(j.name)!, target: target[j.name], gains: { kp: 6, kd: 0.5, maxSpeed: 2, maxForce: 200 } })));
        w.step();
    }
    const fk = forwardKinematics(desc, target);
    const tr = w.readTransforms();
    let worst = 0;
    for (const l of desc.links) {
        const f = fk.get(l.name)!;
        const s = l.shape!.frame;
        // position of the shape centre: link frame * shape frame (rotation of the link applied to the shape offset)
        const [x, y, z, w4] = f.q;
        const v = s.p;
        const t2 = [2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])];
        const p = [f.p[0] + v[0] + w4 * t2[0] + (y * t2[2] - z * t2[1]), f.p[1] + v[1] + w4 * t2[1] + (z * t2[0] - x * t2[2]), f.p[2] + v[2] + w4 * t2[2] + (x * t2[1] - y * t2[0])];
        const o = bot.links.get(l.name)!.id * 7;
        worst = Math.max(worst, Math.hypot(tr[o] - p[0], tr[o + 1] - p[1], tr[o + 2] - p[2]));
    }
    return { worstError: worst };
}

async function run() {
    return { arm: await arm(), quadruped: await stand(quadrupedText, 'trunk', [0, 0.545, 0], 40, 3000), humanoid: await stand(humanoidText, 'pelvis', [0, 0.94, 0], 150, 3000), userAgent: navigator.userAgent };
}

run().then(
    (r) => { (window as any).__result = r; document.getElementById('out')!.textContent = JSON.stringify(r); },
    (e) => { (window as any).__result = { error: String(e) }; document.getElementById('out')!.textContent = String(e); },
);
