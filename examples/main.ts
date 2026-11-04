import { FLOATS_PER_TRANSFORM, SHAPE_SPHERE, type EntityHandle } from '@dynova/sdk';
import { createWorld } from './shared';

const BOX_COUNT    = 30;
const SPHERE_COUNT = 20;
const BOX_HALF     = 0.5;
const SPHERE_R     = 0.4;
const CAPACITY     = BOX_COUNT + SPHERE_COUNT + 3;

const canvas    = document.getElementById('canvas') as HTMLCanvasElement;
const ctx       = canvas.getContext('2d')!;
const modeEl    = document.getElementById('mode')!;
const countEl   = document.getElementById('count')!;
const pendingEl = document.getElementById('pending')!;

canvas.width  = window.innerWidth;
canvas.height = window.innerHeight;

const SCALE = 20;
const cx = canvas.width  / 2;
const cy = canvas.height / 2;

async function main() {
    const world = await createWorld({ capacity: CAPACITY, ringCapacity: 1024 }, true);
    modeEl.textContent = `Mode: ${world.mode} — ${world.mode === 'shared' ? 'physics in worker' : 'physics on main thread'}`;

    const floor = world.spawn({ y: -15, hx: 40, hy: 0.5, hz: 40, invMass: 0, restitution: 0.1 });
    world.spawn({ x: -18, hx: 0.5, hy: 20, hz: 40, invMass: 0, restitution: 0.1 });
    world.spawn({ x:  18, hx: 0.5, hy: 20, hz: 40, invMass: 0, restitution: 0.1 });

    const COLS = 6;
    const boxes: EntityHandle[] = [];
    for (let i = 0; i < BOX_COUNT; i++) {
        const col = i % COLS;
        const row = Math.floor(i / COLS);
        boxes.push(world.spawn({
            x: (col - COLS / 2 + 0.5) * 1.2, y: -12 + row * 1.2,
            hx: BOX_HALF, hy: BOX_HALF, hz: BOX_HALF, invMass: 1, restitution: 0.3,
        }));
    }

    const spheres: EntityHandle[] = [];
    for (let i = 0; i < SPHERE_COUNT; i++) {
        spheres.push(world.spawn({
            x: 8 + (i % 4), y: -12 + Math.floor(i / 4),
            shape: SHAPE_SPHERE, hx: SPHERE_R, hy: SPHERE_R, hz: SPHERE_R, invMass: 1, restitution: 0.2,
        }));
    }

    (window as unknown as { __demo: unknown }).__demo = { world, boxes, spheres };
    let frameCount = 0;
    const screen = (buf: Float32Array, id: number): [number, number] => {
        const o = id * FLOATS_PER_TRANSFORM;
        return [buf[o] * SCALE + cx, -buf[o + 1] * SCALE + cy];
    };
    const live = (buf: Float32Array, id: number) => buf[id * FLOATS_PER_TRANSFORM + 6] !== 0;

    function frame() {
        if (world.mode === 'fallback') world.step();
        frameCount++;
        const buf = world.readTransforms();
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        if (live(buf, floor.id)) {
            const [sx, sy] = screen(buf, floor.id);
            ctx.fillStyle = '#445544';
            ctx.fillRect(sx - 40 * SCALE, sy - 0.5 * SCALE, 80 * SCALE, SCALE);
        }

        ctx.fillStyle = '#00ff88';
        for (const h of boxes) {
            if (!live(buf, h.id)) continue;
            const [sx, sy] = screen(buf, h.id);
            ctx.fillRect(sx - BOX_HALF * SCALE, sy - BOX_HALF * SCALE, 2 * BOX_HALF * SCALE, 2 * BOX_HALF * SCALE);
        }

        ctx.fillStyle = '#ff8844';
        for (const h of spheres) {
            if (!live(buf, h.id)) continue;
            const [sx, sy] = screen(buf, h.id);
            ctx.beginPath();
            ctx.arc(sx, sy, SPHERE_R * SCALE, 0, Math.PI * 2);
            ctx.fill();
        }

        countEl.textContent   = `Bodies: ${BOX_COUNT + SPHERE_COUNT}  frame: ${frameCount}`;
        pendingEl.textContent = `Pending commands: ${world.pendingCommands}`;
        requestAnimationFrame(frame);
    }

    requestAnimationFrame(frame);
}

main().catch((e) => { modeEl.textContent = String(e); console.error(e); });
