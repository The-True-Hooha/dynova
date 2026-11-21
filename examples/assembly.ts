import { Assembly, World, layoutFromRuntime } from '@dynova/sdk';
import { canvas2dAdapter } from '@dynova/canvas2d';

const params = new URLSearchParams(location.search);
const OFFSET = Number(params.get('offset') ?? 0);
const FLOAT = params.get('float') !== '0';
const paused = params.has('paused');
if (!Number.isFinite(OFFSET)) throw new RangeError('offset must be a number');

const PPU = 120;
const ROD_FROM: [number, number] = [1, 0];
const CARRIAGE: [number, number] = [3, 0.6];
const GAINS = { kp: 4, kd: 0.2, maxSpeed: 2, maxForce: 200 };

/** Position of the carriage along the rail for a crank angle, by closing the triangle the connecting rod makes. */
export function carriageX(angle: number): number {
    const rod = Math.hypot(CARRIAGE[0] - ROD_FROM[0], CARRIAGE[1] - ROD_FROM[1]);
    const tip: [number, number] = [Math.cos(angle), Math.sin(angle)];
    return tip[0] + Math.sqrt(rod * rod - (CARRIAGE[1] - tip[1]) ** 2);
}

async function run() {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    const config = { capacity: 16, ringCapacity: 256 };
    const rt = new glue.Runtime(config.capacity, config.ringCapacity);
    rt.enable_physics();
    const world = World.attachSync(config, exports.memory, 'fallback', layoutFromRuntime(rt), rt);
    world.setGravity(0, 0, 0);
    world.record(true);

    // Every coordinate below is "where the model really is": the offset is added here and removed again by the assembly's origin.
    const at = (x: number, y: number, z = 0): [number, number, number] => [x + OFFSET, y + OFFSET, z + OFFSET];
    const asm = new Assembly(world, FLOAT ? [OFFSET, OFFSET, OFFSET] : [0, 0, 0]);
    const scales = new Float32Array(config.capacity * 3).fill(1);
    const styles = new Map<number, string>();
    const part = (name: string, p: [number, number], half: [number, number], color: string, invMass = 1) => {
        const [x, y, z] = at(p[0], p[1]);
        const h = asm.addPart(name, { x, y, z, hx: half[0], hy: half[1], hz: 0.1, invMass, friction: 0 });
        scales.set([half[0] * 2, half[1] * 2, 1], h.id * 3);
        styles.set(h.id, color);
    };
    part('base', [-0.3, 0], [0.1, 0.25], '#555d6e', 0);
    part('crank', [0.5, 0], [0.44, 0.05], '#44aaff');
    part('carriage', CARRIAGE, [0.3, 0.2], '#ffaa44');
    asm.addMate('crank', { type: 'revolute', a: 'base', b: 'crank', at: at(0, 0), axis: [0, 0, 1], range: [-1.2, 1.2] });
    asm.addMate('rail', { type: 'slider', a: 'base', b: 'carriage', at: at(CARRIAGE[0], CARRIAGE[1]), axis: [1, 0, 0], range: [-1.5, 0.5] });
    asm.addMate('rod', { type: 'distance', a: 'crank', b: 'carriage', pointA: at(ROD_FROM[0], ROD_FROM[1]), pointB: at(CARRIAGE[0], CARRIAGE[1]) });

    const canvas = document.getElementById('view') as HTMLCanvasElement;
    const ctx = canvas.getContext('2d')!;
    const view = { pixelsPerUnit: PPU, originX: 120, originY: canvas.height / 2 };
    const adapter = canvas2dAdapter(world, ctx, { scales, view, style: (id) => ({ color: styles.get(id) ?? '#333b4a' }) });
    const status = document.getElementById('status')!;

    let target = 0;
    const step = (n = 1) => {
        for (let i = 0; i < n; i++) {
            world.step();
            asm.drive({ crank: target }, GAINS);
        }
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        adapter.update();
        const s = asm.states();
        status.textContent = `offset ${OFFSET}  crank ${s.crank?.position.toFixed(3) ?? '-'} rad  rail ${s.rail?.position.toFixed(3) ?? '-'} m  rod stretch ${s.rod?.position.toFixed(4) ?? '-'} m`;
    };
    step(1);
    const loop = () => {
        step();
        requestAnimationFrame(loop);
    };
    if (!paused) requestAnimationFrame(loop);

    (window as any).__asm = {
        world, asm, step, offset: OFFSET, floating: FLOAT,
        setTarget: (t: number) => { target = t; },
        carriageX,
        carriageGlobalX: () => asm.toGlobal([world.readTransforms()[asm.part('carriage').id * 7], 0, 0])[0] - OFFSET,
    };
    (window as any).__ready = true;
}

run().catch((e) => {
    (window as any).__error = String(e);
    document.body.append(String(e));
});
