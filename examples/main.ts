import wasmInit, { Runtime } from '../sdk/typescript/wasm/dynova_core.js';
import { FLOATS_PER_TRANSFORM, SLOT_BYTES } from '@dynova/sdk';

const BOX_COUNT     = 30;
const SPHERE_COUNT  = 20;
const RING_CAPACITY = 1024;
const CAPACITY      = BOX_COUNT + SPHERE_COUNT + 3; // floor + left wall + right wall

const canvas    = document.getElementById('canvas') as HTMLCanvasElement;
const ctx       = canvas.getContext('2d')!;
const modeEl    = document.getElementById('mode')!;
const countEl   = document.getElementById('count')!;
const pendingEl = document.getElementById('pending')!;

canvas.width  = window.innerWidth;
canvas.height = window.innerHeight;

// Physics world is centred at canvas centre, 1 unit = 20px
const SCALE = 20;
const cx = canvas.width  / 2;
const cy = canvas.height / 2;

async function main() {
    const exports = await wasmInit(
        new URL('../sdk/typescript/wasm/dynova_core_bg.wasm', import.meta.url)
    ) as unknown as { memory: WebAssembly.Memory };

    const memory  = exports.memory;
    const runtime = new Runtime(CAPACITY, RING_CAPACITY);

    runtime.enable_physics();
    runtime.set_gravity(0, -9.81, 0);

    const aOffset        = runtime.transforms_buf_a_ptr();
    const bOffset        = runtime.transforms_buf_b_ptr();
    const writeIdxOffset = runtime.transforms_write_index_ptr();
    const ringHeadOffset = runtime.ring_head_ptr();
    const ringTailOffset = runtime.ring_tail_ptr();
    const ringSlotsOffset= runtime.ring_slots_ptr();

    const writeIndex  = new Uint32Array(memory.buffer, writeIdxOffset, 1);
    const bufA        = new Float32Array(memory.buffer, aOffset, CAPACITY * FLOATS_PER_TRANSFORM);
    const bufB        = new Float32Array(memory.buffer, bOffset, CAPACITY * FLOATS_PER_TRANSFORM);
    const ringHeadView= new Uint32Array(memory.buffer, ringHeadOffset, 1);
    const wasmRingTail= new Uint32Array(memory.buffer, ringTailOffset, 1);
    const wasmRingView= new DataView(memory.buffer, ringSlotsOffset, RING_CAPACITY * SLOT_BYTES);
    const wasmMask    = RING_CAPACITY - 1;

    // id=0: static floor; id=51,52: left/right walls
    const WALL_ID_L = BOX_COUNT + SPHERE_COUNT + 1;
    const WALL_ID_R = BOX_COUNT + SPHERE_COUNT + 2;
    runtime.spawn_box(0,          0,   -15, 0,  40, 0.5, 40,  0.0, 0.1);
    runtime.spawn_box(WALL_ID_L, -18,   0,  0,  0.5, 20, 40,  0.0, 0.1);
    runtime.spawn_box(WALL_ID_R,  18,   0,  0,  0.5, 20, 40,  0.0, 0.1);

    // ids 1..BOX_COUNT: boxes in a grid, no overlaps
    const COLS = 6;
    for (let i = 1; i <= BOX_COUNT; i++) {
        const col = (i - 1) % COLS;
        const row = Math.floor((i - 1) / COLS);
        const x = (col - COLS / 2 + 0.5) * 1.2;
        const y = -12 + row * 1.2;
        runtime.spawn_box(i, x, y, 0,  0.5, 0.5, 0.5,  1.0, 0.3);
    }

    // ids BOX_COUNT+1..BOX_COUNT+SPHERE_COUNT: spheres offset to the right
    for (let i = 1; i <= SPHERE_COUNT; i++) {
        const col = (i - 1) % 4;
        const row = Math.floor((i - 1) / 4);
        const x = 8 + col * 1.0;
        const y = -12 + row * 1.0;
        runtime.spawn_sphere(BOX_COUNT + i, x, y, 0,  0.4,  1.0, 0.2);
    }

    runtime.step();

    const useWorker = false; // physics demo: main-thread sim only for now
    modeEl.textContent = 'Physics: Stage 2B — 30 boxes + 20 spheres, warm start, sleeping';

    // Shared ring for future worker use (kept alive but unused this demo)
    const SHARED_BYTES = RING_CAPACITY * SLOT_BYTES + 8;
    const sharedBuf = typeof SharedArrayBuffer !== 'undefined'
        ? (() => { try { return new SharedArrayBuffer(SHARED_BYTES); } catch { return null; } })()
        : null;
    const sharedHead  = sharedBuf ? new Int32Array(sharedBuf, RING_CAPACITY * SLOT_BYTES,     1) : null;
    const sharedTail  = sharedBuf ? new Int32Array(sharedBuf, RING_CAPACITY * SLOT_BYTES + 4, 1) : null;
    const sharedSlots = sharedBuf ? new Uint8Array(sharedBuf, 0, RING_CAPACITY * SLOT_BYTES)  : null;

    function bridgeSharedRing() {
        if (!sharedHead || !sharedTail || !sharedSlots) return;
        const head    = Atomics.load(sharedHead, 0);
        const tail    = Atomics.load(sharedTail, 0);
        const pending = (tail - head) >>> 0;
        for (let i = 0; i < pending; i++) {
            const srcOff = ((head + i) & wasmMask) * SLOT_BYTES;
            const dstTail = wasmRingTail[0];
            const dstOff  = (dstTail & wasmMask) * SLOT_BYTES;
            for (let b = 0; b < SLOT_BYTES; b++) {
                wasmRingView.setUint8(dstOff + b, sharedSlots[srcOff + b]);
            }
            wasmRingTail[0] = dstTail + 1;
        }
        if (pending > 0) Atomics.store(sharedHead, 0, tail);
    }

    let frameCount = 0;

    function frame() {
        bridgeSharedRing();
        runtime.step();
        frameCount++;

        ctx.clearRect(0, 0, canvas.width, canvas.height);

        const buf = writeIndex[0] === 0 ? bufB : bufA;

        // Draw floor (id=0) as a rectangle
        {
            const o = 0 * FLOATS_PER_TRANSFORM;
            const sx = buf[o]     * SCALE + cx;
            const sy = -buf[o+1]  * SCALE + cy;
            ctx.fillStyle = '#445544';
            ctx.fillRect(sx - 40 * SCALE, sy - 0.5 * SCALE, 80 * SCALE, 1 * SCALE);
        }

        // Draw dynamic boxes
        ctx.fillStyle = '#00ff88';
        for (let i = 1; i <= BOX_COUNT; i++) {
            const o = i * FLOATS_PER_TRANSFORM;
            const sx = buf[o]    * SCALE + cx;
            const sy = -buf[o+1] * SCALE + cy;
            ctx.fillRect(sx - 0.5 * SCALE, sy - 0.5 * SCALE, SCALE, SCALE);
        }

        // Draw spheres
        ctx.fillStyle = '#ff8844';
        for (let i = 1; i <= SPHERE_COUNT; i++) {
            const id = BOX_COUNT + i;
            const o = id * FLOATS_PER_TRANSFORM;
            const sx = buf[o]    * SCALE + cx;
            const sy = -buf[o+1] * SCALE + cy;
            const r  = 0.4 * SCALE;
            ctx.beginPath();
            ctx.arc(sx, sy, r, 0, Math.PI * 2);
            ctx.fill();
        }

        countEl.textContent   = `Bodies: ${BOX_COUNT + SPHERE_COUNT}  frame: ${frameCount}  sleeping: ${runtime.sleeping_count()}`;
        pendingEl.textContent = `Contacts: ${runtime.contact_count()}`;
        requestAnimationFrame(frame);
    }

    requestAnimationFrame(frame);
}

main().catch(console.error);
