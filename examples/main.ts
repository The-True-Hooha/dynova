import wasmInit, { Runtime } from '../sdk/typescript/wasm/dynova_core.js';
import { FLOATS_PER_TRANSFORM } from '@dynova/sdk';

const ENTITY_COUNT  = 500;
const RING_CAPACITY = 1024;

const canvas = document.getElementById('canvas') as HTMLCanvasElement;
const ctx    = canvas.getContext('2d')!;
const modeEl    = document.getElementById('mode')!;
const countEl   = document.getElementById('count')!;
const pendingEl = document.getElementById('pending')!;

canvas.width  = window.innerWidth;
canvas.height = window.innerHeight;

const cx = canvas.width  / 2;
const cy = canvas.height / 2;

async function main() {
    const wasmExports = await wasmInit(
        new URL('../sdk/typescript/wasm/dynova_core_bg.wasm', import.meta.url)
    ) as unknown as { memory: WebAssembly.Memory };

    const memory = wasmExports.memory;
    modeEl.textContent = `Memory mode: main-thread WASM (Stage 1A)`;

    const runtime = new Runtime(ENTITY_COUNT, RING_CAPACITY);

    const aOffset          = runtime.transforms_buf_a_ptr();
    const bOffset          = runtime.transforms_buf_b_ptr();
    const writeIndexOffset = runtime.transforms_write_index_ptr();
    const ringSlotsOffset  = runtime.ring_slots_ptr();
    const ringHeadOffset   = runtime.ring_head_ptr();
    const ringTailOffset   = runtime.ring_tail_ptr();

    const ringTail = new Uint32Array(memory.buffer, ringTailOffset, 1);
    const writeIndex = new Uint32Array(memory.buffer, writeIndexOffset, 1);
    const bufA = new Float32Array(memory.buffer, aOffset, ENTITY_COUNT * FLOATS_PER_TRANSFORM);
    const bufB = new Float32Array(memory.buffer, bOffset, ENTITY_COUNT * FLOATS_PER_TRANSFORM);

    const ringSlots = new Uint8Array(memory.buffer, ringSlotsOffset, RING_CAPACITY * 40);
    const ringView  = new DataView(memory.buffer, ringSlotsOffset, RING_CAPACITY * 40);
    const mask = RING_CAPACITY - 1;

    function pushSpawn(id: number, x: number, y: number, z: number) {
        const tail = ringTail[0];
        const off  = (tail & mask) * 40;
        ringView.setUint8(off, 0);
        ringView.setUint32(off + 1,  id, true);
        ringView.setFloat32(off + 5,  x,  true);
        ringView.setFloat32(off + 9,  y,  true);
        ringView.setFloat32(off + 13, z,  true);
        ringView.setFloat32(off + 17, 0,  true);
        ringView.setFloat32(off + 21, 0,  true);
        ringView.setFloat32(off + 25, 0,  true);
        ringView.setFloat32(off + 29, 1,  true);
        ringTail[0] = tail + 1;
    }

    for (let i = 0; i < ENTITY_COUNT; i++) {
        const x = (Math.random() * 2 - 1) * cx * 0.9;
        const y = (Math.random() * 2 - 1) * cy * 0.9;
        pushSpawn(i, x, y, 0);
    }

    runtime.step();

    function frame() {
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        const buf = writeIndex[0] === 0 ? bufB : bufA;

        ctx.fillStyle = '#00ff88';
        for (let i = 0; i < ENTITY_COUNT; i++) {
            const o = i * FLOATS_PER_TRANSFORM;
            ctx.beginPath();
            ctx.arc(buf[o] + cx, buf[o + 1] + cy, 3, 0, Math.PI * 2);
            ctx.fill();
        }

        countEl.textContent   = `Entities: ${ENTITY_COUNT}`;
        pendingEl.textContent = `Ring head: ${new Uint32Array(memory.buffer, ringHeadOffset, 1)[0]}`;
        requestAnimationFrame(frame);
    }

    requestAnimationFrame(frame);
}

main().catch(console.error);
