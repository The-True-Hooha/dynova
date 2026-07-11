import wasmInit, { Runtime } from '../sdk/typescript/wasm/dynova_core.js';
import { FLOATS_PER_TRANSFORM, SLOT_BYTES } from '@dynova/sdk';

const ENTITY_COUNT  = 500;
const RING_CAPACITY = 1024;

const canvas    = document.getElementById('canvas') as HTMLCanvasElement;
const ctx       = canvas.getContext('2d')!;
const modeEl    = document.getElementById('mode')!;
const countEl   = document.getElementById('count')!;
const pendingEl = document.getElementById('pending')!;

canvas.width  = window.innerWidth;
canvas.height = window.innerHeight;

const cx = canvas.width  / 2;
const cy = canvas.height / 2;

const useWorker = typeof SharedArrayBuffer !== 'undefined' && (() => {
    try { new SharedArrayBuffer(4); return true; } catch { return false; }
})();

async function main() {
    const exports = await wasmInit(
        new URL('../sdk/typescript/wasm/dynova_core_bg.wasm', import.meta.url)
    ) as unknown as { memory: WebAssembly.Memory };

    const memory  = exports.memory;
    const runtime = new Runtime(ENTITY_COUNT, RING_CAPACITY);

    const aOffset         = runtime.transforms_buf_a_ptr();
    const bOffset         = runtime.transforms_buf_b_ptr();
    const writeIdxOffset  = runtime.transforms_write_index_ptr();
    const ringSlotsOffset = runtime.ring_slots_ptr();
    const ringHeadOffset  = runtime.ring_head_ptr();
    const ringTailOffset  = runtime.ring_tail_ptr();

    const writeIndex    = new Uint32Array(memory.buffer, writeIdxOffset, 1);
    const bufA          = new Float32Array(memory.buffer, aOffset, ENTITY_COUNT * FLOATS_PER_TRANSFORM);
    const bufB          = new Float32Array(memory.buffer, bOffset, ENTITY_COUNT * FLOATS_PER_TRANSFORM);
    const wasmRingTail  = new Uint32Array(memory.buffer, ringTailOffset, 1);
    const wasmRingView  = new DataView(memory.buffer, ringSlotsOffset, RING_CAPACITY * SLOT_BYTES);
    const wasmMask      = RING_CAPACITY - 1;

    // Shared ring: separate SAB, layout [slots | head(4) | tail(4)]
    const SHARED_BYTES  = RING_CAPACITY * SLOT_BYTES + 8;
    const sharedBuf     = useWorker ? new SharedArrayBuffer(SHARED_BYTES) : null;
    const sharedHead    = sharedBuf ? new Int32Array(sharedBuf, RING_CAPACITY * SLOT_BYTES,     1) : null;
    const sharedTail    = sharedBuf ? new Int32Array(sharedBuf, RING_CAPACITY * SLOT_BYTES + 4, 1) : null;
    const sharedSlots   = sharedBuf ? new Uint8Array(sharedBuf, 0, RING_CAPACITY * SLOT_BYTES)  : null;

    modeEl.textContent = useWorker
        ? 'Memory mode: SharedArrayBuffer worker ring (Stage 1B)'
        : 'Memory mode: main-thread WASM (Stage 1A fallback)';

    if (useWorker && sharedBuf) {
        const worker = new Worker(new URL('./worker.ts', import.meta.url), { type: 'module' });
        await new Promise<void>((resolve, reject) => {
            worker.onmessage = (e: MessageEvent<{ type: string }>) => {
                if (e.data.type === 'ready') resolve();
                else reject(new Error(JSON.stringify(e.data)));
            };
            worker.postMessage({
                type: 'init',
                buffer: sharedBuf,
                ringCapacity: RING_CAPACITY,
                entityCount: ENTITY_COUNT,
                halfW: cx * 0.9,
                halfH: cy * 0.9,
            });
        });
    } else {
        for (let i = 0; i < ENTITY_COUNT; i++) {
            const tail = wasmRingTail[0];
            const off  = (tail & wasmMask) * SLOT_BYTES;
            const x = (Math.random() * 2 - 1) * cx * 0.9;
            const y = (Math.random() * 2 - 1) * cy * 0.9;
            wasmRingView.setUint8(off,        0);
            wasmRingView.setUint32(off + 1,   i, true);
            wasmRingView.setUint8(off + 5,    0);
            wasmRingView.setFloat32(off + 6,  x,  true);
            wasmRingView.setFloat32(off + 10, y,  true);
            wasmRingView.setFloat32(off + 14, 0,  true);
            wasmRingView.setFloat32(off + 18, 0,  true);
            wasmRingView.setFloat32(off + 22, 0,  true);
            wasmRingView.setFloat32(off + 26, 0,  true);
            wasmRingView.setFloat32(off + 30, 1,  true);
            wasmRingTail[0] = tail + 1;
        }
    }

    runtime.step();

    const ringHeadView = new Uint32Array(memory.buffer, ringHeadOffset, 1);

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

    function frame() {
        bridgeSharedRing();
        runtime.step();

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
        pendingEl.textContent = `Ring head: ${Atomics.load(ringHeadView, 0)}`;
        requestAnimationFrame(frame);
    }

    requestAnimationFrame(frame);
}

main().catch(console.error);
