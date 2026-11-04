import init, { Runtime } from '../wasm/dynova_core.js';
import { layoutFromRuntime } from './World.js';
import type { SimInit, SimMessage } from './launch.js';

const post = (m: SimMessage) => (self as unknown as Worker).postMessage(m);

let started = false;

self.onmessage = async (e: MessageEvent<SimInit>) => {
    if (e.data.type !== 'init' || started) return;
    started = true;
    const { memory, capacity, ringCapacity, physics, hz, autoRun } = e.data;
    try {
        await init({
            module_or_path: new URL('../wasm/dynova_core_bg.wasm', import.meta.url),
            memory,
        });
        const runtime = new Runtime(capacity, ringCapacity);
        if (physics) runtime.enable_physics();
        post({ type: 'ready', layout: layoutFromRuntime(runtime) });

        if (!autoRun) return;
        const stepMs = 1000 / hz;
        let next = performance.now();
        const loop = () => {
            try {
                const now = performance.now();
                let steps = 0;
                while (next <= now && steps < 5) {
                    runtime.step();
                    steps++;
                    next += stepMs;
                }
                if (steps === 5) next = now + stepMs;
                setTimeout(loop, Math.max(0, next - performance.now()));
            } catch (err) {
                post({ type: 'error', message: err instanceof Error ? err.message : String(err) });
            }
        };
        loop();
    } catch (err) {
        post({ type: 'error', message: err instanceof Error ? err.message : String(err) });
    }
};
