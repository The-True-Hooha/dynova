export type MemoryMode = 'shared' | 'fallback' | 'headers-missing';

export interface MemoryResult {
    memory: WebAssembly.Memory;
    mode: MemoryMode;
}

export function detectMode(): MemoryMode {
    if (typeof SharedArrayBuffer === 'undefined') return 'fallback';
    try {
        new SharedArrayBuffer(4);
        return 'shared';
    } catch {
        return 'headers-missing';
    }
}

export function createMemory(initialPages: number, maximumPages: number): MemoryResult {
    const mode = detectMode();
    if (mode === 'shared') {
        return {
            memory: new WebAssembly.Memory({ initial: initialPages, maximum: maximumPages, shared: true }),
            mode,
        };
    }
    return {
        memory: new WebAssembly.Memory({ initial: initialPages, maximum: maximumPages }),
        mode,
    };
}
