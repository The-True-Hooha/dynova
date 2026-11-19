// The one Node API the real-wasm tests use, declared here so the SDK needs no @types/node.
declare module 'node:fs' {
    export function readFileSync(path: URL | string): Uint8Array;
}
