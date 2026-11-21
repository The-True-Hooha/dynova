// The Node API the real-wasm tests use, declared here so the SDK needs no @types/node.
declare module 'node:fs' {
    export function readFileSync(path: URL | string): Uint8Array;
    export function readFileSync(path: URL | string, encoding: 'utf8'): string;
    export function readdirSync(path: URL | string): string[];
}
declare module 'node:crypto' {
    export function createHash(algorithm: 'sha256'): { update(data: Uint8Array | string): { digest(encoding: 'hex'): string } };
}
