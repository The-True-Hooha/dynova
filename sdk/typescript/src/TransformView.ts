export const FLOATS_PER_TRANSFORM = 7;

export class TransformView {
    private a: Float32Array;
    private b: Float32Array;
    private flag: Int32Array;

    constructor(memory: WebAssembly.Memory, aByteOffset: number, bByteOffset: number, flagByteOffset: number, capacity: number) {
        const len = capacity * FLOATS_PER_TRANSFORM;
        this.a = new Float32Array(memory.buffer, aByteOffset, len);
        this.b = new Float32Array(memory.buffer, bByteOffset, len);
        this.flag = new Int32Array(memory.buffer, flagByteOffset, 1);
    }

    read(): Float32Array {
        const writing = Atomics.load(this.flag, 0);
        return writing === 0 ? this.b : this.a;
    }

    readEntity(entityId: number): { x: number; y: number; z: number; qx: number; qy: number; qz: number; qw: number } {
        const buf = this.read();
        const o = entityId * FLOATS_PER_TRANSFORM;
        return {
            x:  buf[o],
            y:  buf[o + 1],
            z:  buf[o + 2],
            qx: buf[o + 3],
            qy: buf[o + 4],
            qz: buf[o + 5],
            qw: buf[o + 6],
        };
    }

    reacquire(memory: WebAssembly.Memory, aByteOffset: number, bByteOffset: number, flagByteOffset: number, capacity: number): void {
        const len = capacity * FLOATS_PER_TRANSFORM;
        this.a = new Float32Array(memory.buffer, aByteOffset, len);
        this.b = new Float32Array(memory.buffer, bByteOffset, len);
        this.flag = new Int32Array(memory.buffer, flagByteOffset, 1);
    }
}
