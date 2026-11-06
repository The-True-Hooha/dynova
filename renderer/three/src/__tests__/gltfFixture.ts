export interface Geometry {
    positions: number[];
    indices: number[];
}

export interface FixtureMesh extends Partial<Geometry> {
    name?: string;
    translation?: number[];
    rotation?: number[];
    scale?: number[];
    indexType?: 5123 | 5125;
}

export interface FixtureTransform {
    translation?: number[];
    rotation?: number[];
    scale?: number[];
}

export function cube(h = 0.5): Geometry {
    const positions: number[] = [];
    for (const x of [-h, h]) for (const y of [-h, h]) for (const z of [-h, h]) positions.push(x, y, z);
    const indices = [
        0, 1, 3, 0, 3, 2, 4, 6, 7, 4, 7, 5,
        0, 4, 5, 0, 5, 1, 2, 3, 7, 2, 7, 6,
        0, 2, 6, 0, 6, 4, 1, 5, 7, 1, 7, 3,
    ];
    return { positions, indices };
}

export function box(hx: number, hy: number, hz: number): Geometry {
    const g = cube(1);
    for (let i = 0; i < g.positions.length; i += 3) {
        g.positions[i] *= hx;
        g.positions[i + 1] *= hy;
        g.positions[i + 2] *= hz;
    }
    return g;
}

export function merge(...parts: Geometry[]): Geometry {
    const positions: number[] = [];
    const indices: number[] = [];
    for (const g of parts) {
        const base = positions.length / 3;
        positions.push(...g.positions);
        indices.push(...g.indices.map((i) => i + base));
    }
    return { positions, indices };
}

export function translated(g: Geometry, dx: number, dy: number, dz: number): Geometry {
    const positions = g.positions.map((v, i) => v + [dx, dy, dz][i % 3]);
    return { positions, indices: [...g.indices] };
}

export function inverted(g: Geometry): Geometry {
    const indices: number[] = [];
    for (let i = 0; i < g.indices.length; i += 3) indices.push(g.indices[i], g.indices[i + 2], g.indices[i + 1]);
    return { positions: [...g.positions], indices };
}

export function uvSphere(r = 1, segments = 12, rings = 8): Geometry {
    const positions: number[] = [0, r, 0, 0, -r, 0];
    for (let i = 1; i < rings; i++) {
        const phi = (i / rings) * Math.PI;
        for (let j = 0; j < segments; j++) {
            const theta = (j / segments) * Math.PI * 2;
            positions.push(r * Math.sin(phi) * Math.cos(theta), r * Math.cos(phi), r * Math.sin(phi) * Math.sin(theta));
        }
    }
    const indices: number[] = [];
    const ring = (i: number, j: number) => 2 + (i - 1) * segments + (j % segments);
    for (let j = 0; j < segments; j++) {
        indices.push(0, ring(1, j + 1), ring(1, j));
        indices.push(1, ring(rings - 1, j), ring(rings - 1, j + 1));
    }
    for (let i = 1; i < rings - 1; i++) {
        for (let j = 0; j < segments; j++) {
            indices.push(ring(i, j), ring(i, j + 1), ring(i + 1, j));
            indices.push(ring(i, j + 1), ring(i + 1, j + 1), ring(i + 1, j));
        }
    }
    return { positions, indices };
}

export function noisySphere(seed: number, noise: number, segments = 50, rings = 40): Geometry {
    let s = seed >>> 0;
    const rand = () => ((s = (Math.imul(s, 1664525) + 1013904223) >>> 0) / 2 ** 32);
    const g = uvSphere(1, segments, rings);
    for (let i = 6; i < g.positions.length; i += 3) {
        const k = 1 + (rand() * 2 - 1) * noise;
        g.positions[i] *= k;
        g.positions[i + 1] *= k;
        g.positions[i + 2] *= k;
    }
    return g;
}

export function prism(sides: number, radius: number, height: number): Geometry {
    const positions: number[] = [0, height / 2, 0, 0, -height / 2, 0];
    for (let k = 0; k < sides; k++) {
        const t = (k / sides) * Math.PI * 2;
        positions.push(radius * Math.cos(t), height / 2, radius * Math.sin(t), radius * Math.cos(t), -height / 2, radius * Math.sin(t));
    }
    const indices: number[] = [];
    for (let k = 0; k < sides; k++) {
        const a = 2 + k * 2, b = 2 + ((k + 1) % sides) * 2;
        indices.push(0, b, a, 1, a + 1, b + 1, a, b, a + 1, b, b + 1, a + 1);
    }
    return { positions, indices };
}

export function pyramid(sides: number, radius: number, height: number): Geometry {
    const positions: number[] = [0, 0, 0, 0, height, 0];
    for (let k = 0; k < sides; k++) {
        const t = (k / sides) * Math.PI * 2;
        positions.push(radius * Math.cos(t), 0, radius * Math.sin(t));
    }
    const indices: number[] = [];
    for (let k = 0; k < sides; k++) {
        const a = 2 + k, b = 2 + ((k + 1) % sides);
        indices.push(0, b, a, 1, a, b);
    }
    return { positions, indices };
}

export function tetra(): Geometry {
    return {
        positions: [1, 1, 1, 1, -1, -1, -1, 1, -1, -1, -1, 1],
        indices: [0, 1, 2, 0, 3, 1, 0, 2, 3, 1, 3, 2],
    };
}

export function plane(hx = 5, hz = 5): Geometry {
    return { positions: [-hx, 0, -hz, hx, 0, -hz, hx, 0, hz, -hx, 0, hz], indices: [0, 2, 1, 0, 3, 2] };
}

export function lShape(): Geometry {
    return merge(translated(box(1, 0.5, 0.5), 1, 0.5, 0.5), translated(box(0.5, 0.5, 1), 0.5, 0.5, 2));
}

export function buildGlb(meshes: FixtureMesh[], root: FixtureTransform = {}): ArrayBuffer {
    const chunks: Uint8Array[] = [];
    let length = 0;
    const push = (b: Uint8Array) => {
        const padded = new Uint8Array(b.length + ((4 - (b.length % 4)) % 4));
        padded.set(b);
        const at = length;
        chunks.push(padded);
        length += padded.length;
        return { at, bytes: b.length };
    };
    const bufferViews: object[] = [];
    const accessors: object[] = [];
    const gltfMeshes: object[] = [];
    const meshNodes: object[] = [];

    meshes.forEach((m, i) => {
        const positions = m.positions ?? [];
        const pos = push(new Uint8Array(new Float32Array(positions).buffer));
        bufferViews.push({ buffer: 0, byteOffset: pos.at, byteLength: pos.bytes });
        const min = [0, 1, 2].map((k) => Math.min(...positions.filter((_, j) => j % 3 === k), Infinity));
        const max = [0, 1, 2].map((k) => Math.max(...positions.filter((_, j) => j % 3 === k), -Infinity));
        accessors.push({ bufferView: bufferViews.length - 1, componentType: 5126, count: positions.length / 3, type: 'VEC3', min, max });
        const primitive: Record<string, unknown> = { attributes: { POSITION: accessors.length - 1 } };
        if (m.indices) {
            const type = m.indexType ?? 5125;
            const arr = type === 5123 ? new Uint16Array(m.indices) : new Uint32Array(m.indices);
            const idx = push(new Uint8Array(arr.buffer));
            bufferViews.push({ buffer: 0, byteOffset: idx.at, byteLength: idx.bytes });
            accessors.push({ bufferView: bufferViews.length - 1, componentType: type, count: m.indices.length, type: 'SCALAR' });
            primitive.indices = accessors.length - 1;
        }
        gltfMeshes.push({ primitives: [primitive] });
        const node: Record<string, unknown> = { name: m.name ?? `mesh${i}`, mesh: i };
        if (m.translation) node.translation = m.translation;
        if (m.rotation) node.rotation = m.rotation;
        if (m.scale) node.scale = m.scale;
        meshNodes.push(node);
    });

    const rootNode: Record<string, unknown> = { name: 'root', children: meshNodes.map((_, i) => i + 1) };
    if (root.translation) rootNode.translation = root.translation;
    if (root.rotation) rootNode.rotation = root.rotation;
    if (root.scale) rootNode.scale = root.scale;
    const data = new Uint8Array(length);
    let offset = 0;
    for (const c of chunks) { data.set(c, offset); offset += c.length; }
    const json = new TextEncoder().encode(JSON.stringify({
        asset: { version: '2.0' },
        scene: 0,
        scenes: [{ nodes: [0] }],
        nodes: [rootNode, ...meshNodes],
        meshes: gltfMeshes,
        accessors,
        bufferViews,
        buffers: [{ byteLength: data.length }],
    }));
    const pad = (n: number) => (4 - (n % 4)) % 4;
    const jsonLength = json.length + pad(json.length);
    const binLength = data.length + pad(data.length);
    const out = new Uint8Array(12 + 8 + jsonLength + 8 + binLength);
    const view = new DataView(out.buffer);
    view.setUint32(0, 0x46546c67, true);
    view.setUint32(4, 2, true);
    view.setUint32(8, out.length, true);
    view.setUint32(12, jsonLength, true);
    view.setUint32(16, 0x4e4f534a, true);
    out.fill(0x20, 20, 20 + jsonLength);
    out.set(json, 20);
    view.setUint32(20 + jsonLength, binLength, true);
    view.setUint32(24 + jsonLength, 0x004e4942, true);
    out.set(data, 28 + jsonLength);
    return out.buffer;
}
