import { Group, LoadingManager, Mesh, MeshStandardMaterial, Texture, type BufferGeometry } from 'three';
import { DRACOLoader } from 'three/examples/jsm/loaders/DRACOLoader.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import type { KTX2Loader } from 'three/examples/jsm/loaders/KTX2Loader.js';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import { PLYLoader } from 'three/examples/jsm/loaders/PLYLoader.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { MeshoptDecoder } from 'three/examples/jsm/libs/meshopt_decoder.module.js';

export type ModelFormat = 'glb' | 'gltf' | 'obj' | 'stl' | 'ply';

export type ModelLoadResult = { ok: true; scene: Group; format: ModelFormat } | { ok: false; error: string };

export interface ModelFile {
    name: string;
    data: ArrayBuffer;
}

export interface LoadOptions {
    /** URL of the directory holding the Draco decoder (`draco_decoder.js`, `.wasm`, `draco_wasm_wrapper.js`). Needed only for Draco-compressed glTF. */
    dracoDecoderPath?: string;
    /** A configured `KTX2Loader` to decode KTX2 textures. Without one they are skipped, since colliders do not need textures. */
    ktx2Loader?: KTX2Loader;
}

const PRIMARY: readonly ModelFormat[] = ['glb', 'gltf', 'obj', 'stl', 'ply'];
const EXTENSION = /\.([a-z0-9]+)$/i;

const extensionOf = (name: string): string => EXTENSION.exec(name.split(/[?#]/)[0])?.[1].toLowerCase() ?? '';
const baseName = (name: string): string => name.split(/[\\/]/).pop() ?? name;
const normalizePath = (name: string): string => name.replace(/\\/g, '/').replace(/^(\.\/)+/, '');
const safeDecode = (uri: string): string => {
    try {
        return decodeURIComponent(uri);
    } catch {
        return uri;
    }
};
const fail = (error: string): ModelLoadResult => ({ ok: false, error });

function describe(err: unknown, what: string): string {
    if (err instanceof SyntaxError) return `not a valid ${what} file (${err.message})`;
    if (err instanceof RangeError) return `the ${what} file is truncated or corrupt`;
    const message = err instanceof Error ? err.message : String(err);
    if (/DRACOLoader/.test(message)) return 'this file uses Draco compression: pass options.dracoDecoderPath';
    return message;
}

function gltfLoader(options: LoadOptions, manager?: LoadingManager): { loader: GLTFLoader; dispose(): void } {
    const loader = new GLTFLoader(manager);
    loader.setMeshoptDecoder(MeshoptDecoder);
    let draco: DRACOLoader | null = null;
    if (options.dracoDecoderPath) {
        draco = new DRACOLoader(manager).setDecoderPath(options.dracoDecoderPath.endsWith('/') ? options.dracoDecoderPath : `${options.dracoDecoderPath}/`);
        loader.setDRACOLoader(draco);
    }
    if (options.ktx2Loader) loader.setKTX2Loader(options.ktx2Loader);
    else {
        // The built-in plugin throws without a KTX2Loader; textures do not matter for colliders, so optional KTX2 becomes a blank texture.
        loader.register((parser) => ({
            name: 'KHR_texture_basisu',
            loadTexture(index: number) {
                if (!parser.json.textures?.[index]?.extensions?.KHR_texture_basisu) return null;
                if (parser.json.extensionsRequired?.includes('KHR_texture_basisu')) {
                    return Promise.reject(new Error('this file requires KTX2 textures: pass options.ktx2Loader'));
                }
                return Promise.resolve(new Texture());
            },
        }));
    }
    return { loader, dispose: () => draco?.dispose() };
}

interface ResourceIndex {
    resolve(uri: string): ModelFile | undefined;
    ambiguous(uri: string): boolean;
}

function indexResources(files: ModelFile[]): ResourceIndex {
    const byPath = new Map<string, ModelFile>();
    const byBase = new Map<string, ModelFile[]>();
    for (const f of files) {
        byPath.set(normalizePath(f.name), f);
        const list = byBase.get(baseName(f.name)) ?? [];
        list.push(f);
        byBase.set(baseName(f.name), list);
    }
    return {
        resolve(uri) {
            const path = normalizePath(safeDecode(uri));
            const exact = byPath.get(path);
            if (exact) return exact;
            const same = byBase.get(baseName(path));
            return same?.length === 1 ? same[0] : undefined;
        },
        ambiguous(uri) {
            return !byPath.has(normalizePath(safeDecode(uri))) && (byBase.get(baseName(safeDecode(uri)))?.length ?? 0) > 1;
        },
    };
}

function problemResources(json: { buffers?: { uri?: string }[]; images?: { uri?: string }[] }, index: ResourceIndex): string | null {
    const wanted = [...(json.buffers ?? []), ...(json.images ?? [])].map((r) => r.uri).filter((u): u is string => !!u && !/^(data:|[a-z]+:\/\/|\/)/i.test(u));
    const missing = wanted.filter((u) => !index.resolve(u) && !index.ambiguous(u));
    if (missing.length > 0) return `the .gltf references files that were not provided: ${missing.map((u) => baseName(safeDecode(u))).join(', ')}`;
    const ambiguous = wanted.filter((u) => index.ambiguous(u));
    if (ambiguous.length > 0) return `the .gltf references ${ambiguous.join(', ')} but several provided files share that name`;
    return null;
}

async function parseGltfFiles(primary: ModelFile, files: ModelFile[], options: LoadOptions): Promise<ModelLoadResult> {
    const format: ModelFormat = extensionOf(primary.name) === 'glb' ? 'glb' : 'gltf';
    const created: string[] = [];
    const blobFor = new Map<ModelFile, string>();
    const others = files.filter((f) => f !== primary);
    const index = indexResources(others);
    const manager = new LoadingManager();
    manager.setURLModifier((url) => {
        const file = index.resolve(url);
        if (!file) return url;
        let blob = blobFor.get(file);
        if (!blob) {
            blob = URL.createObjectURL(new Blob([file.data]));
            blobFor.set(file, blob);
            created.push(blob);
        }
        return blob;
    });
    const { loader, dispose } = gltfLoader(options, manager);
    try {
        if (format === 'glb') return { ok: true, scene: (await loader.parseAsync(primary.data, '')).scene, format };
        const text = new TextDecoder().decode(primary.data);
        const problem = problemResources(JSON.parse(text), index);
        if (problem) return fail(problem);
        return { ok: true, scene: (await loader.parseAsync(text, '')).scene, format };
    } catch (err) {
        return fail(describe(err, 'glTF or glb'));
    } finally {
        for (const url of created) URL.revokeObjectURL(url);
        dispose();
    }
}

const wrap = (geometry: BufferGeometry, name: string): Group => {
    const mesh = new Mesh(geometry, new MeshStandardMaterial({ color: 0xb0b0b0, flatShading: true }));
    mesh.name = name;
    const group = new Group();
    group.add(mesh);
    return group;
};

function parseSingleFile(file: ModelFile, format: 'obj' | 'stl' | 'ply'): ModelLoadResult {
    try {
        const name = file.name.replace(EXTENSION, '');
        if (format === 'obj') return { ok: true, scene: new OBJLoader().parse(new TextDecoder().decode(file.data)), format };
        if (format === 'stl') return { ok: true, scene: wrap(new STLLoader().parse(file.data), name), format };
        return { ok: true, scene: wrap(new PLYLoader().parse(file.data), name), format };
    } catch (err) {
        return fail(describe(err, format.toUpperCase()));
    }
}

/**
 * Loads a model from the files a user picked or dropped. The primary file is the first `.glb`, `.gltf`, `.obj`, `.stl`
 * or `.ply` (in that order); the others are resources for a `.gltf` (`.bin`, images). Returns an error result instead of
 * throwing. STL and PLY carry no units or up axis: they are used as given (Y up, 1 unit = 1 metre).
 */
export async function loadModelFiles(files: readonly ModelFile[], options: LoadOptions = {}): Promise<ModelLoadResult> {
    const all = [...files];
    const primary = PRIMARY.map((fmt) => all.find((f) => extensionOf(f.name) === fmt)).find((f) => f !== undefined);
    if (!primary) return fail(all.length === 0 ? 'no files' : `no supported model file among: ${all.map((f) => f.name).join(', ')} (expected .glb, .gltf, .obj, .stl or .ply)`);
    if (primary.data.byteLength === 0) return fail('empty file');
    const format = extensionOf(primary.name) as ModelFormat;
    return format === 'glb' || format === 'gltf' ? parseGltfFiles(primary, all, options) : parseSingleFile(primary, format);
}

/** Loads a model from a URL, choosing the loader from its extension. `.gltf` resources resolve relative to the URL. */
export async function loadModelUrl(url: string, options: LoadOptions = {}): Promise<ModelLoadResult> {
    const format = extensionOf(url);
    if (!PRIMARY.includes(format as ModelFormat)) return fail(`unsupported file type in ${url} (expected .glb, .gltf, .obj, .stl or .ply)`);
    if (format === 'obj' || format === 'stl' || format === 'ply') {
        try {
            const response = await fetch(url);
            if (!response.ok) return fail(`could not load ${url}: HTTP ${response.status}`);
            return loadModelFiles([{ name: url, data: await response.arrayBuffer() }], options);
        } catch (err) {
            return fail(`could not load ${url}: ${err instanceof Error ? err.message : String(err)}`);
        }
    }
    const { loader, dispose } = gltfLoader(options);
    try {
        return { ok: true, scene: (await loader.loadAsync(url)).scene, format: format as ModelFormat };
    } catch (err) {
        return fail(err instanceof Error ? describe(err, 'glTF or glb') : `could not load ${url}`);
    } finally {
        dispose();
    }
}

/** glTF only. `loadModelFiles` and `loadModelUrl` accept every supported format. */
export async function loadGltfScene(input: string | ArrayBuffer, options: LoadOptions = {}): Promise<ModelLoadResult> {
    if (typeof input === 'string') return loadModelUrl(input, options);
    return loadModelFiles([{ name: 'model.glb', data: input }], options);
}
