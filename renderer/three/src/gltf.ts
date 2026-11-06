import type { Group } from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';

export type GltfLoadResult = { ok: true; scene: Group } | { ok: false; error: string };

/**
 * Loads a glTF scene from a URL, or from the bytes of a `.glb` file. A `.gltf` that references separate files only
 * works from a URL. Draco and KTX2 compressed files are not supported (no decoders are configured). Failures are
 * returned, not thrown: a bad or missing file is an expected outcome.
 */
export async function loadGltfScene(input: string | ArrayBuffer): Promise<GltfLoadResult> {
    if (typeof input !== 'string' && input.byteLength === 0) return { ok: false, error: 'empty file' };
    try {
        const loader = new GLTFLoader();
        const gltf = typeof input === 'string' ? await loader.loadAsync(input) : await loader.parseAsync(input, '');
        return { ok: true, scene: gltf.scene };
    } catch (err) {
        if (err instanceof SyntaxError) return { ok: false, error: `not a valid glTF or glb file (${err.message})` };
        if (err instanceof Error) return { ok: false, error: err.message };
        return { ok: false, error: typeof input === 'string' ? `could not load ${input}` : String(err) };
    }
}
