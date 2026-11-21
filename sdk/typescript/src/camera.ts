import { NO_ENTITY } from './CommandRing.js';

/** A pinhole camera. The simulation takes intrinsics in pixels, because turning a field of view into them needs `tan`. */
export interface Camera {
    width: number;
    height: number;
    /** Vertical field of view in radians; with square pixels this fixes both focal lengths. */
    fovY: number;
    near?: number;
    far?: number;
    /** A body the rays pass through, usually the one the camera is mounted on. */
    ignore?: number;
}

/** Position and unit rotation (x, y, z, w). The camera looks down its local -z with +x to the right and +y up. */
export interface CameraPose {
    position: readonly [number, number, number];
    rotation: readonly [number, number, number, number];
}

/** One world's depth (metres along the view axis, 0 where nothing is hit) and body id (`NO_ENTITY` for none), row 0 at the top. */
export interface WorldImage {
    depth: Float32Array;
    id: Uint32Array;
}

export const DEFAULT_NEAR = 0.05;
export const DEFAULT_FAR = 100;

/** The six lens words `[fx, fy, cx, cy, near, far]` the wasm takes, validated here so a mistake names the field. */
export function lensWords(c: Camera): Float32Array {
    if (!Number.isInteger(c.width) || !Number.isInteger(c.height) || c.width < 1 || c.height < 1) throw new RangeError(`image size must be positive integers, got ${c.width}x${c.height}`);
    if (!Number.isFinite(c.fovY) || c.fovY <= 0 || c.fovY >= Math.PI) throw new RangeError(`fovY must be in (0, pi) radians, got ${c.fovY}`);
    const near = c.near ?? DEFAULT_NEAR;
    const far = c.far ?? DEFAULT_FAR;
    if (!Number.isFinite(near) || !Number.isFinite(far) || near < 0 || far <= near) throw new RangeError(`need 0 <= near < far, got ${near} and ${far}`);
    const f = c.height / 2 / Math.tan(c.fovY / 2);
    if (c.ignore !== undefined && !(Number.isInteger(c.ignore) && c.ignore >= 0 && c.ignore <= NO_ENTITY)) throw new RangeError(`ignore must be a body id, got ${c.ignore}`);
    return new Float32Array([f, f, c.width / 2, c.height / 2, near, far]);
}

export function poseWords(poses: readonly CameraPose[]): Float32Array {
    const out = new Float32Array(poses.length * 7);
    poses.forEach((p, i) => {
        const words = [...p.position, ...p.rotation];
        if (words.length !== 7 || !words.every(Number.isFinite)) throw new RangeError(`pose ${i} needs a finite position and rotation`);
        out.set(words, i * 7);
    });
    return out;
}

/** The pose of a camera at `eye` looking at `target`, with `up` (default +y) pointing up the image. */
export function lookAt(eye: readonly [number, number, number], target: readonly [number, number, number], up: readonly [number, number, number] = [0, 1, 0]): CameraPose {
    const norm = (v: number[]): number[] => {
        const l = Math.hypot(v[0], v[1], v[2]);
        if (!(l > 1e-9)) throw new RangeError('lookAt needs eye and target apart and a non-zero up');
        return v.map((c) => c / l);
    };
    const cross = (a: number[], b: number[]) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
    const f = norm([target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]]);
    const side = cross(f, [...up]);
    if (Math.hypot(side[0], side[1], side[2]) < 1e-6) throw new RangeError('lookAt: the view direction is parallel to up');
    const x = norm(side);
    const y = cross(x, f);
    const z = [-f[0], -f[1], -f[2]];
    // Rotation matrix with columns x, y, z to a quaternion.
    const [m00, m10, m20, m01, m11, m21, m02, m12, m22] = [x[0], x[1], x[2], y[0], y[1], y[2], z[0], z[1], z[2]];
    const trace = m00 + m11 + m22;
    let q: [number, number, number, number];
    if (trace > 0) {
        const s = Math.sqrt(trace + 1) * 2;
        q = [(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, s / 4];
    } else if (m00 > m11 && m00 > m22) {
        const s = Math.sqrt(1 + m00 - m11 - m22) * 2;
        q = [s / 4, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s];
    } else if (m11 > m22) {
        const s = Math.sqrt(1 + m11 - m00 - m22) * 2;
        q = [(m01 + m10) / s, s / 4, (m12 + m21) / s, (m02 - m20) / s];
    } else {
        const s = Math.sqrt(1 + m22 - m00 - m11) * 2;
        q = [(m02 + m20) / s, (m12 + m21) / s, s / 4, (m10 - m01) / s];
    }
    return { position: [eye[0], eye[1], eye[2]], rotation: q };
}

/** RGBA bytes for a body-id image: a fixed colour per id, black for no body. For looking at; learning code should use the ids. */
export function idColors(ids: Uint32Array): Uint8ClampedArray {
    const out = new Uint8ClampedArray(ids.length * 4);
    for (let i = 0; i < ids.length; i++) {
        const id = ids[i];
        if (id === NO_ENTITY) continue;
        // Integer hash so neighbouring ids differ visibly; 64 + keeps every body off black.
        const h = Math.imul(id + 1, 0x9e3779b1) >>> 0;
        out.set([64 + (h & 0xbf), 64 + ((h >>> 8) & 0xbf), 64 + ((h >>> 16) & 0xbf), 255], i * 4);
    }
    return out;
}
