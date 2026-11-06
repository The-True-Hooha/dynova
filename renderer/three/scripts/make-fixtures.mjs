// Regenerates src/testing/fixtures/*. Needs (not repo dependencies):
//   npm i @gltf-transform/core @gltf-transform/extensions @gltf-transform/functions draco3d meshoptimizer
// Run from a directory with those installed: node make-fixtures.mjs <outDir>
import { Document, NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS, KHRTextureBasisu } from '@gltf-transform/extensions';
import { draco, meshopt } from '@gltf-transform/functions';
import draco3d from 'draco3d';
import { MeshoptEncoder, MeshoptDecoder } from 'meshoptimizer';
import { mkdirSync, writeFileSync } from 'node:fs';

const out = process.argv[2] ?? '.';
mkdirSync(out, { recursive: true });

function cube(hx, hy, hz) {
    const positions = [];
    for (const x of [-hx, hx]) for (const y of [-hy, hy]) for (const z of [-hz, hz]) positions.push(x, y, z);
    const indices = [0, 1, 3, 0, 3, 2, 4, 6, 7, 4, 7, 5, 0, 4, 5, 0, 5, 1, 2, 3, 7, 2, 7, 6, 0, 2, 6, 0, 6, 4, 1, 5, 7, 1, 7, 3];
    return { positions, indices };
}
function pyramid(sides, r, h) {
    const positions = [0, 0, 0, 0, h, 0];
    for (let k = 0; k < sides; k++) { const t = (k / sides) * Math.PI * 2; positions.push(r * Math.cos(t), 0, r * Math.sin(t)); }
    const indices = [];
    for (let k = 0; k < sides; k++) { const a = 2 + k, b = 2 + ((k + 1) % sides); indices.push(0, b, a, 1, a, b); }
    return { positions, indices };
}
function ramp(n, rise) {
    const positions = [];
    for (let i = 0; i <= n; i++) for (let j = 0; j <= n; j++) positions.push(i, rise * i, j);
    const indices = [];
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) { const a = i * (n + 1) + j, b = a + 1, c = a + n + 1, d = c + 1; indices.push(a, b, c, b, d, c); }
    return { positions, indices };
}

const level = [
    ['floor', cube(6, 0.25, 6), [0, -0.25, 0]],
    ['wall-n', cube(6.5, 1, 0.25), [0, 1, -6.25]],
    ['ramp', ramp(5, 0.5), [-5, 0, -2]],
    ['pillar', pyramid(7, 1.2, 3), [3, 0, 3]],
];

const PNG = Uint8Array.from(Buffer.from('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mP8z8BQDwAEhQGAhKmMIQAAAABJRU5ErkJggg==', 'base64'));

function build({ withTexture = false, png = false } = {}) {
    const doc = new Document();
    const buffer = doc.createBuffer();
    const scene = doc.createScene('level');
    for (const [name, g, at] of level) {
        const pos = doc.createAccessor().setType('VEC3').setArray(new Float32Array(g.positions)).setBuffer(buffer);
        const idx = doc.createAccessor().setType('SCALAR').setArray(new Uint32Array(g.indices)).setBuffer(buffer);
        const prim = doc.createPrimitive().setAttribute('POSITION', pos).setIndices(idx);
        if (png) {
            const tex = doc.createTexture('albedo').setImage(PNG).setMimeType('image/png');
            prim.setMaterial(doc.createMaterial(name).setBaseColorTexture(tex));
        }
        if (withTexture) {
            const tex = doc.createTexture('albedo').setImage(new Uint8Array([1, 2, 3, 4, 5, 6, 7, 8])).setMimeType('image/ktx2');
            prim.setMaterial(doc.createMaterial(name).setBaseColorTexture(tex));
        }
        scene.addChild(doc.createNode(name).setTranslation(at).setMesh(doc.createMesh(name).addPrimitive(prim)));
    }
    return doc;
}

const io = new NodeIO().registerExtensions(ALL_EXTENSIONS).registerDependencies({
    'draco3d.encoder': await draco3d.createEncoderModule(),
    'draco3d.decoder': await draco3d.createDecoderModule(),
    'meshopt.encoder': MeshoptEncoder,
    'meshopt.decoder': MeshoptDecoder,
});

await MeshoptEncoder.ready;
await MeshoptDecoder.ready;

let doc = build();
await doc.transform(draco());
writeFileSync(`${out}/level-draco.glb`, await io.writeBinary(doc));

doc = build();
await doc.transform(meshopt({ encoder: MeshoptEncoder, level: 'medium' }));
writeFileSync(`${out}/level-meshopt.glb`, await io.writeBinary(doc));

doc = build({ png: true });
writeFileSync(`${out}/level-png.glb`, await io.writeBinary(doc));

doc = build({ withTexture: true });
doc.createExtension(KHRTextureBasisu).setRequired(false);
writeFileSync(`${out}/level-ktx2.glb`, await io.writeBinary(doc));

doc = build({ withTexture: true });
doc.createExtension(KHRTextureBasisu).setRequired(true);
writeFileSync(`${out}/level-ktx2-required.glb`, await io.writeBinary(doc));

doc = build({ withTexture: false });
const split = await io.writeJSON(doc, { basename: 'level-split' });
writeFileSync(`${out}/level-split.gltf`, JSON.stringify(split.json));
for (const [uri, data] of Object.entries(split.resources)) writeFileSync(`${out}/${uri}`, data);
console.log('wrote', out);
