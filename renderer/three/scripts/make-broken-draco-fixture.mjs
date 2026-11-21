// Derives level-draco-broken.glb from level-draco.glb: one extra mesh points at a bufferView that does not exist,
// so the parse fails immediately while the Draco decoder libraries are still loading. No dependencies.
// Run from this directory: node scripts/make-broken-draco-fixture.mjs
import { readFileSync, writeFileSync } from 'node:fs';

const dir = new URL('../src/testing/fixtures/', import.meta.url);
const glb = readFileSync(new URL('level-draco.glb', dir));
const jsonLength = glb.readUInt32LE(12);
const json = JSON.parse(glb.subarray(20, 20 + jsonLength).toString('utf8'));
const rest = glb.subarray(20 + jsonLength);

json.accessors.push({ bufferView: 999, componentType: 5126, count: 3, type: 'VEC3' });
json.meshes.push({ name: 'broken', primitives: [{ attributes: { POSITION: json.accessors.length - 1 } }] });
json.nodes.push({ name: 'broken', mesh: json.meshes.length - 1 });
json.scenes[0].nodes.push(json.nodes.length - 1);

let body = Buffer.from(JSON.stringify(json), 'utf8');
body = Buffer.concat([body, Buffer.alloc((4 - (body.length % 4)) % 4, 0x20)]);
const header = Buffer.alloc(20);
header.write('glTF', 0, 'ascii');
header.writeUInt32LE(2, 4);
header.writeUInt32LE(20 + body.length + rest.length, 8);
header.writeUInt32LE(body.length, 12);
header.write('JSON', 16, 'ascii');
writeFileSync(new URL('level-draco-broken.glb', dir), Buffer.concat([header, body, rest]));
