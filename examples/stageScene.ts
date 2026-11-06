import { BoxGeometry, BufferAttribute, BufferGeometry, Group, Mesh, MeshStandardMaterial, SphereGeometry } from 'three';

export const CRATES = 12;
export const BALLS = 8;
export const CONES = 6;
export const PROPS = CRATES + BALLS + CONES;

export function pyramidGeometry(sides: number, radius: number, height: number): BufferGeometry {
    const pos: number[] = [0, 0, 0, 0, height, 0];
    for (let k = 0; k < sides; k++) {
        const t = (k / sides) * Math.PI * 2;
        pos.push(radius * Math.cos(t), 0, radius * Math.sin(t));
    }
    const idx: number[] = [];
    for (let k = 0; k < sides; k++) {
        const a = 2 + k, b = 2 + ((k + 1) % sides);
        idx.push(0, b, a, 1, a, b);
    }
    const g = new BufferGeometry();
    g.setAttribute('position', new BufferAttribute(new Float32Array(pos), 3));
    g.setIndex(idx);
    g.computeVertexNormals();
    return g;
}

export function prismGeometry(sides: number, radius: number, height: number): BufferGeometry {
    const g = pyramidGeometry(sides, radius, height);
    const pos = g.getAttribute('position') as BufferAttribute;
    for (let i = 0; i < pos.count; i++) if (pos.getY(i) === height) pos.setXYZ(i, 0, height, 0);
    return g;
}

export const material = (color: number) => new MeshStandardMaterial({ color, flatShading: true });

export function buildLevel(): Group {
    const level = new Group();
    const floor = new Mesh(new BoxGeometry(80, 1, 80), material(0x445544));
    floor.name = 'floor';
    floor.position.y = -0.5;

    const rampGeo = new BufferGeometry();
    const rampPts: number[] = [];
    for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) rampPts.push(i, 0.5 * i, j);
    rampGeo.setAttribute('position', new BufferAttribute(new Float32Array(rampPts), 3));
    const tri: number[] = [];
    for (let i = 0; i < 5; i++) for (let j = 0; j < 5; j++) {
        const a = i * 6 + j, b = a + 1, c = a + 6, d = c + 1;
        tri.push(a, b, c, b, d, c);
    }
    rampGeo.setIndex(tri);
    rampGeo.computeVertexNormals();
    const ramp = new Mesh(rampGeo, material(0x886644));
    ramp.name = 'ramp';
    ramp.position.set(-9, 0, -3);

    const pillar = new Mesh(pyramidGeometry(7, 1.2, 3), material(0x555577));
    pillar.name = 'pillar';
    pillar.position.set(6, 0, 4);

    level.add(floor, ramp, pillar);
    return level;
}

export function buildProps(): Group {
    const props = new Group();
    for (let i = 0; i < CRATES; i++) {
        const size = 0.6 + (i % 3) * 0.2;
        const m = new Mesh(new BoxGeometry(1, 1, 1), material(0xffaa44));
        m.name = `crate${i}`;
        m.scale.setScalar(size);
        m.position.set(-3 + (i % 4) * 1.6, 3 + Math.floor(i / 4) * 1.4, -2 + (i % 3));
        m.rotation.set(0.2 * i, 0.5 * i, 0);
        props.add(m);
    }
    for (let i = 0; i < BALLS; i++) {
        const m = new Mesh(new SphereGeometry(0.5, 16, 12), material(0x44aaff));
        m.name = `ball${i}`;
        m.scale.setScalar(0.5 + (i % 3) * 0.2);
        m.position.set(-8 + (i % 4) * 0.6, 6 + Math.floor(i / 4) * 1.2, -2.5 + (i % 2) * 2);
        props.add(m);
    }
    for (let i = 0; i < CONES; i++) {
        const m = new Mesh(pyramidGeometry(8, 0.7, 1.6), material(0xdd5588));
        m.name = `cone${i}`;
        m.position.set(2 + (i % 3) * 1.8, 5 + Math.floor(i / 3) * 1.6, -3 + (i % 2) * 5);
        m.rotation.set(0.4 * i, 0, 0.3 * i);
        props.add(m);
    }
    return props;
}

