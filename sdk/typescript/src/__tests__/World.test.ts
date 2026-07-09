import { describe, it, expect } from 'vitest';
import { World } from '../World.js';

const config = { capacity: 128, ringCapacity: 64 };

describe('World', () => {
    it('creates with static factory', () => {
        const world = World.create(config);
        expect(world.allocatedEntities).toBe(0);
        expect(world.pendingCommands).toBe(0);
    });

    it('spawn returns incrementing ids', () => {
        const world = World.create(config);
        expect(world.spawn()).toBe(0);
        expect(world.spawn()).toBe(1);
        expect(world.spawn()).toBe(2);
    });

    it('spawn enqueues a command', () => {
        const world = World.create(config);
        world.spawn({ x: 1, y: 2, z: 3 });
        expect(world.pendingCommands).toBe(1);
    });

    it('destroy re-enqueues id for reuse', () => {
        const world = World.create(config);
        const id = world.spawn();
        world.destroy(id);
        expect(world.spawn()).toBe(id);
    });

    it('force enqueues a command', () => {
        const world = World.create(config);
        const id = world.spawn();
        world.force(id, 0, -9.8, 0);
        expect(world.pendingCommands).toBe(2);
    });

    it('spawn 10k entities without throwing', () => {
        const world = World.create({ capacity: 10_000, ringCapacity: 16384 });
        for (let i = 0; i < 10_000; i++) world.spawn();
        expect(world.allocatedEntities).toBe(10_000);
        expect(world.pendingCommands).toBe(10_000);
    });

    it('throws when ring is full', () => {
        const world = World.create({ capacity: 128, ringCapacity: 4 });
        world.spawn(); world.spawn(); world.spawn(); world.spawn();
        expect(() => world.spawn()).toThrow('CommandRing full');
    });
});
