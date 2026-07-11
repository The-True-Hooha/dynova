import { describe, it, expect } from 'vitest';
import { World } from '../World.js';

const config = { capacity: 128, ringCapacity: 64 };

describe('World', () => {
    it('creates with static factory', () => {
        const world = World.create(config);
        expect(world.allocatedEntities).toBe(0);
        expect(world.pendingCommands).toBe(0);
    });

    it('spawn returns handle with id=0 gen=0 first', () => {
        const world = World.create(config);
        expect(world.spawn()).toEqual({ id: 0, gen: 0 });
    });

    it('spawn returns incrementing ids', () => {
        const world = World.create(config);
        expect(world.spawn().id).toBe(0);
        expect(world.spawn().id).toBe(1);
        expect(world.spawn().id).toBe(2);
    });

    it('spawn enqueues a command', () => {
        const world = World.create(config);
        world.spawn({ x: 1, y: 2, z: 3 });
        expect(world.pendingCommands).toBe(1);
    });

    it('destroy re-enqueues id with incremented gen', () => {
        const world = World.create(config);
        const h = world.spawn();
        world.destroy(h);
        const h2 = world.spawn();
        expect(h2.id).toBe(h.id);
        expect(h2.gen).toBe(1);
    });

    it('force enqueues a command', () => {
        const world = World.create(config);
        const h = world.spawn();
        world.force(h, 0, -9.8, 0);
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

    it('backpressureDrops increments on ring-full rejection', () => {
        const world = World.create({ capacity: 128, ringCapacity: 2 });
        world.spawn(); world.spawn();
        expect(world.backpressureDrops).toBe(0);
        expect(() => world.spawn()).toThrow();
        expect(world.backpressureDrops).toBe(1);
        expect(() => world.spawn()).toThrow();
        expect(world.backpressureDrops).toBe(2);
    });

    it('grow returns previous page count and views remain valid', () => {
        const world = World.create({ capacity: 128, ringCapacity: 64, initialPages: 2, maximumPages: 16 });
        const h = world.spawn({ x: 1, y: 2, z: 3 });
        const prev = world.grow(1);
        expect(prev).toBeGreaterThanOrEqual(0);
        expect(() => world.spawn()).not.toThrow();
    });

    it('grow returns -1 when max pages exceeded', () => {
        const world = World.create({ capacity: 128, ringCapacity: 64, initialPages: 2, maximumPages: 2 });
        const result = world.grow(1);
        expect(result).toBe(-1);
    });
});
