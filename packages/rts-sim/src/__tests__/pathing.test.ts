import {getAllEntities, hasComponent} from "bitecs";
import {describe, expect, it} from "vitest";
import {PathState, UnitKindId} from "../sim/components";
import {SpatialHash} from "../sim/grid/spatial_hash";
import {hashWorld} from "../sim/snapshot";
import {run, step} from "../sim/tick";
import {createSimWorld, idOf, spawnUnit, type SimConfig, type SimWorld} from "../sim/world";

/** A 32x32 map, 8-cell sectors, walled at x = 16 and open only past y = 28. */
const WALLED: Partial<SimConfig> = {
    seed: 7,
    capacity: 256,
    mapWidth: 32,
    mapHeight: 32,
    sectorSize: 8,
    obstacles: [{x: 16, y: 0, w: 1, h: 28, weight: 0}],
};

function live(world: SimWorld): number[] {
    return getAllEntities(world)
        .slice()
        .sort((a, b) => idOf(world, a) - idOf(world, b));
}

describe("SpatialHash", () => {
    const xs = new Float64Array([1, 1.5, 9, 1.2]);
    const ys = new Float64Array([1, 1.0, 9, 5.0]);
    const entities = [0, 1, 2, 3];
    const rawIds = new Int32Array([0, 1, 2, 3]);

    function build(): SpatialHash {
        const grid = new SpatialHash(16, 16, 2, 16);
        grid.build(entities, rawIds, 4, xs, ys);
        return grid;
    }

    it("finds only what is inside the radius", () => {
        const out = new Int32Array(8);
        const found = build().collect(1, 1, 1, out);

        expect(Array.from(out.subarray(0, found)).sort()).toEqual([0, 1]);
    });

    it("reaches across cell boundaries", () => {
        const out = new Int32Array(8);
        const found = build().collect(1.2, 3, 2.5, out);

        expect(Array.from(out.subarray(0, found)).sort()).toEqual([0, 1, 3]);
    });

    it("returns the same order every time it is rebuilt", () => {
        const out1 = new Int32Array(8);
        const out2 = new Int32Array(8);
        const n1 = build().collect(1, 1, 4, out1);
        const n2 = build().collect(1, 1, 4, out2);

        expect(n2).toBe(n1);
        expect(Array.from(out2)).toEqual(Array.from(out1));
    });

    it("stops at the output buffer's capacity", () => {
        const out = new Int32Array(1);
        expect(build().collect(1, 1, 8, out)).toBe(1);
    });

    it("clamps positions outside the grid rather than writing out of bounds", () => {
        const grid = new SpatialHash(16, 16, 2, 16);
        const far = new Float64Array([-50, 400]);
        const farY = new Float64Array([-50, 400]);
        grid.build([0, 1], new Int32Array([0, 1]), 2, far, farY);

        const out = new Int32Array(4);
        expect(grid.collect(-50, -50, 1, out)).toBe(1);
        expect(out[0]).toBe(0);
    });
});

describe("segment budget", () => {
    it("serves a whole group in one sector from a single segment", () => {
        const world = createSimWorld({...WALLED, segmentBudget: 1});

        for (let i = 0; i < 12; i++) {
            const eid = spawnUnit(
                world,
                UnitKindId.Militia,
                0,
                2 + (i % 4) * 0.5,
                2 + (i / 4) * 0.5,
            );
            world.cmd.setMoveTarget(eid, 28, 4);
        }
        step(world);

        // One integration answers all twelve: no threshold, no per-unit search.
        expect(world.paths.lastBuilt).toBe(1);
        expect(world.paths.lastServiced).toBe(12);
        expect(world.paths.pending).toBe(0);

        for (const eid of live(world)) {
            expect(world.stores.Path.state[idOf(world, eid)]).toBe(PathState.Flow);
        }
    });

    it("spends the budget when units are spread across sectors", () => {
        const world = createSimWorld({...WALLED, segmentBudget: 1});

        // One unit per sector along the top, so each needs its own segment.
        for (let i = 0; i < 2; i++) {
            const eid = spawnUnit(world, UnitKindId.Militia, 0, 2 + i * 8, 2);
            world.cmd.setMoveTarget(eid, 28, 4);
        }
        step(world);

        expect(world.paths.lastBuilt).toBe(1);
        expect(world.paths.pending).toBe(1);

        step(world);
        expect(world.paths.lastBuilt).toBe(1);
        expect(world.paths.pending).toBe(0);
    });

    it("reuses a cached segment for free, ignoring the budget", () => {
        const world = createSimWorld({...WALLED, segmentBudget: 1});

        const first = spawnUnit(world, UnitKindId.Militia, 0, 2, 2);
        world.cmd.setMoveTarget(first, 28, 4);
        step(world);
        expect(world.paths.lastBuilt).toBe(1);

        // A latecomer in the same sector to the same goal, with the budget
        // already accounted for.
        const late = spawnUnit(world, UnitKindId.Archer, 0, 3, 3);
        world.cmd.setMoveTarget(late, 28, 4);
        step(world);

        expect(world.paths.lastBuilt).toBe(0);
        expect(world.stores.Path.state[idOf(world, late)]).toBe(PathState.Flow);
    });

    it("rebuilds after terrain changes, one tick later", () => {
        const world = createSimWorld(WALLED);
        const eid = spawnUnit(world, UnitKindId.Militia, 0, 2, 2);
        world.cmd.setMoveTarget(eid, 28, 4);
        step(world);
        expect(world.paths.lastBuilt).toBe(1);
        expect(world.paths.segments.size).toBe(1);

        // A building goes up: every cached segment now describes a map that no
        // longer exists.
        world.map.setWeight(6, 6, 0);
        expect(world.paths.segments.size).toBe(1);

        // One tick of latency by design: movement is what notices the segment
        // has gone, and it runs after the pathfinder in the schedule.
        step(world);
        expect(world.stores.Path.state[idOf(world, eid)]).toBe(PathState.None);

        step(world);
        expect(world.paths.lastBuilt).toBe(1);
        expect(world.stores.Path.state[idOf(world, eid)]).toBe(PathState.Flow);
    });

    it("drops a pending request when its unit dies", () => {
        const world = createSimWorld({...WALLED, segmentBudget: 0});
        const eid = spawnUnit(world, UnitKindId.Militia, 0, 2, 2);
        world.cmd.setMoveTarget(eid, 28, 4);
        step(world);
        expect(world.paths.pending).toBe(1);

        world.cmd.despawn(eid);
        step(world);

        expect(world.paths.pending).toBe(0);
        expect(world.paths.lastBuilt).toBe(0);
    });

    it("hashes the pending queue, so a differently-ordered backlog diverges", () => {
        const config = {...WALLED, segmentBudget: 0};
        const a = createSimWorld(config);
        const b = createSimWorld(config);

        const a1 = spawnUnit(a, UnitKindId.Militia, 0, 2, 2);
        const a2 = spawnUnit(a, UnitKindId.Militia, 0, 2, 4);
        const b1 = spawnUnit(b, UnitKindId.Militia, 0, 2, 2);
        const b2 = spawnUnit(b, UnitKindId.Militia, 0, 2, 4);
        expect(hashWorld(a)).toBe(hashWorld(b));

        // Same units, same goals, opposite request order.
        a.paths.request(a1, 100);
        a.paths.request(a2, 101);
        b.paths.request(b2, 101);
        b.paths.request(b1, 100);

        expect(hashWorld(a)).not.toBe(hashWorld(b));
    });
});

describe("flow following", () => {
    it("gets a unit around a wall to a goal it cannot see", () => {
        const world = createSimWorld(WALLED);
        const eid = spawnUnit(world, UnitKindId.Militia, 0, 2, 2);
        world.cmd.setMoveTarget(eid, 28, 4);

        run(world, 900);

        const id = idOf(world, eid);
        expect(world.stores.Position.x[id]).toBeCloseTo(28, 3);
        expect(world.stores.Position.y[id]).toBeCloseTo(4, 3);
        // Arriving clears the order, which is how a caller knows it is done.
        expect(hasComponent(world, eid, world.stores.MoveTarget)).toBe(false);
    });

    it("never steps onto an impassable tile on the way", () => {
        const world = createSimWorld(WALLED);
        const eid = spawnUnit(world, UnitKindId.Militia, 0, 2, 2);
        world.cmd.setMoveTarget(eid, 28, 4);

        const id = idOf(world, eid);
        for (let i = 0; i < 900; i++) {
            step(world);
            const tile = world.map.worldToIndex(
                world.stores.Position.x[id],
                world.stores.Position.y[id],
            );
            expect(world.map.isPassableIndex(tile), `tick ${i}`).toBe(true);
        }
    });

    it("picks up new segments as it crosses sectors, with no route length to run out", () => {
        const world = createSimWorld({
            seed: 3,
            capacity: 64,
            mapWidth: 128,
            mapHeight: 16,
            sectorSize: 8,
            obstacles: [],
        });
        const eid = spawnUnit(world, UnitKindId.Militia, 0, 2, 8);
        world.cmd.setMoveTarget(eid, 120, 8);

        run(world, 900);

        const id = idOf(world, eid);
        expect(world.stores.Position.x[id]).toBeCloseTo(120, 3);
        // A long crossing needs a segment per sector, all cached for reuse.
        expect(world.paths.segments.size).toBeGreaterThan(3);
    });

    it("gives up once, rather than re-asking forever, when ordered off the map", () => {
        const world = createSimWorld(WALLED);
        const eid = spawnUnit(world, UnitKindId.Militia, 0, 2, 2);
        world.cmd.setMoveTarget(eid, 500, 500);

        run(world, 10);

        const id = idOf(world, eid);
        expect(world.stores.Path.state[id]).toBe(PathState.Failed);
        expect(world.paths.pending).toBe(0);
    });

    it("fails a unit whose goal is sealed off", () => {
        const world = createSimWorld({
            ...WALLED,
            obstacles: [
                {x: 20, y: 20, w: 5, h: 1, weight: 0},
                {x: 20, y: 24, w: 5, h: 1, weight: 0},
                {x: 20, y: 20, w: 1, h: 5, weight: 0},
                {x: 24, y: 20, w: 1, h: 5, weight: 0},
            ],
        });
        const eid = spawnUnit(world, UnitKindId.Militia, 0, 2, 2);
        world.cmd.setMoveTarget(eid, 22, 22);

        run(world, 20);

        expect(world.stores.Path.state[idOf(world, eid)]).toBe(PathState.Failed);
    });
});

describe("an army on one destination", () => {
    it("crosses a wall together", () => {
        const world = createSimWorld({
            seed: 21,
            capacity: 256,
            mapWidth: 32,
            mapHeight: 32,
            sectorSize: 8,
            obstacles: [{x: 16, y: 0, w: 1, h: 26, weight: 0}],
        });

        for (let i = 0; i < 40; i++) {
            const eid = spawnUnit(
                world,
                UnitKindId.Militia,
                0,
                2 + (i % 8) * 0.6,
                2 + (i / 8) * 0.6,
            );
            world.cmd.setMoveTarget(eid, 28, 4);
        }

        step(world);
        expect(world.paths.lastServiced).toBe(40);

        run(world, 1200);

        // 40 units cannot all stand on one point — there are no formations yet —
        // so the test is that the army got there, not that it converged.
        const {Position} = world.stores;
        for (const eid of live(world)) {
            const id = idOf(world, eid);
            const dx = Position.x[id] - 28;
            const dy = Position.y[id] - 4;
            expect(Math.sqrt(dx * dx + dy * dy), `unit ${eid}`).toBeLessThan(6);
        }
    });
});

describe("separation", () => {
    it("pushes a pile of units apart", () => {
        const world = createSimWorld({...WALLED, seed: 11});
        for (let i = 0; i < 8; i++) {
            spawnUnit(world, UnitKindId.Militia, 0, 4, 4);
        }

        run(world, 120);

        const entities = live(world);
        const {Position, Radius} = world.stores;
        for (let i = 0; i < entities.length; i++) {
            for (let j = i + 1; j < entities.length; j++) {
                const a = idOf(world, entities[i]);
                const b = idOf(world, entities[j]);
                const dx = Position.x[a] - Position.x[b];
                const dy = Position.y[a] - Position.y[b];
                const distance = Math.sqrt(dx * dx + dy * dy);
                const minimum = Radius.value[a] + Radius.value[b];
                expect(distance, `${i} vs ${j}`).toBeGreaterThan(minimum * 0.9);
            }
        }
    });

    it("separates coincident units without producing NaN", () => {
        const world = createSimWorld({...WALLED, seed: 5});
        const a = spawnUnit(world, UnitKindId.Militia, 0, 4, 4);
        const b = spawnUnit(world, UnitKindId.Militia, 0, 4, 4);

        // Force exact coincidence, past the spawn jitter.
        const idA = idOf(world, a);
        const idB = idOf(world, b);
        world.stores.Position.x[idB] = world.stores.Position.x[idA];
        world.stores.Position.y[idB] = world.stores.Position.y[idA];

        run(world, 30);

        for (const id of [idA, idB]) {
            expect(Number.isFinite(world.stores.Position.x[id])).toBe(true);
            expect(Number.isFinite(world.stores.Position.y[id])).toBe(true);
        }
        expect(world.stores.Position.x[idA]).not.toBe(world.stores.Position.x[idB]);
    });
});
