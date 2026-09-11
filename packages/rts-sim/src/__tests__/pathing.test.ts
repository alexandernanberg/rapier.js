import {getAllEntities, hasComponent} from "bitecs";
import {describe, expect, it} from "vitest";
import {PathState} from "../sim/components";
import {UnitKindId} from "../sim/components";
import {SpatialHash} from "../sim/grid/spatial_hash";
import {hashWorld} from "../sim/snapshot";
import {run, step} from "../sim/tick";
import {createSimWorld, idOf, spawnUnit, type SimConfig, type SimWorld} from "../sim/world";

/** A 16x16 map walled at x = 8, open only past y = 12. */
const WALLED: Partial<SimConfig> = {
    seed: 7,
    capacity: 256,
    mapWidth: 16,
    mapHeight: 16,
    obstacles: [{x: 8, y: 0, w: 1, h: 12, weight: 0}],
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

describe("pathfinding budget", () => {
    it("services exactly the budget per tick and no more", () => {
        const world = createSimWorld({...WALLED, pathBudget: 2});

        for (let i = 0; i < 7; i++) {
            const eid = spawnUnit(world, UnitKindId.Militia, 0, 2, 2 + i * 0.5);
            world.cmd.setMoveTarget(eid, 13, 2);
        }
        step(world);

        expect(world.paths.lastServiced).toBe(2);
        expect(world.paths.pending).toBe(5);

        step(world);
        expect(world.paths.lastServiced).toBe(2);
        expect(world.paths.pending).toBe(3);

        run(world, 2);
        expect(world.paths.pending).toBe(0);
    });

    it("keeps a retargeted unit in its original queue position", () => {
        const world = createSimWorld({...WALLED, pathBudget: 0});

        const first = spawnUnit(world, UnitKindId.Militia, 0, 2, 2);
        const second = spawnUnit(world, UnitKindId.Militia, 0, 2, 4);
        world.cmd.setMoveTarget(first, 13, 2);
        world.cmd.setMoveTarget(second, 13, 4);
        step(world);
        expect(world.paths.pending).toBe(2);

        // The first unit re-clicks; it must not jump ahead of nobody, nor fall
        // behind the unit that asked later.
        world.cmd.setMoveTarget(first, 13, 10);
        step(world);
        expect(world.paths.pending).toBe(2);

        const world2 = createSimWorld({...WALLED, pathBudget: 1});
        const a = spawnUnit(world2, UnitKindId.Militia, 0, 2, 2);
        const b = spawnUnit(world2, UnitKindId.Militia, 0, 2, 4);
        world2.cmd.setMoveTarget(a, 13, 2);
        world2.cmd.setMoveTarget(b, 13, 4);
        step(world2);

        // With a budget of one, the unit that asked first is the one served.
        expect(world2.stores.Path.state[idOf(world2, a)]).toBe(PathState.Active);
        expect(world2.stores.Path.state[idOf(world2, b)]).toBe(PathState.Pending);
    });

    it("drops a pending request when its unit dies", () => {
        const world = createSimWorld({...WALLED, pathBudget: 0});
        const eid = spawnUnit(world, UnitKindId.Militia, 0, 2, 2);
        world.cmd.setMoveTarget(eid, 13, 2);
        step(world);
        expect(world.paths.pending).toBe(1);

        world.cmd.despawn(eid);
        step(world);

        expect(world.paths.pending).toBe(0);
        expect(world.paths.lastServiced).toBe(0);
    });

    it("hashes the pending queue, so a differently-ordered backlog diverges", () => {
        const configA = {...WALLED, pathBudget: 0};
        const a = createSimWorld(configA);
        const b = createSimWorld(configA);

        const a1 = spawnUnit(a, UnitKindId.Militia, 0, 2, 2);
        const a2 = spawnUnit(a, UnitKindId.Militia, 0, 2, 4);
        const b1 = spawnUnit(b, UnitKindId.Militia, 0, 2, 2);
        const b2 = spawnUnit(b, UnitKindId.Militia, 0, 2, 4);
        expect(hashWorld(a)).toBe(hashWorld(b));

        // Same units, same goals, opposite request order.
        a.paths.request(a1, 40);
        a.paths.request(a2, 41);
        b.paths.request(b2, 41);
        b.paths.request(b1, 40);

        expect(hashWorld(a)).not.toBe(hashWorld(b));
    });
});

describe("path following", () => {
    it("gets a unit around a wall to a goal it cannot see", () => {
        const world = createSimWorld(WALLED);
        const eid = spawnUnit(world, UnitKindId.Militia, 0, 2, 2);
        world.cmd.setMoveTarget(eid, 13, 2);

        run(world, 400);

        const id = idOf(world, eid);
        expect(world.stores.Position.x[id]).toBeCloseTo(13, 3);
        expect(world.stores.Position.y[id]).toBeCloseTo(2, 3);
        // Arriving clears the order, which is how a caller knows it is done.
        expect(hasComponent(world, eid, world.stores.MoveTarget)).toBe(false);
    });

    it("never steps onto an impassable tile on the way", () => {
        const world = createSimWorld(WALLED);
        const eid = spawnUnit(world, UnitKindId.Militia, 0, 2, 2);
        world.cmd.setMoveTarget(eid, 13, 2);

        const id = idOf(world, eid);
        for (let i = 0; i < 400; i++) {
            step(world);
            const tile = world.map.worldToIndex(
                world.stores.Position.x[id],
                world.stores.Position.y[id],
            );
            expect(world.map.isPassableIndex(tile), `tick ${i}`).toBe(true);
        }
    });

    it("re-plans when a route is longer than the path buffer", () => {
        // A long corridor forces several legs of MAX_PATH waypoints.
        const world = createSimWorld({
            seed: 3,
            capacity: 64,
            mapWidth: 128,
            mapHeight: 8,
            obstacles: [],
        });
        const eid = spawnUnit(world, UnitKindId.Militia, 0, 2, 4);
        world.cmd.setMoveTarget(eid, 120, 4);

        run(world, 800);

        const id = idOf(world, eid);
        expect(world.stores.Position.x[id]).toBeCloseTo(120, 3);
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
