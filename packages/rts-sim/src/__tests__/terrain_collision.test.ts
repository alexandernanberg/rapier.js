import {getAllEntities} from "bitecs";
import {describe, expect, it} from "vitest";
import {UnitKindId} from "../sim/components";
import {step} from "../sim/tick";
import {createSimWorld, idOf, spawnUnit, type SimWorld} from "../sim/world";

function violations(world: SimWorld): number {
    let count = 0;
    for (const eid of getAllEntities(world)) {
        const id = idOf(world, eid);
        const tile = world.map.worldToIndex(
            world.stores.Position.x[id],
            world.stores.Position.y[id],
        );
        if (!world.map.isPassableIndex(tile)) count++;
    }
    return count;
}

describe("terrain collision", () => {
    /**
     * The case pathfinding cannot cover on its own: separation pushes units
     * with no idea where the walls are, so a crowd squeezed against a building
     * shoves its neighbours through it. Before `moveClamped` this run logged
     * 1281 violations, first appearing at tick 63.
     */
    it("keeps a crowd out of walls while it is herded through a maze", () => {
        const world = createSimWorld({
            seed: 4,
            capacity: 512,
            mapWidth: 48,
            mapHeight: 48,
            sectorSize: 8,
            obstacles: [
                {x: 12, y: 0, w: 1, h: 40, weight: 0},
                {x: 24, y: 8, w: 1, h: 40, weight: 0},
                {x: 36, y: 0, w: 1, h: 40, weight: 0},
            ],
        });

        for (let i = 0; i < 60; i++) {
            const eid = spawnUnit(
                world,
                UnitKindId.Militia,
                0,
                2 + (i % 6) * 0.4,
                2 + ((i / 6) | 0) * 0.4,
            );
            world.cmd.setMoveTarget(eid, 44, 44);
        }

        for (let t = 0; t < 1500; t++) {
            step(world);
            expect(violations(world), `tick ${t}`).toBe(0);
        }
    });

    /**
     * A goal a few tiles away whose only approach leaves the segment's window.
     * The unit is at (8,4) and the goal at (20,4) with a wall between them from
     * y=0 to y=19, so the detour runs well outside the 3x3 window around the
     * unit's sector. A segment that aimed straight at the goal would find no
     * route and jam against the wall's face; walking the abstract route and
     * stopping where it exits the window is what gets this right.
     */
    it("routes around a wall whose detour leaves the segment window", () => {
        const world = createSimWorld({
            seed: 1,
            capacity: 16,
            mapWidth: 32,
            mapHeight: 32,
            sectorSize: 8,
            obstacles: [{x: 10, y: 0, w: 1, h: 20, weight: 0}],
        });

        const eid = spawnUnit(world, UnitKindId.Militia, 0, 8, 4);
        const id = idOf(world, eid);
        world.cmd.setMoveTarget(eid, 20, 4);

        let wentSouth = false;
        for (let t = 0; t < 400; t++) {
            step(world);
            if (world.stores.Position.y[id] > 18) wentSouth = true;
            expect(violations(world), `tick ${t}`).toBe(0);
        }

        // It went the long way round and arrived, rather than pressing into
        // the wall or being marked unreachable.
        expect(wentSouth).toBe(true);
        expect(world.stores.Position.x[id]).toBeCloseTo(20, 2);
        expect(world.stores.Position.y[id]).toBeCloseTo(4, 2);
    });

    it("keeps units on the map when ordered at its edge", () => {
        const world = createSimWorld({seed: 2, capacity: 16, mapWidth: 32, mapHeight: 32});
        const eid = spawnUnit(world, UnitKindId.Militia, 0, 1, 1);
        const id = idOf(world, eid);

        world.cmd.setMoveTarget(eid, 0.1, 0.1);
        for (let t = 0; t < 100; t++) step(world);

        expect(world.stores.Position.x[id]).toBeGreaterThanOrEqual(0);
        expect(world.stores.Position.y[id]).toBeGreaterThanOrEqual(0);
        expect(violations(world)).toBe(0);
    });
});
