import {getAllEntities} from "bitecs";
import {describe, expect, it} from "vitest";
import {Recorder} from "../replay";
import {UnitKindId} from "../sim/components";
import {OrderType} from "../sim/orders";
import {run, step} from "../sim/tick";
import {createSimWorld, idOf, spawnUnit, type SimConfig, type SimWorld} from "../sim/world";

const CONFIG: Partial<SimConfig> = {
    seed: 17,
    capacity: 256,
    mapWidth: 48,
    mapHeight: 48,
    sectorSize: 8,
};

function live(world: SimWorld): number[] {
    return getAllEntities(world)
        .slice()
        .sort((a, b) => idOf(world, a) - idOf(world, b));
}

function spawnSquad(world: SimWorld, count: number): number[] {
    const units: number[] = [];
    for (let i = 0; i < count; i++) {
        units.push(
            spawnUnit(world, UnitKindId.Militia, 0, 4 + (i % 5) * 0.5, 4 + ((i / 5) | 0) * 0.5),
        );
    }
    return units;
}

describe("formations", () => {
    it("spreads a group into distinct slots around one order position", () => {
        const world = createSimWorld(CONFIG);
        const units = spawnSquad(world, 9);
        const recorder = new Recorder();

        recorder.issueGroup(world, 0, OrderType.Move, units, 30, 30);
        run(world, world.config.orderDelay + 1);

        const {Formation} = world.stores;
        const seen = new Set<string>();
        for (const eid of units) {
            const id = idOf(world, eid);
            seen.add(`${Formation.offsetX[id]},${Formation.offsetY[id]}`);
        }

        // Nine units, nine distinct slots in a 3x3 block.
        expect(seen.size).toBe(9);
        expect(world.config.formationSpacing).toBe(1);
    });

    it("leaves a lone unit at the order position itself", () => {
        const world = createSimWorld(CONFIG);
        const [eid] = spawnSquad(world, 1);
        const recorder = new Recorder();

        recorder.issueGroup(world, 0, OrderType.Move, [eid], 30, 30);
        run(world, world.config.orderDelay + 1);

        const id = idOf(world, eid);
        expect(world.stores.Formation.offsetX[id]).toBe(0);
        expect(world.stores.Formation.offsetY[id]).toBe(0);
    });

    it("assigns slots by handle, not by the order units were listed in", () => {
        const forward = createSimWorld(CONFIG);
        const reversed = createSimWorld(CONFIG);
        const a = spawnSquad(forward, 6);
        const b = spawnSquad(reversed, 6);

        new Recorder().issueGroup(forward, 0, OrderType.Move, a, 30, 30);
        new Recorder().issueGroup(reversed, 0, OrderType.Move, b.slice().reverse(), 30, 30);
        run(forward, forward.config.orderDelay + 1);
        run(reversed, reversed.config.orderDelay + 1);

        for (let i = 0; i < 6; i++) {
            const idA = idOf(forward, a[i]);
            const idB = idOf(reversed, b[i]);
            expect(forward.stores.Formation.offsetX[idA]).toBe(
                reversed.stores.Formation.offsetX[idB],
            );
            expect(forward.stores.Formation.offsetY[idA]).toBe(
                reversed.stores.Formation.offsetY[idB],
            );
        }
    });

    it("drops a slot when the unit is given its own order", () => {
        const world = createSimWorld(CONFIG);
        const units = spawnSquad(world, 4);
        const recorder = new Recorder();

        recorder.issueGroup(world, 0, OrderType.Move, units, 30, 30);
        run(world, world.config.orderDelay + 1);
        const id = idOf(world, units[0]);
        expect(world.stores.Formation.offsetX[id]).not.toBe(0);

        recorder.issue(world, 0, OrderType.Move, units[0], 10, 10);
        run(world, world.config.orderDelay + 1);

        expect(world.stores.Formation.offsetX[id]).toBe(0);
        expect(world.stores.Formation.offsetY[id]).toBe(0);
    });

    /**
     * The property the whole design is arranged around. Offsets rather than
     * separate destinations mean the group keeps one flow goal, so forty units
     * still cost one integration per sector instead of forty.
     */
    it("still shares one segment per sector across the whole group", () => {
        const world = createSimWorld({...CONFIG, segmentBudget: 1});
        const units = spawnSquad(world, 25);
        const recorder = new Recorder();

        recorder.issueGroup(world, 0, OrderType.Move, units, 30, 30);
        run(world, world.config.orderDelay);
        step(world);

        expect(world.paths.lastBuilt).toBe(1);
        expect(world.paths.lastServiced).toBe(25);
        expect(world.paths.pending).toBe(0);
    });

    it("settles a group into a block instead of jostling over one point", () => {
        const world = createSimWorld(CONFIG);
        const units = spawnSquad(world, 25);
        const recorder = new Recorder();

        recorder.issueGroup(world, 0, OrderType.Move, units, 30, 30);
        run(world, 900);

        const {Position, Velocity, Radius} = world.stores;
        const entities = live(world);

        // Everyone stopped: with a slot each there is nothing left to fight over.
        for (const eid of entities) {
            const id = idOf(world, eid);
            const speed = Math.sqrt(Velocity.x[id] ** 2 + Velocity.y[id] ** 2);
            expect(speed, `unit ${eid}`).toBeLessThan(0.01);
        }

        // And nobody overlaps.
        for (let i = 0; i < entities.length; i++) {
            for (let j = i + 1; j < entities.length; j++) {
                const a = idOf(world, entities[i]);
                const b = idOf(world, entities[j]);
                const distance = Math.sqrt(
                    (Position.x[a] - Position.x[b]) ** 2 + (Position.y[a] - Position.y[b]) ** 2,
                );
                expect(distance, `${i} vs ${j}`).toBeGreaterThan(
                    (Radius.value[a] + Radius.value[b]) * 0.9,
                );
            }
        }
    });

    it("keeps a group together through a gap in a wall", () => {
        const world = createSimWorld({
            ...CONFIG,
            obstacles: [
                {x: 24, y: 0, w: 1, h: 20, weight: 0},
                {x: 24, y: 24, w: 1, h: 24, weight: 0},
            ],
        });
        const units = spawnSquad(world, 16);
        const recorder = new Recorder();

        recorder.issueGroup(world, 0, OrderType.Move, units, 40, 10);
        run(world, 1500);

        // All through the gap and settled on the far side.
        for (const eid of live(world)) {
            const id = idOf(world, eid);
            expect(world.stores.Position.x[id], `unit ${eid}`).toBeGreaterThan(25);
            const tile = world.map.worldToIndex(
                world.stores.Position.x[id],
                world.stores.Position.y[id],
            );
            expect(world.map.isPassableIndex(tile)).toBe(true);
        }
    });
});
