import {hasComponent} from "bitecs";
import {describe, expect, it} from "vitest";
import {cos, sin} from "../core/math";
import {Recorder} from "../replay";
import {UnitKindId} from "../sim/components";
import {FormationShape} from "../sim/formation";
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

/** Worst distance between any member and the slot it should be standing in. */
function maxSlotError(world: SimWorld, units: readonly number[]): number {
    const {Position, Formation, Facing} = world.stores;
    const leader = Formation.leader[idOf(world, units[0])];
    if (leader === -1) return Number.POSITIVE_INFINITY;

    const lid = idOf(world, leader);
    const angle = Facing.angle[lid];
    const c = cos(angle);
    const sn = sin(angle);

    let worst = 0;
    for (const eid of units) {
        const id = idOf(world, eid);
        const slotX = Position.x[lid] + Formation.localX[id] * c - Formation.localY[id] * sn;
        const slotY = Position.y[lid] + Formation.localX[id] * sn + Formation.localY[id] * c;
        const error = Math.sqrt((Position.x[id] - slotX) ** 2 + (Position.y[id] - slotY) ** 2);
        if (error > worst) worst = error;
    }
    return worst;
}

/** Mean distance from the group's own centroid. */
function spread(world: SimWorld, units: readonly number[]): number {
    const {Position} = world.stores;
    let cx = 0;
    let cy = 0;
    for (const eid of units) {
        const id = idOf(world, eid);
        cx += Position.x[id];
        cy += Position.y[id];
    }
    cx /= units.length;
    cy /= units.length;

    let sum = 0;
    for (const eid of units) {
        const id = idOf(world, eid);
        sum += Math.sqrt((Position.x[id] - cx) ** 2 + (Position.y[id] - cy) ** 2);
    }
    return sum / units.length;
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
        const leaders = new Set<number>();
        for (const eid of units) {
            const id = idOf(world, eid);
            seen.add(`${Formation.localX[id]},${Formation.localY[id]}`);
            leaders.add(Formation.leader[id]);
        }

        // Nine units, nine distinct slots in a 3x3 block, one shared leader.
        expect(seen.size).toBe(9);
        expect(leaders.size).toBe(1);
        expect(leaders.has(-1)).toBe(false);
    });

    it("gives a lone unit the centre slot, which is the order position", () => {
        const world = createSimWorld(CONFIG);
        const [eid] = spawnSquad(world, 1);
        const recorder = new Recorder();

        recorder.issueGroup(world, 0, OrderType.Move, [eid], 30, 30);
        run(world, world.config.orderDelay + 1);

        const id = idOf(world, eid);
        expect(world.stores.Formation.localX[id]).toBe(0);
        expect(world.stores.Formation.localY[id]).toBe(0);
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
            expect(forward.stores.Formation.localX[idA]).toBe(
                reversed.stores.Formation.localX[idB],
            );
            expect(forward.stores.Formation.localY[idA]).toBe(
                reversed.stores.Formation.localY[idB],
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
        expect(world.stores.Formation.leader[id]).not.toBe(-1);

        recorder.issue(world, 0, OrderType.Move, units[0], 10, 10);
        run(world, world.config.orderDelay + 1);

        expect(world.stores.Formation.leader[id]).toBe(-1);
        expect(world.stores.Formation.localX[id]).toBe(0);
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
        // 25 members plus their leader, which paths like any other entity.
        expect(world.paths.lastServiced).toBe(26);
        expect(world.paths.pending).toBe(0);
    });

    it("settles a group into a block instead of jostling over one point", () => {
        const world = createSimWorld(CONFIG);
        const units = spawnSquad(world, 25);
        const recorder = new Recorder();

        recorder.issueGroup(world, 0, OrderType.Move, units, 30, 30);
        run(world, 900);

        const {Position, Velocity, Radius} = world.stores;
        // Iterate the squad, not every entity: the formation's leader is an
        // entity as well, and it has no radius to overlap with.
        const entities = units;

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
        for (const eid of units) {
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

describe("cohesive marching", () => {
    const MARCH: Partial<SimConfig> = {
        seed: 31,
        capacity: 256,
        mapWidth: 96,
        mapHeight: 96,
        sectorSize: 16,
    };

    function marchingSquad(count: number): {world: SimWorld; units: number[]} {
        const world = createSimWorld(MARCH);
        const units: number[] = [];
        for (let i = 0; i < count; i++) {
            units.push(
                spawnUnit(world, UnitKindId.Militia, 0, 6 + (i % 6) * 0.9, 6 + ((i / 6) | 0) * 0.9),
            );
        }
        new Recorder().issueGroup(world, 0, OrderType.Move, units, 80, 80, FormationShape.Block);
        run(world, world.config.orderDelay + 1);
        return {world, units};
    }

    it("holds its shape for the whole march", () => {
        const {world, units} = marchingSquad(36);
        const leader = world.stores.Formation.leader[idOf(world, units[0])];

        // A 6x6 block at the default spacing spans 6x6, so its mean distance
        // from the centroid is about 2.6 whatever heading it is on. Holding that
        // steady while under way is what "in formation" means.
        //
        // Sampled only while the leader still has somewhere to be: the ranks
        // compress briefly as the rear catches up at the destination, which is
        // the arrival transient rather than a loss of formation, and the
        // settling test below covers where it ends up.
        let sampled = 0;
        for (let t = 0; t < 900; t++) {
            step(world);
            if (!hasComponent(world, leader, world.stores.MoveTarget)) break;
            if (t % 100 !== 0) continue;
            sampled++;
            expect(spread(world, units), `tick ${t}`).toBeGreaterThan(2.0);
            expect(spread(world, units), `tick ${t}`).toBeLessThan(3.4);
        }
        expect(sampled).toBeGreaterThan(4);
    });

    it("settles exactly onto its slots", () => {
        const {world, units} = marchingSquad(36);
        run(world, 1800);

        expect(maxSlotError(world, units)).toBeLessThan(0.05);
    });

    /**
     * A unit that dropped its order on reaching its slot would leave the
     * movement query, and the next arrival's separation push would shove it off
     * station with nothing left to pull it back. Members therefore keep their
     * order while they belong to a formation, and the *group* reports completion
     * through its leader.
     */
    it("keeps a standing formation tidy as latecomers arrive", () => {
        const {world, units} = marchingSquad(36);
        run(world, 1800);

        const leader = world.stores.Formation.leader[idOf(world, units[0])];
        expect(hasComponent(world, leader, world.stores.MoveTarget)).toBe(false);
        for (const eid of units) {
            expect(hasComponent(world, eid, world.stores.MoveTarget), `unit ${eid}`).toBe(true);
        }

        run(world, 600);
        expect(maxSlotError(world, units)).toBeLessThan(0.05);
    });

    it("moves the leader slower than its members, so stragglers can re-form", () => {
        const {world, units} = marchingSquad(9);
        const leader = world.stores.Formation.leader[idOf(world, units[0])];
        const memberSpeed = world.stores.Speed.value[idOf(world, units[0])];

        expect(world.stores.Speed.value[idOf(world, leader)]).toBeCloseTo(
            memberSpeed * world.config.formationLeaderSpeed,
            6,
        );
        expect(world.config.formationLeaderSpeed).toBeLessThan(1);
    });

    it("keeps the leader out of separation and out of the death system", () => {
        const {world, units} = marchingSquad(9);
        const leader = world.stores.Formation.leader[idOf(world, units[0])];

        expect(hasComponent(world, leader, world.stores.Radius)).toBe(false);
        expect(hasComponent(world, leader, world.stores.Health)).toBe(false);
        expect(hasComponent(world, leader, world.stores.FormationLeader)).toBe(true);
    });

    it("retires a leader once its last member has left", () => {
        const {world, units} = marchingSquad(4);
        const leader = world.stores.Formation.leader[idOf(world, units[0])];
        expect(leader).not.toBe(-1);

        for (const eid of units) world.cmd.despawn(eid);
        run(world, 3);

        expect(hasComponent(world, leader, world.stores.FormationLeader)).toBe(false);
    });

    it("rotates a line to sit across the direction of travel", () => {
        const spans: Record<string, [number, number]> = {};

        for (const [name, gx, gy] of [
            ["east", 80, 48],
            ["north", 48, 8],
        ] as [string, number, number][]) {
            const world = createSimWorld(MARCH);
            const units: number[] = [];
            for (let i = 0; i < 9; i++) {
                units.push(
                    spawnUnit(
                        world,
                        UnitKindId.Militia,
                        0,
                        46 + (i % 3) * 0.8,
                        46 + ((i / 3) | 0) * 0.8,
                    ),
                );
            }
            new Recorder().issueGroup(world, 0, OrderType.Move, units, gx, gy, FormationShape.Line);
            run(world, 1500);

            const {Position} = world.stores;
            const xs = units.map((eid) => Position.x[idOf(world, eid)]);
            const ys = units.map((eid) => Position.y[idOf(world, eid)]);
            spans[name] = [Math.max(...xs) - Math.min(...xs), Math.max(...ys) - Math.min(...ys)];
        }

        // Marching east the rank runs north-south; marching north it runs
        // east-west. A shape is written once and works at any heading.
        expect(spans.east[1]).toBeGreaterThan(8);
        expect(spans.east[0]).toBeLessThan(2);
        expect(spans.north[0]).toBeGreaterThan(8);
        expect(spans.north[1]).toBeLessThan(2);
    });
});
