import {getAllEntities} from "bitecs";
import {describe, expect, it} from "vitest";
import {
    Recorder,
    replay,
    runWithChecksums,
    SIM_VERSION,
    verifyReplay,
    type ReplayLog,
} from "../replay";
import {OrderType, type Order} from "../sim/orders";
import {diffWorlds, hashWorld} from "../sim/snapshot";
import {step} from "../sim/tick";
import {createSimWorld, idOf, type SimConfig, type SimWorld} from "../sim/world";

/**
 * A 64x64 map with a wall across most of its middle, so routes have to go the
 * long way round and the replay actually exercises A* rather than straight-line
 * steering.
 */
const CONFIG: SimConfig = {
    seed: 0xc0ffee,
    capacity: 1 << 12,
    dt: 0.05,
    orderDelay: 4,
    mapWidth: 64,
    mapHeight: 64,
    tileSize: 1,
    pathBudget: 4,
    obstacles: [{x: 30, y: 0, w: 2, h: 50, weight: 0}],
};
const TICKS = 240;

function liveEntities(world: SimWorld): number[] {
    return getAllEntities(world)
        .slice()
        .sort((a, b) => idOf(world, a) - idOf(world, b));
}

/**
 * A scripted match: spawn units for two players, march them across the map,
 * kill one and reroute another. Keyed off the tick number rather than
 * wall-clock so the script itself is reproducible.
 */
function issueScriptedOrders(world: SimWorld, recorder: Recorder, spawned: number[]): void {
    if (world.tick === 0) {
        for (let u = 0; u < 6; u++) {
            // Spaced closer than their radii, so the separation pass is part
            // of what the replay has to reproduce.
            recorder.issue(world, u % 2, OrderType.Spawn, u % 3, 5 + u * 0.3, 10);
        }
    }
    if (world.tick === 10) {
        for (const eid of liveEntities(world)) spawned.push(eid);
        // Across the wall, so every unit has to route around its open end. With
        // pathBudget 4 and 6 units, the queue also spills into a second tick.
        spawned.forEach((eid, index) => {
            recorder.issue(world, index % 2, OrderType.Move, eid, 50, 12 + index);
        });
    }
    if (world.tick === 80 && spawned.length > 0) {
        recorder.issue(world, 0, OrderType.Damage, spawned[0], 999);
    }
    if (world.tick === 120 && spawned.length > 1) {
        recorder.issue(world, 1, OrderType.Stop, spawned[1]);
        recorder.issue(world, 1, OrderType.Move, spawned[1], 12, 55);
    }
}

function recordMatch(config: SimConfig = CONFIG, ticks = TICKS): ReplayLog {
    const world = createSimWorld(config);
    const recorder = new Recorder();
    const checksums: {tick: number; hash: string}[] = [];
    const spawned: number[] = [];

    for (let i = 0; i < ticks; i++) {
        issueScriptedOrders(world, recorder, spawned);
        step(world);
        checksums.push({tick: world.tick, hash: hashWorld(world)});
    }

    return {simVersion: SIM_VERSION, config, ticks, orders: recorder.orders, checksums};
}

describe("determinism", () => {
    it("produces an identical checksum sequence for two independent worlds", () => {
        const a = recordMatch();
        const b = recordMatch();

        expect(b.checksums).toEqual(a.checksums);
        expect(a.checksums.at(-1)!.hash).toMatch(/^[0-9a-f]{16}$/);
    });

    it("reproduces a recorded match from its order log alone", () => {
        const log = recordMatch();

        expect(verifyReplay(log)).toEqual({ok: true, divergedAtTick: -1});
    });

    it("is unaffected by the order in which a tick's orders arrive", () => {
        const log = recordMatch();

        // Stands in for packets arriving in a different sequence on a peer.
        const shuffled = shuffle(log.orders, 0x1234);
        expect(shuffled.map((o) => o.seq)).not.toEqual(log.orders.map((o) => o.seq));

        const {checksums} = replay({...log, orders: shuffled});
        expect(checksums).toEqual(log.checksums);
    });

    it("diverges when the seed changes", () => {
        const a = recordMatch();
        const b = recordMatch({...CONFIG, seed: CONFIG.seed + 1});

        expect(b.checksums.at(-1)!.hash).not.toBe(a.checksums.at(-1)!.hash);
    });

    it("names the tick and the field when two worlds disagree", () => {
        const log = recordMatch(CONFIG, 40);
        const left = replay(log).world;
        const right = replay(log).world;

        expect(hashWorld(left)).toBe(hashWorld(right));

        // The smallest divergence a non-deterministic system could introduce.
        const entities = liveEntities(right);
        expect(entities.length).toBeGreaterThan(0);
        right.stores.Position.x[idOf(right, entities[0])] += 1e-9;

        expect(hashWorld(left)).not.toBe(hashWorld(right));

        const diffs = diffWorlds(left, right);
        expect(diffs).toHaveLength(1);
        expect(diffs[0]).toMatchObject({component: "Position", field: "x"});
    });

    it("rejects a log recorded against a different sim version", () => {
        const log = recordMatch(CONFIG, 20);

        expect(verifyReplay({...log, simVersion: SIM_VERSION + 1}).ok).toBe(false);
    });

    it("reaches the same state whether stepped in one run or resumed", () => {
        const log = recordMatch(CONFIG, 60);
        const oneGo = replay(log).world;

        const resumed = createSimWorld(log.config);
        resumed.orders.scheduleAll(log.orders);
        runWithChecksums(resumed, 25);
        runWithChecksums(resumed, 35);

        expect(hashWorld(resumed)).toBe(hashWorld(oneGo));
    });

    it("removes a unit whose health reaches zero", () => {
        const log = recordMatch(CONFIG, 100);
        const before = replay({...log, ticks: 80}).world;
        const after = replay(log).world;

        expect(liveEntities(after).length).toBe(liveEntities(before).length - 1);
    });
});

/** Deterministic Fisher-Yates, so the shuffle test is itself reproducible. */
function shuffle(orders: readonly Order[], seed: number): Order[] {
    const out = orders.slice();
    let state = seed >>> 0;
    for (let i = out.length - 1; i > 0; i--) {
        state = (Math.imul(state, 1664525) + 1013904223) >>> 0;
        const j = state % (i + 1);
        [out[i], out[j]] = [out[j], out[i]];
    }
    return out;
}
