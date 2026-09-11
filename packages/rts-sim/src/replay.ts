import {makeOrder, type Order, type OrderTypeValue} from "./sim/orders";
import {hashWorld} from "./sim/snapshot";
import {step} from "./sim/tick";
import {createSimWorld, type SimConfig, type SimWorld} from "./sim/world";

/**
 * Records every order a session issues, which is the entire save format.
 *
 * A lockstep sim is a pure function of (config, order log), so the log plus the
 * seed reproduces the match exactly. That is what makes replays, saves and
 * desync bisection the same feature.
 */
export class Recorder {
    private readonly seqs = new Map<number, number>();
    readonly orders: Order[] = [];

    /**
     * Issues an order from `player`, scheduled `orderDelay` ticks ahead so it
     * executes on a tick every client can have received it by.
     */
    issue(
        world: SimWorld,
        player: number,
        type: OrderTypeValue,
        a = 0,
        b = 0,
        c = 0,
        d = 0,
    ): Order {
        const seq = (this.seqs.get(player) ?? 0) + 1;
        this.seqs.set(player, seq);

        const order = makeOrder(
            world.tick + world.config.orderDelay,
            player,
            seq,
            type,
            a,
            b,
            c,
            d,
        );
        world.orders.schedule(order);
        this.orders.push(order);
        return order;
    }
}

export interface Checksum {
    readonly tick: number;
    readonly hash: string;
}

export interface ReplayLog {
    /** Bump when the tick shape, system order or component set changes. */
    readonly simVersion: number;
    readonly config: SimConfig;
    readonly ticks: number;
    readonly orders: readonly Order[];
    readonly checksums: readonly Checksum[];
}

export const SIM_VERSION = 4;

export interface ReplayOptions {
    /** Hash every Nth tick. 1 in tests; higher in a real match. */
    readonly checksumEvery?: number;
}

/** Replays a log into a fresh world and returns the checksums it produced. */
export function replay(
    log: ReplayLog,
    options: ReplayOptions = {},
): {
    world: SimWorld;
    checksums: Checksum[];
} {
    const every = options.checksumEvery ?? 1;
    const world = createSimWorld(log.config);
    world.orders.scheduleAll(log.orders);

    const checksums: Checksum[] = [];
    for (let i = 0; i < log.ticks; i++) {
        step(world);
        if (world.tick % every === 0) {
            checksums.push({tick: world.tick, hash: hashWorld(world)});
        }
    }

    return {world, checksums};
}

export interface VerifyResult {
    readonly ok: boolean;
    /** First tick whose hash disagreed, or -1 when the replay matched. */
    readonly divergedAtTick: number;
    readonly expected?: string;
    readonly actual?: string;
}

/**
 * Re-runs a log and compares against the checksums recorded alongside it.
 *
 * This is the regression test that makes an ECS migration, a refactor of the
 * movement code, or a change to the tick shape a pass/fail question instead of
 * a silent-bug hunt. Run it in CI.
 */
export function verifyReplay(log: ReplayLog, options: ReplayOptions = {}): VerifyResult {
    if (log.simVersion !== SIM_VERSION) {
        return {ok: false, divergedAtTick: 0};
    }

    const {checksums} = replay(log, options);
    const expected = log.checksums;
    const count = Math.min(expected.length, checksums.length);

    for (let i = 0; i < count; i++) {
        if (expected[i].hash !== checksums[i].hash) {
            return {
                ok: false,
                divergedAtTick: expected[i].tick,
                expected: expected[i].hash,
                actual: checksums[i].hash,
            };
        }
    }

    if (expected.length !== checksums.length) {
        return {ok: false, divergedAtTick: checksums.length};
    }

    return {ok: true, divergedAtTick: -1};
}

/** Runs a world forward, collecting a checksum every `every` ticks. */
export function runWithChecksums(world: SimWorld, ticks: number, every = 1): Checksum[] {
    const checksums: Checksum[] = [];
    for (let i = 0; i < ticks; i++) {
        step(world);
        if (world.tick % every === 0) {
            checksums.push({tick: world.tick, hash: hashWorld(world)});
        }
    }
    return checksums;
}
