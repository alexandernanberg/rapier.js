import {
    addComponent,
    addEntity,
    createEntityIndex,
    createWorld,
    entityExists,
    getId,
    getVersion,
    hasComponent,
    removeComponent,
    removeEntity,
    withVersioning,
    type EntityId,
    type World,
} from "bitecs";
import {Rng} from "../core/rand";
import {CommandBuffer, CommandKind, type Command} from "./command_buffer";
import {createStores, UNIT_STATS, type Stores} from "./components";
import {OrderQueue, OrderType, type Order} from "./orders";

export interface SimConfig {
    readonly seed: number;
    /** Upper bound on *concurrent* entities; ids are recycled. */
    readonly capacity: number;
    /** Fixed timestep, in seconds. 20 Hz is a typical RTS sim rate. */
    readonly dt: number;
    /** How many ticks ahead an issued order executes. The latency budget. */
    readonly orderDelay: number;
}

/** Half-width of the random offset applied to a spawn position. */
const SPAWN_JITTER = 0.05;

export const DEFAULT_CONFIG: SimConfig = {
    seed: 0x5eed,
    capacity: 1 << 14,
    dt: 0.05,
    orderDelay: 4,
};

export interface SimContext {
    tick: number;
    readonly config: SimConfig;
    readonly rng: Rng;
    readonly stores: Stores;
    readonly cmd: CommandBuffer;
    readonly orders: OrderQueue;
    readonly entityIndex: ReturnType<typeof createEntityIndex>;
}

export type SimWorld = World<SimContext>;

export function createSimWorld(config: Partial<SimConfig> = {}): SimWorld {
    const merged = {...DEFAULT_CONFIG, ...config};
    const entityIndex = createEntityIndex(withVersioning());
    return createWorld<SimContext>(entityIndex, {
        tick: 0,
        config: merged,
        rng: new Rng(merged.seed),
        stores: createStores(merged.capacity),
        cmd: new CommandBuffer(),
        orders: new OrderQueue(),
        entityIndex,
    });
}

/**
 * Raw array index for an entity handle.
 *
 * bitECS packs a generation counter into the handle so a stale reference can be
 * detected, which means the handle is NOT a valid array index. Every component
 * read goes through this. Getting it wrong reads another unit's data — and
 * would still be deterministic, so the desync hash will not save you. Always
 * use it.
 */
export function idOf(world: SimWorld, eid: EntityId): number {
    return getId(world.entityIndex, eid);
}

export function versionOf(world: SimWorld, eid: EntityId): number {
    return getVersion(world.entityIndex, eid);
}

/** True if the handle still refers to the entity it was taken from. */
export function isAlive(world: SimWorld, eid: EntityId): boolean {
    return entityExists(world, eid);
}

/**
 * Applies queued structural changes. Called once per tick, between systems and
 * the end of the tick — never from inside a system.
 */
export function flushCommands(world: SimWorld): void {
    const commands = world.cmd.drain();
    for (let i = 0; i < commands.length; i++) {
        applyCommand(world, commands[i]);
    }
}

function applyCommand(world: SimWorld, command: Command): void {
    const {stores} = world;
    switch (command.kind) {
        case CommandKind.Spawn: {
            spawnUnit(world, command.a, command.b, command.c, command.d);
            break;
        }
        case CommandKind.Despawn: {
            if (entityExists(world, command.a)) removeEntity(world, command.a);
            break;
        }
        case CommandKind.SetMoveTarget: {
            if (!entityExists(world, command.a)) break;
            const id = idOf(world, command.a);
            addComponent(world, command.a, stores.MoveTarget);
            stores.MoveTarget.x[id] = command.b;
            stores.MoveTarget.y[id] = command.c;
            break;
        }
        case CommandKind.ClearMoveTarget: {
            if (!entityExists(world, command.a)) break;
            if (hasComponent(world, command.a, stores.MoveTarget)) {
                removeComponent(world, command.a, stores.MoveTarget);
            }
            const id = idOf(world, command.a);
            stores.Velocity.x[id] = 0;
            stores.Velocity.y[id] = 0;
            break;
        }
        case CommandKind.Damage: {
            if (!entityExists(world, command.a)) break;
            const id = idOf(world, command.a);
            stores.Health.current[id] -= command.b | 0;
            break;
        }
    }
}

/** Immediate spawn. Systems must go through `world.cmd.spawn` instead. */
export function spawnUnit(
    world: SimWorld,
    kind: number,
    player: number,
    x: number,
    y: number,
): EntityId {
    const {stores} = world;
    const eid = addEntity(world);
    const id = idOf(world, eid);

    addComponent(world, eid, stores.Position);
    addComponent(world, eid, stores.Velocity);
    addComponent(world, eid, stores.Facing);
    addComponent(world, eid, stores.Speed);
    addComponent(world, eid, stores.Health);
    addComponent(world, eid, stores.Owner);
    addComponent(world, eid, stores.UnitKind);

    const stats = UNIT_STATS[kind] ?? UNIT_STATS[0];

    // Nudge spawns off the exact rally point so a batch does not stack into a
    // single pile. Drawing from the sim rng (never Math.random) keeps this part
    // of the simulation and therefore part of the checksum.
    const jitterX = world.rng.nextRange(-SPAWN_JITTER, SPAWN_JITTER);
    const jitterY = world.rng.nextRange(-SPAWN_JITTER, SPAWN_JITTER);

    stores.Position.x[id] = x + jitterX;
    stores.Position.y[id] = y + jitterY;
    stores.Velocity.x[id] = 0;
    stores.Velocity.y[id] = 0;
    stores.Facing.angle[id] = 0;
    stores.Speed.value[id] = stats.speed;
    stores.Health.current[id] = stats.health;
    stores.Health.max[id] = stats.health;
    stores.Owner.player[id] = player;
    stores.UnitKind.kind[id] = kind;

    return eid;
}

/**
 * Translates this tick's player orders into simulation commands.
 *
 * Orders arrive already sorted into canonical order, so this loop is the point
 * at which network non-determinism has been fully squeezed out.
 */
export function applyOrders(world: SimWorld, orders: readonly Order[]): void {
    for (let i = 0; i < orders.length; i++) {
        const order = orders[i];
        switch (order.type) {
            case OrderType.Spawn:
                world.cmd.spawn(order.a, order.player, order.b, order.c);
                break;
            case OrderType.Move:
                world.cmd.setMoveTarget(order.a, order.b, order.c);
                break;
            case OrderType.Stop:
                world.cmd.clearMoveTarget(order.a);
                break;
            case OrderType.Damage:
                world.cmd.damage(order.a, order.b);
                break;
        }
    }
}
