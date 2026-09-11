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
import {createStores, MAX_PATH, PathState, UNIT_STATS, type Stores} from "./components";
import {SpatialHash} from "./grid/spatial_hash";
import {TileMap} from "./grid/tile_map";
import {OrderQueue, OrderType, type Order} from "./orders";
import {PathQueue} from "./path/path_queue";

/** A rectangle of modified terrain — a cliff, a lake, a building footprint. */
export interface TerrainRect {
    readonly x: number;
    readonly y: number;
    readonly w: number;
    readonly h: number;
    /** 0 is impassable; 1 is normal going; higher is slower. */
    readonly weight: number;
}

export interface SimConfig {
    readonly seed: number;
    /** Upper bound on *concurrent* entities; ids are recycled. */
    readonly capacity: number;
    /** Fixed timestep, in seconds. 20 Hz is a typical RTS sim rate. */
    readonly dt: number;
    /** How many ticks ahead an issued order executes. The latency budget. */
    readonly orderDelay: number;
    readonly mapWidth: number;
    readonly mapHeight: number;
    readonly tileSize: number;
    /**
     * Route requests serviced per tick. A count, not a time slice — see
     * `PathQueue`.
     *
     * Must be tuned to map size. A corner-to-corner flat A* search costs
     * roughly 0.7ms on a 64x64 map, 2.1ms on 128x128 and 7.5ms on 256x256
     * (measured; it scales with tile count). Keep `pathBudget * msPerSearch`
     * inside about a fifth of `dt`.
     */
    readonly pathBudget: number;
    /**
     * Terrain, declared rather than mutated, so the config alone reproduces the
     * map. A replay log carries this and nothing else about the terrain.
     */
    readonly obstacles: readonly TerrainRect[];
}

/** Half-width of the random offset applied to a spawn position. */
const SPAWN_JITTER = 0.05;

export const DEFAULT_CONFIG: SimConfig = {
    seed: 0x5eed,
    capacity: 1 << 14,
    dt: 0.05,
    orderDelay: 4,
    mapWidth: 128,
    mapHeight: 128,
    tileSize: 1,
    // 4 searches x ~2.1ms is about 8ms of a 50ms tick on the default 128x128
    // map. Raise it only alongside a cheaper search — see the README.
    pathBudget: 4,
    obstacles: [],
};

/**
 * Preallocated working memory for systems.
 *
 * Transient by contract: every tick fully writes what it reads, so none of this
 * is hashed. Anything that needs to survive a tick belongs in a component or in
 * a structure with its own `hashInto`.
 */
export interface SimScratch {
    readonly pushX: Float64Array;
    readonly pushY: Float64Array;
    readonly neighbours: Int32Array;
    /** Raw array indices for the entities a system is iterating. */
    readonly rawIds: Int32Array;
}

/** Most neighbours one unit considers when resolving overlap. */
export const MAX_NEIGHBOURS = 32;

export interface SimContext {
    tick: number;
    readonly config: SimConfig;
    readonly rng: Rng;
    readonly stores: Stores;
    readonly cmd: CommandBuffer;
    readonly orders: OrderQueue;
    readonly map: TileMap;
    readonly paths: PathQueue;
    readonly grid: SpatialHash;
    readonly scratch: SimScratch;
    readonly entityIndex: ReturnType<typeof createEntityIndex>;
}

export type SimWorld = World<SimContext>;

export function createSimWorld(config: Partial<SimConfig> = {}): SimWorld {
    const merged = {...DEFAULT_CONFIG, ...config};
    const entityIndex = createEntityIndex(withVersioning());

    const map = new TileMap({
        width: merged.mapWidth,
        height: merged.mapHeight,
        tileSize: merged.tileSize,
    });
    for (const rect of merged.obstacles) {
        map.fillRect(rect.x, rect.y, rect.w, rect.h, rect.weight);
    }

    return createWorld<SimContext>(entityIndex, {
        tick: 0,
        config: merged,
        rng: new Rng(merged.seed),
        stores: createStores(merged.capacity),
        cmd: new CommandBuffer(),
        orders: new OrderQueue(),
        map,
        paths: new PathQueue(map),
        grid: new SpatialHash(
            merged.mapWidth * merged.tileSize,
            merged.mapHeight * merged.tileSize,
            2 * merged.tileSize,
            merged.capacity,
        ),
        scratch: {
            pushX: new Float64Array(merged.capacity),
            pushY: new Float64Array(merged.capacity),
            neighbours: new Int32Array(MAX_NEIGHBOURS),
            rawIds: new Int32Array(merged.capacity),
        },
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
            if (!entityExists(world, command.a)) break;
            world.paths.cancel(command.a);
            removeEntity(world, command.a);
            break;
        }
        case CommandKind.SetMoveTarget: {
            if (!entityExists(world, command.a)) break;
            const id = idOf(world, command.a);
            addComponent(world, command.a, stores.MoveTarget);
            stores.MoveTarget.x[id] = command.b;
            stores.MoveTarget.y[id] = command.c;
            // A new destination invalidates the route in hand; the request
            // system will queue a fresh search next tick.
            resetPath(world, id);
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
            world.paths.cancel(command.a);
            resetPath(world, id);
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

function resetPath(world: SimWorld, id: number): void {
    const {Path} = world.stores;
    Path.state[id] = PathState.None;
    Path.goal[id] = -1;
    Path.length[id] = 0;
    Path.cursor[id] = 0;
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
    addComponent(world, eid, stores.Radius);
    addComponent(world, eid, stores.Health);
    addComponent(world, eid, stores.Owner);
    addComponent(world, eid, stores.UnitKind);
    // Path is always present, with `state = None` standing in for "no route".
    // Keeping it resident makes setting a route a plain data write rather than
    // an archetype change, which is both faster and one less structural edit to
    // sequence.
    addComponent(world, eid, stores.Path);

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
    stores.Radius.value[id] = stats.radius;
    stores.Health.current[id] = stats.health;
    stores.Health.max[id] = stats.health;
    stores.Owner.player[id] = player;
    stores.UnitKind.kind[id] = kind;
    resetPath(world, id);
    stores.Path.tiles.fill(0, id * MAX_PATH, (id + 1) * MAX_PATH);

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
