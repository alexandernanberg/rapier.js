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
import {createStores, PathState, UNIT_STATS, type Stores} from "./components";
import {SpatialHash} from "./grid/spatial_hash";
import {TileMap} from "./grid/tile_map";
import {OrderQueue, OrderType, type Order} from "./orders";
import {Pathfinder} from "./path/pathfinder";

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
     * Cells per sector edge. Sectors bound the portal graph and the window a
     * flow segment integrates, so this is the main cost dial: a segment covers
     * a 3x3 block, so it integrates about `9 * sectorSize^2` cells regardless
     * of how large the map is.
     */
    readonly sectorSize: number;
    /**
     * Flow segments built per tick. A count, not a time slice — see
     * `Pathfinder`. Reading a cached segment is free and unbudgeted.
     */
    readonly segmentBudget: number;
    /** Segments kept cached. A miss re-requests, which is harmless. */
    readonly segmentCapacity: number;
    /** World units between formation slots. Roughly two unit diameters. */
    readonly formationSpacing: number;
    /**
     * Distance at which a unit starts steering at its formation slot instead of
     * following the flow. Bounds the line-of-sight test's ray length, and means
     * a group marches as a crowd and fans out only on arrival.
     */
    readonly formationRange: number;
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
    sectorSize: 16,
    segmentBudget: 4,
    segmentCapacity: 64,
    formationSpacing: 1,
    formationRange: 8,
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
    /** Out-param for the point a unit steers at this tick: [x, y]. */
    readonly steer: Float64Array;
    /** Entity handles of one formation being assembled. */
    readonly formation: Int32Array;
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
    readonly paths: Pathfinder;
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
        paths: new Pathfinder(map, merged.sectorSize, merged.segmentCapacity),
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
            steer: new Float64Array(2),
            formation: new Int32Array(merged.capacity),
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
            // A new destination drops any slot held from a previous order; a
            // grouped order re-sets it immediately after.
            stores.Formation.offsetX[id] = 0;
            stores.Formation.offsetY[id] = 0;
            // A new destination invalidates the segment in hand; the request
            // system queues a fresh one next tick.
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
        case CommandKind.SetFormationSlot: {
            if (!entityExists(world, command.a)) break;
            const id = idOf(world, command.a);
            stores.Formation.offsetX[id] = command.b;
            stores.Formation.offsetY[id] = command.c;
            break;
        }
    }
}

function resetPath(world: SimWorld, id: number): void {
    const {Path} = world.stores;
    Path.state[id] = PathState.None;
    Path.goal[id] = -1;
    Path.sector[id] = -1;
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
    addComponent(world, eid, stores.Formation);
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
    stores.Formation.offsetX[id] = 0;
    stores.Formation.offsetY[id] = 0;
    stores.Speed.value[id] = stats.speed;
    stores.Radius.value[id] = stats.radius;
    stores.Health.current[id] = stats.health;
    stores.Health.max[id] = stats.health;
    stores.Owner.player[id] = player;
    stores.UnitKind.kind[id] = kind;
    resetPath(world, id);

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
                if (order.group !== 0) {
                    // Grouped orders are issued consecutively, so a group's
                    // orders are contiguous once sorted by (player, seq).
                    i = assignFormation(world, orders, i) - 1;
                } else {
                    world.cmd.setMoveTarget(order.a, order.b, order.c);
                }
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

/**
 * Turns one player action into a formation, and returns the index just past it.
 *
 * Every unit keeps the *same* destination — so the group still shares one flow
 * segment per sector — and gets an offset from it instead. Slots fill a centred
 * grid, assigned by ascending entity id so the layout is reproducible rather
 * than dependent on selection order.
 */
function assignFormation(world: SimWorld, orders: readonly Order[], start: number): number {
    const group = orders[start].group;
    const player = orders[start].player;

    let end = start;
    while (
        end < orders.length &&
        orders[end].group === group &&
        orders[end].player === player &&
        orders[end].type === OrderType.Move
    ) {
        end++;
    }

    const count = Math.min(end - start, world.scratch.formation.length);
    const members = world.scratch.formation.subarray(0, count);
    for (let i = 0; i < count; i++) members[i] = orders[start + i].a;

    // Sort by handle, so which unit takes which slot does not depend on the
    // order the client happened to list them in. Handles are distinct, so the
    // comparator is a strict total order and the result is unique.
    members.sort((a, b) => a - b);

    const spacing = world.config.formationSpacing;
    const columns = Math.ceil(Math.sqrt(count));
    const rows = Math.ceil(count / columns);

    for (let i = 0; i < count; i++) {
        const eid = members[i];
        const column = i % columns;
        const row = (i / columns) | 0;

        world.cmd.setMoveTarget(eid, orders[start].b, orders[start].c);
        if (count > 1) {
            world.cmd.setFormationSlot(
                eid,
                (column - (columns - 1) / 2) * spacing,
                (row - (rows - 1) / 2) * spacing,
            );
        }
    }

    return end;
}
