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
import {atan2, length} from "../core/math";
import {Rng} from "../core/rand";
import {CommandBuffer, CommandKind, type Command} from "./command_buffer";
import {createStores, PathState, UNIT_STATS, type Stores} from "./components";
import {layoutSlots, MAX_FORMATION} from "./formation";
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
    /**
     * World units between formation slots.
     *
     * Must clear twice the largest unit radius with slack to spare, or
     * separation jitter cannot resolve and a few units never reach their places.
     * At a radius of 0.4, spacing 1.0 left six of two hundred units stuck three
     * tiles short; 1.1 settled every size tested. The default carries margin.
     */
    readonly formationSpacing: number;
    /**
     * Fraction of its slowest member's speed that a formation's leader travels
     * at.
     *
     * Must be below 1. A leader moving at exactly member speed cannot be caught
     * up with: a unit knocked off its slot by separation or a corner closes the
     * gap at zero, so it lags for the rest of the march and the formation
     * steadily comes apart. The shortfall is the headroom members need to
     * re-form, and it is why a formation moves slower than a lone unit.
     */
    readonly formationLeaderSpeed: number;
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
    formationSpacing: 1.2,
    formationLeaderSpeed: 0.9,
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
    /** Slot offsets for one formation, as x,y pairs in formation-local space. */
    readonly slots: Float64Array;
    /** Sort keys while ordering a formation's members. */
    readonly projection: Float64Array;
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
            slots: new Float64Array(MAX_FORMATION * 2),
            projection: new Float64Array(MAX_FORMATION),
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
            // A new destination leaves any formation the unit was in; a grouped
            // order re-joins it immediately after.
            leaveFormation(world, id);
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
            if (!entityExists(world, command.b)) break;
            const id = idOf(world, command.a);
            stores.Formation.leader[id] = command.b;
            stores.Formation.localX[id] = command.c;
            stores.Formation.localY[id] = command.d;
            break;
        }
    }
}

/** Drops a unit out of whatever formation it was in. */
function leaveFormation(world: SimWorld, id: number): void {
    const {Formation} = world.stores;
    Formation.leader[id] = -1;
    Formation.localX[id] = 0;
    Formation.localY[id] = 0;
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
    stores.Formation.leader[id] = -1;
    stores.Formation.localX[id] = 0;
    stores.Formation.localY[id] = 0;
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
 * Every member keeps the *same* destination — so the group still shares one flow
 * segment per sector — and gets a slot relative to a virtual leader instead.
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

    const capacity = Math.min(world.scratch.formation.length, MAX_FORMATION);
    const total = end - start;
    const count = Math.min(total, capacity);
    const members = world.scratch.formation.subarray(0, count);
    for (let i = 0; i < count; i++) members[i] = orders[start + i].a;

    // A selection larger than a formation can hold still gets its order; the
    // overflow simply marches as a crowd. Dropping it would lose the order
    // outright, which is far worse than an untidy tail.
    for (let i = count; i < total; i++) {
        world.cmd.setMoveTarget(orders[start + i].a, orders[start].b, orders[start].c);
    }

    createFormation(world, members, count, orders[start].b, orders[start].c, orders[start].d);
    return end;
}

/**
 * Orders members so the ones already at the front of the group take the front
 * slots.
 *
 * Assigning slots by handle instead leaves units having to walk *through* the
 * formation to reach a slot on its far side, and the units already standing
 * there block them — a 200-strong block left eleven units stuck eight tiles
 * short of their places. Sorting both by position along the march axis means
 * almost nobody has to cross.
 *
 * Slots come out of `layoutSlots` rear-first (local +x is forward and rows step
 * along it), so members are sorted rear-first to match. Ties break on handle, so
 * the result never depends on selection order.
 */
function orderMembersForSlots(
    world: SimWorld,
    members: Int32Array,
    count: number,
    originX: number,
    originY: number,
    goalX: number,
    goalY: number,
): void {
    const {Position} = world.stores;
    const dx = goalX - originX;
    const dy = goalY - originY;
    const distance = length(dx, dy);

    if (distance === 0) {
        members.sort((a, b) => a - b);
        return;
    }

    const dirX = dx / distance;
    const dirY = dy / distance;
    const projection = world.scratch.projection;

    for (let i = 0; i < count; i++) {
        const eid = members[i];
        if (!entityExists(world, eid)) {
            projection[i] = Number.POSITIVE_INFINITY;
            continue;
        }
        const id = idOf(world, eid);
        projection[i] = (Position.x[id] - originX) * dirX + (Position.y[id] - originY) * dirY;
    }

    // Insertion sort over (projection, handle): the list is short, already
    // roughly ordered, and this keeps the comparison explicit.
    for (let i = 1; i < count; i++) {
        const eid = members[i];
        const key = projection[i];
        let j = i - 1;
        while (j >= 0 && (projection[j] > key || (projection[j] === key && members[j] > eid))) {
            members[j + 1] = members[j];
            projection[j + 1] = projection[j];
            j--;
        }
        members[j + 1] = eid;
        projection[j + 1] = key;
    }
}

/**
 * Spawns a virtual leader for a group and hands each member a slot behind it.
 *
 * Created here rather than through the command buffer because the members need
 * the leader's handle immediately. That is safe: order application runs before
 * any system, and is followed straight away by a flush. The no-structural-change
 * rule exists to stop mutation *during* a query iteration, which this is not.
 *
 * The leader is a real entity with Position, MoveTarget, Path, Speed and Facing,
 * so the existing schedule paths and moves it with no special cases. It
 * deliberately has no Radius — separation ignores it — and no Health, so the
 * death system does too.
 */
export function createFormation(
    world: SimWorld,
    members: Int32Array,
    count: number,
    goalX: number,
    goalY: number,
    shape: number,
): EntityId | -1 {
    const {stores} = world;
    if (count === 0) return -1;

    // Start the leader where the group already is, so nobody has to catch up,
    // and hold it to the slowest member so nobody is left behind.
    let sumX = 0;
    let sumY = 0;
    let live = 0;
    let speed = Number.POSITIVE_INFINITY;
    for (let i = 0; i < count; i++) {
        const eid = members[i];
        if (!entityExists(world, eid)) continue;
        const id = idOf(world, eid);
        sumX += stores.Position.x[id];
        sumY += stores.Position.y[id];
        if (stores.Speed.value[id] < speed) speed = stores.Speed.value[id];
        live++;
    }
    if (live === 0) return -1;

    const leader = addEntity(world);
    const leaderId = idOf(world, leader);

    addComponent(world, leader, stores.Position);
    addComponent(world, leader, stores.Velocity);
    addComponent(world, leader, stores.Facing);
    addComponent(world, leader, stores.Speed);
    addComponent(world, leader, stores.Path);
    addComponent(world, leader, stores.Formation);
    addComponent(world, leader, stores.FormationLeader);
    addComponent(world, leader, stores.MoveTarget);

    const originX = sumX / live;
    const originY = sumY / live;
    stores.Position.x[leaderId] = originX;
    stores.Position.y[leaderId] = originY;
    stores.Velocity.x[leaderId] = 0;
    stores.Velocity.y[leaderId] = 0;
    stores.Speed.value[leaderId] = speed * world.config.formationLeaderSpeed;
    stores.MoveTarget.x[leaderId] = goalX;
    stores.MoveTarget.y[leaderId] = goalY;
    // Face the destination from the off, so slots are oriented sensibly on the
    // very first tick rather than snapping round on the second.
    stores.Facing.angle[leaderId] = atan2(goalY - originY, goalX - originX);
    resetPath(world, leaderId);
    leaveFormation(world, leaderId);

    stores.FormationLeader.shape[leaderId] = shape;
    stores.FormationLeader.spacing[leaderId] = world.config.formationSpacing;
    stores.FormationLeader.memberCount[leaderId] = live;

    const slots = world.scratch.slots;
    layoutSlots(shape, count, world.config.formationSpacing, slots);
    orderMembersForSlots(world, members, count, originX, originY, goalX, goalY);

    for (let i = 0; i < count; i++) {
        const eid = members[i];
        if (!entityExists(world, eid)) continue;

        // Order matters: `setMoveTarget` drops any existing membership, so the
        // join has to be queued behind it rather than written here.
        world.cmd.setMoveTarget(eid, goalX, goalY);
        world.cmd.setFormationSlot(eid, leader, slots[i * 2], slots[i * 2 + 1]);
    }

    return leader;
}
