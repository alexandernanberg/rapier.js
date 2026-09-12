import {entityExists, hasComponent, Not, query} from "bitecs";
import {atan2, cos, length, sin} from "../core/math";
import {PathState} from "./components";
import {hasLineOfSight} from "./path/flow_segment";
import {idOf, type SimWorld} from "./world";

/** Distance at which a unit is considered to have reached its order position. */
const ARRIVE_EPSILON = 1e-6;
/** Fraction of an overlap each of the two units resolves. */
const SEPARATION_SHARE = 0.5;
/** Below this squared distance two units count as exactly coincident. */
const COINCIDENT_EPSILON_SQ = 1e-12;

/**
 * Writes a position, refusing to put a unit inside terrain.
 *
 * Pathfinding cannot be trusted to do this on its own, and it is not supposed
 * to: separation pushes units with no idea where the walls are, and a crowd
 * squeezed against a building will shove its neighbours straight through it. A
 * stress run of 60 units herded through a walled map logged 1281 such
 * violations before this existed.
 *
 * Axes resolve one at a time so a blocked diagonal still slides along the wall
 * instead of stopping dead.
 *
 * Only the unit's centre is tested, so a radius can still visually overlap a
 * wall by a fraction of a tile. Fixing that properly means obstacle-avoidance
 * steering, not a bigger clamp.
 */
function moveClamped(
    world: SimWorld,
    id: number,
    fromX: number,
    fromY: number,
    toX: number,
    toY: number,
): void {
    const {Position} = world.stores;
    const map = world.map;

    const x = map.isPassableIndex(map.worldToIndex(toX, fromY)) ? toX : fromX;
    const y = map.isPassableIndex(map.worldToIndex(x, toY)) ? toY : fromY;

    Position.x[id] = x;
    Position.y[id] = y;
}

/**
 * Queues a flow segment for anything that wants to move and lacks a current one.
 *
 * Nothing is integrated here — this only enqueues, so wanting to move costs a
 * constant, and the integration stays inside the tick's budget.
 *
 * A unit already following a segment from a sector it has since left is
 * re-queued for an upgrade without losing what it has: a segment's window
 * covers the neighbouring sectors too, so it keeps steering correctly while it
 * waits rather than stalling at every sector boundary.
 */
export function pathRequestSystem(world: SimWorld): void {
    const {stores, map, paths} = world;
    const {Position, MoveTarget, Path} = stores;
    const layout = paths.graph.layout;

    const entities = query(world, [Position, MoveTarget, Path]);

    for (let i = 0; i < entities.length; i++) {
        const eid = entities[i];
        const id = idOf(world, eid);
        if (Path.state[id] === PathState.Failed) continue;

        const goal = map.worldToIndex(MoveTarget.x[id], MoveTarget.y[id]);
        if (goal === -1) {
            // Ordered off the map. Refuse once rather than re-asking forever.
            Path.state[id] = PathState.Failed;
            Path.sector[id] = -1;
            continue;
        }

        if (Path.goal[id] !== goal) {
            Path.goal[id] = goal;
            Path.sector[id] = -1;
            Path.state[id] = PathState.None;
        }

        const tx = map.worldToTileX(Position.x[id]);
        const ty = map.worldToTileY(Position.y[id]);
        if (!map.inBounds(tx, ty)) continue;

        const sector = layout.sectorOfTile(tx, ty);
        if (Path.state[id] === PathState.Flow && Path.sector[id] === sector) continue;

        // Deduplicated by entity, so re-queueing an upgrade every tick is free.
        paths.request(eid, goal);
    }
}

/** Spends this tick's segment budget. */
export function pathServiceSystem(world: SimWorld): void {
    world.paths.service(world);
}

/**
 * Moves the virtual leaders of formations.
 *
 * Separate from, and ahead of, `movementSystem` so a member always reads its
 * leader's position *after* the leader has advanced this tick. Leaving that to
 * whatever order the query returned would be deterministic but arbitrary, and
 * would make the formation trail a tick behind at some headings and not others.
 */
export function leaderMovementSystem(world: SimWorld): void {
    const {Position, Velocity, MoveTarget, Speed, Facing, Path, FormationLeader} = world.stores;
    const entities = query(world, [
        Position,
        Velocity,
        MoveTarget,
        Speed,
        Facing,
        Path,
        FormationLeader,
    ]);

    for (let i = 0; i < entities.length; i++) {
        moveEntity(world, entities[i]);
    }
}

/**
 * Moves everything that is not a formation leader: members steering at their
 * slots, and lone units steering at their order position.
 */
export function movementSystem(world: SimWorld): void {
    const {Position, Velocity, MoveTarget, Speed, Facing, Path, Formation, FormationLeader} =
        world.stores;
    const entities = query(world, [
        Position,
        Velocity,
        MoveTarget,
        Speed,
        Facing,
        Path,
        Formation,
        Not(FormationLeader),
    ]);

    for (let i = 0; i < entities.length; i++) {
        moveEntity(world, entities[i]);
    }
}

/**
 * Advances one entity toward whatever it should be steering at.
 *
 * Shared by both movement systems so there is one integration step, one arrival
 * rule and one terrain clamp rather than two that can drift apart.
 *
 * An order is cleared only on arriving at the order position itself. A formation
 * member keeps its order for as long as it belongs to the formation, because
 * that is what holds it on station — whether a *group* has finished is asked of
 * its leader, which drops its own order on arrival like anything else.
 */
function moveEntity(world: SimWorld, eid: number): void {
    const {stores, cmd} = world;
    const {Position, Velocity, MoveTarget, Speed, Facing} = stores;
    const dt = world.config.dt;
    const id = idOf(world, eid);

    const px = Position.x[id];
    const py = Position.y[id];

    const steer = nextSteeringPoint(world, id, px, py);
    const towardOrder = steer === TOWARD_ORDER;
    const targetX = towardOrder ? MoveTarget.x[id] : world.scratch.steer[0];
    const targetY = towardOrder ? MoveTarget.y[id] : world.scratch.steer[1];

    const dx = targetX - px;
    const dy = targetY - py;
    const dist = length(dx, dy);

    if (dist <= ARRIVE_EPSILON) {
        if (towardOrder) cmd.clearMoveTarget(eid);
        return;
    }

    Facing.angle[id] = atan2(dy, dx);

    const step = Speed.value[id] * dt;
    if (step >= dist) {
        // Snap rather than overshoot, so a unit's resting position is a
        // function of its order alone and not of the tick it arrived on.
        moveClamped(world, id, px, py, targetX, targetY);
        Velocity.x[id] = 0;
        Velocity.y[id] = 0;
        if (towardOrder) cmd.clearMoveTarget(eid);
        return;
    }

    const inv = 1 / dist;
    const vx = dx * inv * Speed.value[id];
    const vy = dy * inv * Speed.value[id];

    Velocity.x[id] = vx;
    Velocity.y[id] = vy;
    moveClamped(world, id, px, py, px + vx * dt, py + vy * dt);
}

/** `nextSteeringPoint` wrote a flow-derived steering point into `scratch.steer`. */
const TOWARD_WAYPOINT = 0;
/** No usable intermediate point; head straight at the order position. */
const TOWARD_ORDER = 1;
/** `scratch.steer` holds this unit's formation slot. */
const TOWARD_SLOT = 2;

/**
 * Picks the point a unit should steer at this tick, writing it into
 * `world.scratch.steer`, and maintains the unit's segment state as a side
 * effect (dropping a stale segment so it gets re-queued).
 *
 * Returns `TOWARD_ORDER` when the unit should head at its order position
 * directly — having no segment yet, a failed route, or the final approach once
 * it stands on the goal tile, since a tile centre is not where the player
 * clicked. The scratch out-param is per-world rather than module state, so two
 * simulations in one process cannot interfere.
 */
function nextSteeringPoint(world: SimWorld, id: number, px: number, py: number): number {
    const {stores, map, paths, scratch} = world;
    const {Path, Formation} = stores;

    const leader = Formation.leader[id];
    if (leader !== -1 && entityExists(world, leader)) {
        const leaderId = idOf(world, leader);
        // The slot is stored in formation-local space, where +x is the direction
        // of travel, so it rotates with the leader and one shape works at any
        // heading.
        const angle = stores.Facing.angle[leaderId];
        const c = cos(angle);
        const sn = sin(angle);
        const localX = Formation.localX[id];
        const localY = Formation.localY[id];
        const spotX = stores.Position.x[leaderId] + localX * c - localY * sn;
        const spotY = stores.Position.y[leaderId] + localX * sn + localY * c;

        // Once the leader has stopped the slot is static and the group is
        // already where it was going, so sight lines are settled: hold station
        // without paying for a raycast. This is also what keeps a standing
        // formation tidy — a unit nudged by a latecomer's separation gets pulled
        // back, where a unit that had dropped its order would stay shoved.
        if (!hasComponent(world, leader, stores.MoveTarget)) {
            scratch.steer[0] = spotX;
            scratch.steer[1] = spotY;
            return TOWARD_SLOT;
        }

        const fromX = map.worldToTileX(px);
        const fromY = map.worldToTileY(py);
        const toX = map.worldToTileX(spotX);
        const toY = map.worldToTileY(spotY);

        // While marching, follow the slot when it is in sight and the flow when
        // it is not — one rule covering both holding formation and squeezing
        // through a gap. The ray is bounded by the formation's own size, since a
        // slot is always near its leader.
        if (map.inBounds(toX, toY) && hasLineOfSight(map, fromX, fromY, toX, toY)) {
            scratch.steer[0] = spotX;
            scratch.steer[1] = spotY;
            return TOWARD_SLOT;
        }
    }

    if (Path.state[id] !== PathState.Flow) return TOWARD_ORDER;

    const segment = paths.segments.peek(Path.sector[id], Path.goal[id]);
    if (segment === undefined) {
        // Evicted, or terrain moved under it. Ask again.
        Path.state[id] = PathState.None;
        Path.sector[id] = -1;
        return TOWARD_ORDER;
    }

    const tx = map.worldToTileX(px);
    const ty = map.worldToTileY(py);

    if (tx === map.tileX(segment.target) && ty === map.tileY(segment.target)) {
        if (segment.target !== Path.goal[id]) {
            // Standing on an intermediate target: this segment has taken the
            // unit as far as it goes, so ask for the next one.
            Path.state[id] = PathState.None;
            Path.sector[id] = -1;
        }
        return TOWARD_ORDER;
    }

    if (!segment.hasFlow(tx, ty)) {
        Path.state[id] = PathState.None;
        Path.sector[id] = -1;
        return TOWARD_ORDER;
    }

    segment.directionAt(tx, ty, scratch.steer);
    // The segment gives a direction; movement wants a point. Project a tile
    // ahead along it — far enough that the arrival snap never triggers on it.
    scratch.steer[0] = px + scratch.steer[0] * map.tileSize;
    scratch.steer[1] = py + scratch.steer[1] * map.tileSize;
    return TOWARD_WAYPOINT;
}

/**
 * Pushes overlapping units apart.
 *
 * This is what stands in for physics: a symmetric circle push-apart on the
 * spatial grid. Each unit resolves half of each overlap, so a pair converges
 * without either being authoritative.
 *
 * Float addition is not associative, so the accumulation order matters. It is
 * the query's order, which is deterministic for clients that performed the same
 * operations — the same reason the checksum deliberately does *not* use query
 * order (see `snapshot.ts`).
 */
export function separationSystem(world: SimWorld): void {
    const {stores, grid, scratch} = world;
    const {Position, Radius} = stores;
    const {pushX, pushY, neighbours, rawIds} = scratch;

    const entities = query(world, [Position, Radius]);
    const count = entities.length;
    if (count < 2) return;

    for (let i = 0; i < count; i++) {
        const id = idOf(world, entities[i]);
        rawIds[i] = id;
        pushX[id] = 0;
        pushY[id] = 0;
    }

    grid.build(entities, rawIds, count, Position.x, Position.y);

    for (let i = 0; i < count; i++) {
        const selfId = rawIds[i];
        const sx = Position.x[selfId];
        const sy = Position.y[selfId];
        const selfR = Radius.value[selfId];

        // Reach far enough to catch anything that could overlap. Doubling the
        // radius covers a neighbour of equal size; a mixed-size roster wants
        // the largest radius on the map here.
        const found = grid.collect(sx, sy, selfR * 2, neighbours);

        for (let n = 0; n < found; n++) {
            const other = neighbours[n];
            if (other === entities[i]) continue;

            const otherId = idOf(world, other);
            const dx = sx - Position.x[otherId];
            const dy = sy - Position.y[otherId];
            const d2 = dx * dx + dy * dy;
            const minDist = selfR + Radius.value[otherId];

            if (d2 >= minDist * minDist) continue;

            if (d2 < COINCIDENT_EPSILON_SQ) {
                // Exactly coincident: no normal exists, so pick one by id so
                // both units agree and neither divides by zero.
                const sign = selfId < otherId ? -1 : 1;
                pushX[selfId] += sign * minDist * SEPARATION_SHARE;
                continue;
            }

            const d = Math.sqrt(d2);
            const overlap = (minDist - d) * SEPARATION_SHARE;
            const inv = 1 / d;
            pushX[selfId] += dx * inv * overlap;
            pushY[selfId] += dy * inv * overlap;
        }
    }

    for (let i = 0; i < count; i++) {
        const id = rawIds[i];
        if (pushX[id] === 0 && pushY[id] === 0) continue;
        const px = Position.x[id];
        const py = Position.y[id];
        moveClamped(world, id, px, py, px + pushX[id], py + pushY[id]);
    }
}

/**
 * Keeps formations consistent: drops members whose leader is gone, recounts each
 * leader's members, and retires leaders nobody follows.
 *
 * A leader is an entity like any other, so it needs something to end its life;
 * without this a formation's leader would outlive its last member and keep
 * marching an empty slot grid across the map.
 */
export function formationUpkeepSystem(world: SimWorld): void {
    const {stores, cmd} = world;
    const {Formation, FormationLeader} = stores;

    const leaders = query(world, [FormationLeader]);
    for (let i = 0; i < leaders.length; i++) {
        FormationLeader.memberCount[idOf(world, leaders[i])] = 0;
    }

    const members = query(world, [Formation, Not(FormationLeader)]);
    for (let i = 0; i < members.length; i++) {
        const id = idOf(world, members[i]);
        const leader = Formation.leader[id];
        if (leader === -1) continue;

        if (!entityExists(world, leader)) {
            Formation.leader[id] = -1;
            Formation.localX[id] = 0;
            Formation.localY[id] = 0;
            continue;
        }
        FormationLeader.memberCount[idOf(world, leader)]++;
    }

    for (let i = 0; i < leaders.length; i++) {
        const eid = leaders[i];
        if (FormationLeader.memberCount[idOf(world, eid)] === 0) cmd.despawn(eid);
    }
}

/** Queues removal of anything that has run out of health. */
export function deathSystem(world: SimWorld): void {
    const {stores, cmd} = world;
    const entities = query(world, [stores.Health]);

    for (let i = 0; i < entities.length; i++) {
        const eid = entities[i];
        if (stores.Health.current[idOf(world, eid)] <= 0) {
            cmd.despawn(eid);
        }
    }
}

/**
 * The tick's system schedule, as a literal list.
 *
 * Order is part of the simulation's contract: change it and every existing
 * replay stops matching. Keeping it as data rather than a sequence of calls
 * makes that explicit and reviewable.
 *
 * Requests are queued before they are serviced, so a unit ordered this tick can
 * be pathed this tick if the budget allows. Leaders move before their members,
 * so a slot is read at this tick's leader position rather than last tick's.
 * Separation runs after movement so it corrects the positions movement just
 * wrote, and upkeep runs after that so it sees the tick's final state.
 */
export const SYSTEMS: readonly ((world: SimWorld) => void)[] = [
    pathRequestSystem,
    pathServiceSystem,
    leaderMovementSystem,
    movementSystem,
    separationSystem,
    formationUpkeepSystem,
    deathSystem,
];
