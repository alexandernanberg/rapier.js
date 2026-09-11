import {query} from "bitecs";
import {atan2, length} from "../core/math";
import {MAX_PATH, PathState} from "./components";
import {idOf, type SimWorld} from "./world";

/** Distance at which a unit is considered to have reached its order position. */
const ARRIVE_EPSILON = 1e-6;
/** Distance at which a unit is considered to have reached a waypoint. */
const WAYPOINT_EPSILON = 0.05;
/** Fraction of an overlap each of the two units resolves. */
const SEPARATION_SHARE = 0.5;
/** Below this squared distance two units count as exactly coincident. */
const COINCIDENT_EPSILON_SQ = 1e-12;

/**
 * Queues a route for anything that wants to move and has no plan.
 *
 * Nothing is searched here — this only enqueues, so the cost of wanting to move
 * is constant and the searching stays inside the tick's budget.
 */
export function pathRequestSystem(world: SimWorld): void {
    const {stores, map, paths} = world;
    const {Position, MoveTarget, Path} = stores;

    const entities = query(world, [Position, MoveTarget, Path]);

    for (let i = 0; i < entities.length; i++) {
        const eid = entities[i];
        const id = idOf(world, eid);
        if (Path.state[id] !== PathState.None) continue;

        const goal = map.worldToIndex(MoveTarget.x[id], MoveTarget.y[id]);
        if (goal === -1) {
            // Ordered off the map. Refuse once rather than re-asking forever.
            Path.state[id] = PathState.Failed;
            continue;
        }

        Path.goal[id] = goal;
        Path.state[id] = PathState.Pending;
        paths.request(eid, goal);
    }
}

/** Spends this tick's pathfinding budget. */
export function pathServiceSystem(world: SimWorld): void {
    world.paths.service(world, world.config.pathBudget);
}

/**
 * Moves units along their route, or straight at the order position when they
 * have no usable route.
 *
 * Steering straight while a request is still queued is deliberate: a unit that
 * froze for the few ticks until the pathfinder reached it would read as input
 * lag. It walks hopefully in the right direction and corrects once the route
 * lands.
 */
export function movementSystem(world: SimWorld): void {
    const {stores, cmd, map} = world;
    const {Position, Velocity, MoveTarget, Speed, Facing, Path} = stores;
    const dt = world.config.dt;

    const entities = query(world, [Position, Velocity, MoveTarget, Speed, Facing, Path]);

    for (let i = 0; i < entities.length; i++) {
        const eid = entities[i];
        const id = idOf(world, eid);

        const px = Position.x[id];
        const py = Position.y[id];

        // Retire waypoints already reached, so a fast unit can cross several in
        // one tick rather than steering at a point behind it.
        if (Path.state[id] === PathState.Active) {
            const base = id * MAX_PATH;
            while (Path.cursor[id] < Path.length[id]) {
                const tile = Path.tiles[base + Path.cursor[id]];
                const wx = map.centerX(tile);
                const wy = map.centerY(tile);
                if (length(wx - px, wy - py) > WAYPOINT_EPSILON) break;
                Path.cursor[id]++;
            }

            // Out of waypoints but not at the goal tile: the route was
            // truncated, so ask for the next leg.
            if (Path.cursor[id] >= Path.length[id] && Path.length[id] > 0) {
                const last = Path.tiles[base + Path.length[id] - 1];
                if (last !== Path.goal[id]) {
                    Path.state[id] = PathState.None;
                    Path.length[id] = 0;
                    Path.cursor[id] = 0;
                }
            }
        }

        // Steer at the next waypoint when there is one, otherwise at the order
        // position itself — which is also the final approach inside the goal
        // tile, since a tile centre is not where the player clicked.
        let targetX = MoveTarget.x[id];
        let targetY = MoveTarget.y[id];
        if (Path.state[id] === PathState.Active && Path.cursor[id] < Path.length[id]) {
            const tile = Path.tiles[id * MAX_PATH + Path.cursor[id]];
            targetX = map.centerX(tile);
            targetY = map.centerY(tile);
        }

        const dx = targetX - px;
        const dy = targetY - py;
        const dist = length(dx, dy);

        if (dist <= ARRIVE_EPSILON) {
            cmd.clearMoveTarget(eid);
            continue;
        }

        Facing.angle[id] = atan2(dy, dx);

        const step = Speed.value[id] * dt;
        if (step >= dist) {
            // Snap rather than overshoot, so a unit's resting position is a
            // function of its order alone and not of the tick it arrived on.
            Position.x[id] = targetX;
            Position.y[id] = targetY;
            Velocity.x[id] = 0;
            Velocity.y[id] = 0;
            if (targetX === MoveTarget.x[id] && targetY === MoveTarget.y[id]) {
                cmd.clearMoveTarget(eid);
            }
            continue;
        }

        const inv = 1 / dist;
        const vx = dx * inv * Speed.value[id];
        const vy = dy * inv * Speed.value[id];

        Velocity.x[id] = vx;
        Velocity.y[id] = vy;
        Position.x[id] = px + vx * dt;
        Position.y[id] = py + vy * dt;
    }
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
        Position.x[id] += pushX[id];
        Position.y[id] += pushY[id];
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
 * be pathed this tick if the budget allows. Separation runs after movement so it
 * corrects the positions movement just wrote.
 */
export const SYSTEMS: readonly ((world: SimWorld) => void)[] = [
    pathRequestSystem,
    pathServiceSystem,
    movementSystem,
    separationSystem,
    deathSystem,
];
