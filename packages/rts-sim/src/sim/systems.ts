import {query} from "bitecs";
import {atan2, length} from "../core/math";
import {idOf, type SimWorld} from "./world";

/** Distance at which a unit is considered to have reached its target. */
const ARRIVE_EPSILON = 1e-6;

/**
 * Moves units toward their target and faces them along the path.
 *
 * Reads and writes only component fields; the arrival that removes MoveTarget
 * is queued on the command buffer, never applied inline, so the query being
 * iterated is not mutated underneath us.
 */
export function movementSystem(world: SimWorld): void {
    const {stores, cmd} = world;
    const {Position, Velocity, MoveTarget, Speed, Facing} = stores;
    const dt = world.config.dt;

    const entities = query(world, [Position, Velocity, MoveTarget, Speed, Facing]);

    for (let i = 0; i < entities.length; i++) {
        const eid = entities[i];
        const id = idOf(world, eid);

        const dx = MoveTarget.x[id] - Position.x[id];
        const dy = MoveTarget.y[id] - Position.y[id];
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
            Position.x[id] = MoveTarget.x[id];
            Position.y[id] = MoveTarget.y[id];
            Velocity.x[id] = 0;
            Velocity.y[id] = 0;
            cmd.clearMoveTarget(eid);
            continue;
        }

        const inv = 1 / dist;
        const vx = dx * inv * Speed.value[id];
        const vy = dy * inv * Speed.value[id];

        Velocity.x[id] = vx;
        Velocity.y[id] = vy;
        Position.x[id] += vx * dt;
        Position.y[id] += vy * dt;
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
 */
export const SYSTEMS: readonly ((world: SimWorld) => void)[] = [movementSystem, deathSystem];
