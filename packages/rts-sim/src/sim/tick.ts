import {SYSTEMS} from "./systems";
import {applyOrders, flushCommands, type SimWorld} from "./world";

/**
 * Advances the simulation by exactly one fixed timestep.
 *
 * The shape of a tick is itself part of the determinism contract — reordering
 * these phases changes results and invalidates every recorded replay:
 *
 *   1. take this tick's orders, in canonical (player, seq) order
 *   2. flush, so an order's structural effects are visible to this tick's systems
 *   3. run systems in their declared order; they queue, never mutate structure
 *   4. flush again, so system-queued changes land at a single known point
 *
 * There is deliberately no wall-clock, no `performance.now`, and no variable
 * timestep anywhere in here. `dt` is a constant from config.
 */
export function step(world: SimWorld): void {
    applyOrders(world, world.orders.take(world.tick));
    flushCommands(world);

    for (let i = 0; i < SYSTEMS.length; i++) {
        SYSTEMS[i](world);
    }
    flushCommands(world);

    world.tick++;
}

/** Runs `count` ticks. */
export function run(world: SimWorld, count: number): void {
    for (let i = 0; i < count; i++) step(world);
}
