/**
 * Player orders — the only thing that crosses the network in a lockstep game.
 *
 * An order is issued on the tick the player clicked, but *executes* a fixed
 * number of ticks later so every client has received it in time. Within an
 * execution tick, orders are sorted by (player, seq) before they are applied,
 * so the order in which packets happen to arrive cannot change the outcome.
 * That canonical sort is what makes the sim independent of the network.
 */

export const OrderType = {
    Spawn: 0,
    Move: 1,
    Stop: 2,
    Damage: 3,
} as const;

export type OrderTypeValue = (typeof OrderType)[keyof typeof OrderType];

/**
 * Payload slots are plain numbers so an order is trivially hashable, loggable
 * and serialisable. `a`-`d` are interpreted per order type:
 *
 * - `Spawn`: a = unit kind, b = x, c = y
 * - `Move`:  a = entity handle, b = x, c = y
 * - `Stop`:  a = entity handle
 * - `Damage`: a = entity handle, b = amount
 */
export interface Order {
    /** Tick on which this executes. Identical on every client. */
    readonly tick: number;
    readonly player: number;
    /** Per-player monotonic counter. Breaks ties within a tick. */
    readonly seq: number;
    /**
     * Orders issued by one player action share a group id; 0 means ungrouped.
     *
     * A real lockstep RTS sends one command carrying a unit list. This flat
     * model approximates that with one order per unit and a shared id, which
     * keeps an order a fixed-size record while still letting the simulation see
     * that forty units were selected together.
     */
    readonly group: number;
    readonly type: OrderTypeValue;
    readonly a: number;
    readonly b: number;
    readonly c: number;
    readonly d: number;
}

export function makeOrder(
    tick: number,
    player: number,
    seq: number,
    type: OrderTypeValue,
    a = 0,
    b = 0,
    c = 0,
    d = 0,
    group = 0,
): Order {
    return {tick, player, seq, group, type, a, b, c, d};
}

/** Total order over orders inside one execution tick. */
export function compareOrders(x: Order, y: Order): number {
    if (x.player !== y.player) return x.player - y.player;
    if (x.seq !== y.seq) return x.seq - y.seq;
    return x.type - y.type;
}

/**
 * Buckets orders by execution tick.
 *
 * Nothing here depends on arrival order: `take` sorts before returning, so a
 * client that receives a tick's orders out of order still executes them in the
 * same sequence as everyone else.
 */
export class OrderQueue {
    private readonly buckets = new Map<number, Order[]>();

    schedule(order: Order): void {
        const bucket = this.buckets.get(order.tick);
        if (bucket === undefined) {
            this.buckets.set(order.tick, [order]);
        } else {
            bucket.push(order);
        }
    }

    scheduleAll(orders: readonly Order[]): void {
        for (const order of orders) this.schedule(order);
    }

    /** Returns this tick's orders in canonical order and drops the bucket. */
    take(tick: number): readonly Order[] {
        const bucket = this.buckets.get(tick);
        if (bucket === undefined) return EMPTY;
        this.buckets.delete(tick);
        bucket.sort(compareOrders);
        return bucket;
    }

    get pendingTicks(): number {
        return this.buckets.size;
    }
}

const EMPTY: readonly Order[] = [];
