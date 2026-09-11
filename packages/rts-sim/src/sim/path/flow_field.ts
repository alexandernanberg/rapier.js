import type {Hasher} from "../../core/hash";
import {TieBrokenHeap} from "../grid/heap";
import {CARDINAL_COST, DIAGONAL_COST, type TileMap} from "../grid/tile_map";

/** Integrated cost of a tile no route reaches. */
export const UNREACHABLE = 0x7fffffff;

/** Same fixed compass order A* expands in: N, NE, E, SE, S, SW, W, NW. */
const NEIGHBOUR_DX = [0, 1, 1, 1, 0, -1, -1, -1];
const NEIGHBOUR_DY = [-1, -1, 0, 1, 1, 1, 0, -1];
const NEIGHBOUR_DIAGONAL = [false, true, false, true, false, true, false, true];

/**
 * Cost-to-goal for every tile on the map, plus the best next step from each.
 *
 * This is the answer to the measurement that made flat A* untenable: a corner-
 * to-corner search costs ~2ms on a 128x128 map and serves exactly one unit,
 * while one Dijkstra from the goal costs a few times that and serves *every*
 * unit heading there. For the dominant RTS case — box an army, right-click —
 * that turns thousands of searches into one.
 *
 * It also removes two problems waypoint routes have: there is no path length to
 * truncate, so no re-planning mid-journey, and a unit shoved off its route by
 * separation simply reads the field at wherever it now stands.
 *
 * Built by Dijkstra *from* the goal, so `cost[t]` is the price of travelling
 * from `t` to the goal rather than the other way round.
 */
export class FlowField {
    /** Tile the field flows toward. -1 when never built. */
    goal = -1;
    /** Terrain revision this was built against; a mismatch means it is stale. */
    terrainRevision = -1;
    /** Cheapest cost from each tile to the goal. */
    readonly cost: Int32Array;
    /** Next tile on the way to the goal. -1 unreachable; the goal points at itself. */
    readonly next: Int32Array;

    private readonly closed: Uint8Array;
    /** Tiles finalised by the last build. Diagnostics and budgeting. */
    expanded = 0;

    constructor(tileCount: number) {
        this.cost = new Int32Array(tileCount);
        this.next = new Int32Array(tileCount);
        this.closed = new Uint8Array(tileCount);
    }

    build(map: TileMap, goal: number, heap: TieBrokenHeap): void {
        const {cost, next, closed} = this;

        cost.fill(UNREACHABLE);
        next.fill(-1);
        closed.fill(0);
        heap.clear();

        this.goal = goal;
        this.terrainRevision = map.revision;
        this.expanded = 0;

        if (!map.isPassableIndex(goal)) return;

        cost[goal] = 0;
        next[goal] = goal;
        heap.push(goal, 0, 0);

        while (heap.length > 0) {
            const current = heap.pop();
            if (closed[current] === 1) continue;
            closed[current] = 1;
            this.expanded++;

            const cx = map.tileX(current);
            const cy = map.tileY(current);
            const currentCost = cost[current];
            // Stepping *into* `current` costs its weight, matching A*'s
            // convention of charging for the tile being entered.
            const enterCurrent = map.weight[current];

            for (let n = 0; n < 8; n++) {
                const nx = cx + NEIGHBOUR_DX[n];
                const ny = cy + NEIGHBOUR_DY[n];
                if (!map.isPassable(nx, ny)) continue;

                // Same corner rule as A*, and symmetric, so a field and a
                // search never disagree about whether a gap is passable.
                if (NEIGHBOUR_DIAGONAL[n] && (!map.isPassable(cx, ny) || !map.isPassable(nx, cy))) {
                    continue;
                }

                const neighbour = map.index(nx, ny);
                if (closed[neighbour] === 1) continue;

                const step = NEIGHBOUR_DIAGONAL[n] ? DIAGONAL_COST : CARDINAL_COST;
                const candidate = currentCost + step * enterCurrent;
                if (candidate >= cost[neighbour]) continue;

                cost[neighbour] = candidate;
                next[neighbour] = current;
                heap.push(neighbour, candidate, 0);
            }
        }
    }

    isStale(map: TileMap): boolean {
        return this.goal === -1 || this.terrainRevision !== map.revision;
    }

    reaches(tile: number): boolean {
        return tile >= 0 && tile < this.cost.length && this.cost[tile] !== UNREACHABLE;
    }
}

/**
 * Fixed-size cache of flow fields, keyed by goal tile.
 *
 * Eviction is first-in-first-out rather than least-recently-used, because a
 * read would otherwise mutate cache order — and movement reads a field for
 * every unit, every tick. FIFO keeps the hot path free of state changes at the
 * cost of occasionally dropping a field still in use, which is self-healing:
 * the units holding it re-request and the field is rebuilt.
 *
 * Cache membership decides whether a unit gets a field or an A* search, so it
 * affects the simulation and is hashed. It is derived state — a pure function
 * of terrain and goal — so only the key set needs hashing, not the fields.
 */
export class FlowFieldCache {
    private readonly map: TileMap;
    private readonly capacity: number;
    private readonly heap: TieBrokenHeap;
    private readonly fields = new Map<number, FlowField>();
    /** Evicted instances, reused so a rebuild allocates nothing. */
    private readonly pool: FlowField[] = [];

    /** Fields built on the most recent tick. Diagnostics only. */
    lastBuilt = 0;
    /** Tiles expanded building them. Diagnostics only. */
    lastExpanded = 0;

    constructor(map: TileMap, capacity: number) {
        this.map = map;
        this.capacity = Math.max(1, capacity);
        this.heap = new TieBrokenHeap(map.tileCount * 2);
    }

    /** Reads a usable field without touching cache order. */
    peek(goal: number): FlowField | undefined {
        const field = this.fields.get(goal);
        if (field === undefined) return undefined;
        if (field.isStale(this.map)) {
            // Terrain moved under it — a building went up, a tree fell.
            this.fields.delete(goal);
            this.pool.push(field);
            return undefined;
        }
        return field;
    }

    has(goal: number): boolean {
        return this.peek(goal) !== undefined;
    }

    /** Builds a field for `goal`, evicting the oldest if the cache is full. */
    build(goal: number): FlowField {
        const existing = this.peek(goal);
        if (existing !== undefined) return existing;

        let field: FlowField;
        if (this.fields.size >= this.capacity) {
            const oldest = this.fields.keys().next().value as number;
            field = this.fields.get(oldest)!;
            this.fields.delete(oldest);
        } else {
            field = this.pool.pop() ?? new FlowField(this.map.tileCount);
        }

        field.build(this.map, goal, this.heap);
        this.fields.set(goal, field);

        this.lastBuilt++;
        this.lastExpanded += field.expanded;
        return field;
    }

    /** Called at the start of each tick's pathfinding pass. */
    resetCounters(): void {
        this.lastBuilt = 0;
        this.lastExpanded = 0;
    }

    get size(): number {
        return this.fields.size;
    }

    hashInto(hasher: Hasher): void {
        hasher.writeU32(this.fields.size);
        for (const goal of this.fields.keys()) {
            hasher.writeU32(goal >>> 0);
        }
    }
}
