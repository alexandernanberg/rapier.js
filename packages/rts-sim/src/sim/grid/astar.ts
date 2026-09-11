import {CARDINAL_COST, DIAGONAL_COST, type TileMap} from "./tile_map";

/**
 * Grid A* with a total ordering on the open set.
 *
 * The determinism-critical detail is the comparator. Ranking only by `f` leaves
 * ties to be broken by heap layout, which depends on insertion history — two
 * clients then pick different but equally cheap routes and desync a minute
 * later. Ties here fall through to `h` and then to the tile index, which is
 * unique, so the pop order is total and the search is reproducible.
 *
 * Neighbours are expanded in a fixed compass order for the same reason, and
 * costs are integers throughout (see `tile_map.ts`) so "cheaper" is an exact
 * comparison rather than a float one.
 */

/** Fixed expansion order: N, NE, E, SE, S, SW, W, NW. */
const NEIGHBOUR_DX = [0, 1, 1, 1, 0, -1, -1, -1];
const NEIGHBOUR_DY = [-1, -1, 0, 1, 1, 1, 0, -1];
const NEIGHBOUR_DIAGONAL = [false, true, false, true, false, true, false, true];

export interface SearchResult {
    /** Waypoints written to the output buffer. 0 means no route. */
    readonly length: number;
    /** True when the route was longer than the buffer and got truncated. */
    readonly partial: boolean;
    /** Tiles popped from the open set — the cost knob for budgeting. */
    readonly expanded: number;
}

const NO_PATH: SearchResult = {length: 0, partial: false, expanded: 0};

export class AStar {
    private readonly map: TileMap;
    private readonly gScore: Int32Array;
    private readonly cameFrom: Int32Array;
    /**
     * Generation stamps instead of clearing the score arrays between searches.
     * Clearing is O(tiles) per request, which at a few hundred requests a
     * second dominates the searches themselves.
     */
    private readonly visitedStamp: Int32Array;
    private readonly closedStamp: Int32Array;
    private generation = 0;

    private readonly heapTile: Int32Array;
    private readonly heapF: Int32Array;
    private readonly heapH: Int32Array;
    private heapSize = 0;

    private readonly scratch: Int32Array;

    constructor(map: TileMap) {
        this.map = map;
        const tiles = map.tileCount;
        this.gScore = new Int32Array(tiles);
        this.cameFrom = new Int32Array(tiles);
        this.visitedStamp = new Int32Array(tiles);
        this.closedStamp = new Int32Array(tiles);
        // A tile can sit in the open set more than once (we never decrease-key,
        // we push again and skip the stale pop), so leave room.
        this.heapTile = new Int32Array(tiles * 2);
        this.heapF = new Int32Array(tiles * 2);
        this.heapH = new Int32Array(tiles * 2);
        this.scratch = new Int32Array(tiles);
    }

    /**
     * Searches from `start` to `goal`, writing waypoint tile indices into `out`.
     *
     * The start tile is not included — waypoints are places to move *to*. When
     * the route is longer than `out`, its first `out.length` steps are written
     * and `partial` is set; the caller re-plans on reaching the end.
     */
    search(start: number, goal: number, out: Int32Array): SearchResult {
        const map = this.map;
        if (start === goal) return NO_PATH;
        if (!map.isPassableIndex(start) || !map.isPassableIndex(goal)) return NO_PATH;

        this.generation++;
        this.heapSize = 0;

        const gen = this.generation;
        const {gScore, cameFrom, visitedStamp, closedStamp} = this;
        const goalX = map.tileX(goal);
        const goalY = map.tileY(goal);

        gScore[start] = 0;
        cameFrom[start] = -1;
        visitedStamp[start] = gen;
        this.push(start, this.heuristic(map.tileX(start), map.tileY(start), goalX, goalY), 0);

        let expanded = 0;

        while (this.heapSize > 0) {
            const current = this.pop();
            if (closedStamp[current] === gen) continue;
            closedStamp[current] = gen;
            expanded++;

            if (current === goal) return this.reconstruct(start, goal, out, expanded);

            const cx = map.tileX(current);
            const cy = map.tileY(current);
            const currentG = gScore[current];

            for (let n = 0; n < 8; n++) {
                const nx = cx + NEIGHBOUR_DX[n];
                const ny = cy + NEIGHBOUR_DY[n];
                if (!map.isPassable(nx, ny)) continue;

                // Refuse to squeeze diagonally past two blocked tiles; a unit
                // would visually clip the corner of a building.
                if (NEIGHBOUR_DIAGONAL[n] && (!map.isPassable(cx, ny) || !map.isPassable(nx, cy))) {
                    continue;
                }

                const neighbour = map.index(nx, ny);
                if (closedStamp[neighbour] === gen) continue;

                const step = NEIGHBOUR_DIAGONAL[n] ? DIAGONAL_COST : CARDINAL_COST;
                const tentative = currentG + step * map.weight[neighbour];

                if (visitedStamp[neighbour] === gen && tentative >= gScore[neighbour]) continue;

                visitedStamp[neighbour] = gen;
                gScore[neighbour] = tentative;
                cameFrom[neighbour] = current;

                const h = this.heuristic(nx, ny, goalX, goalY);
                this.push(neighbour, tentative + h, h);
            }
        }

        return {length: 0, partial: false, expanded};
    }

    /**
     * Octile distance — the exact cost of an unobstructed diagonal-plus-straight
     * run over weight-1 tiles, so it never overestimates and A* stays optimal.
     */
    private heuristic(x: number, y: number, goalX: number, goalY: number): number {
        const dx = Math.abs(x - goalX);
        const dy = Math.abs(y - goalY);
        const diagonal = dx < dy ? dx : dy;
        return CARDINAL_COST * (dx + dy) + (DIAGONAL_COST - 2 * CARDINAL_COST) * diagonal;
    }

    private reconstruct(
        start: number,
        goal: number,
        out: Int32Array,
        expanded: number,
    ): SearchResult {
        const {cameFrom, scratch} = this;

        let total = 0;
        for (let tile = goal; tile !== start; tile = cameFrom[tile]) {
            scratch[total++] = tile;
        }

        const capacity = out.length;
        const length = total < capacity ? total : capacity;

        // `scratch` holds the route goal-first; walk it backwards so the buffer
        // comes out start-first, keeping only as much as fits.
        for (let i = 0; i < length; i++) {
            out[i] = scratch[total - 1 - i];
        }

        return {length, partial: total > capacity, expanded};
    }

    private push(tile: number, f: number, h: number): void {
        const {heapTile, heapF, heapH} = this;
        let i = this.heapSize++;
        heapTile[i] = tile;
        heapF[i] = f;
        heapH[i] = h;

        while (i > 0) {
            const parent = (i - 1) >> 1;
            if (!this.less(i, parent)) break;
            this.swap(i, parent);
            i = parent;
        }
    }

    private pop(): number {
        const top = this.heapTile[0];
        const last = --this.heapSize;

        if (last > 0) {
            this.moveSlot(last, 0);
            let i = 0;
            for (;;) {
                const left = 2 * i + 1;
                if (left >= last) break;
                const right = left + 1;
                const child = right < last && this.less(right, left) ? right : left;
                if (!this.less(child, i)) break;
                this.swap(i, child);
                i = child;
            }
        }

        return top;
    }

    /**
     * Total order on the open set: cheapest `f`, then closest to the goal, then
     * lowest tile index. That last term is what makes the search deterministic
     * — without it, equal-cost nodes pop in heap-layout order.
     */
    private less(a: number, b: number): boolean {
        const {heapF, heapH, heapTile} = this;
        if (heapF[a] !== heapF[b]) return heapF[a] < heapF[b];
        if (heapH[a] !== heapH[b]) return heapH[a] < heapH[b];
        return heapTile[a] < heapTile[b];
    }

    private swap(a: number, b: number): void {
        const {heapTile, heapF, heapH} = this;
        const tile = heapTile[a];
        const f = heapF[a];
        const h = heapH[a];
        heapTile[a] = heapTile[b];
        heapF[a] = heapF[b];
        heapH[a] = heapH[b];
        heapTile[b] = tile;
        heapF[b] = f;
        heapH[b] = h;
    }

    private moveSlot(from: number, to: number): void {
        const {heapTile, heapF, heapH} = this;
        heapTile[to] = heapTile[from];
        heapF[to] = heapF[from];
        heapH[to] = heapH[from];
    }
}
