import {TieBrokenHeap} from "./heap";
import {CARDINAL_COST, DIAGONAL_COST, type TileMap} from "./tile_map";

/**
 * Grid A* for a single unit's route.
 *
 * Ordering of the open set is `TieBrokenHeap`'s job: `f`, then `h`, then the
 * tile index. Neighbours are expanded in a fixed compass order for the same
 * reason, and costs are integers throughout (see `tile_map.ts`) so "cheaper" is
 * an exact comparison rather than a float one.
 *
 * For many units heading to one place this is the wrong tool — see
 * `FlowField`, which pays once for a destination instead of once per unit.
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

    private readonly heap: TieBrokenHeap;
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
        this.heap = new TieBrokenHeap(tiles * 2);
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
        this.heap.clear();

        const gen = this.generation;
        const {gScore, cameFrom, visitedStamp, closedStamp} = this;
        const goalX = map.tileX(goal);
        const goalY = map.tileY(goal);

        gScore[start] = 0;
        cameFrom[start] = -1;
        visitedStamp[start] = gen;
        this.heap.push(start, this.heuristic(map.tileX(start), map.tileY(start), goalX, goalY), 0);

        let expanded = 0;

        while (this.heap.length > 0) {
            const current = this.heap.pop();
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
                this.heap.push(neighbour, tentative + h, h);
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
}
