import {describe, expect, it} from "vitest";
import {AStar} from "../sim/grid/astar";
import {TieBrokenHeap} from "../sim/grid/heap";
import {CARDINAL_COST, DIAGONAL_COST, IMPASSABLE, TileMap} from "../sim/grid/tile_map";
import {FlowField, FlowFieldCache, UNREACHABLE} from "../sim/path/flow_field";

function build(map: TileMap, goal: number): FlowField {
    const field = new FlowField(map.tileCount);
    field.build(map, goal, new TieBrokenHeap(map.tileCount * 2));
    return field;
}

/** Cost of walking a waypoint list, by the same rule both planners use. */
function routeCost(map: TileMap, start: number, tiles: Int32Array, length: number): number {
    let total = 0;
    let prev = start;
    for (let i = 0; i < length; i++) {
        const tile = tiles[i];
        const diagonal = map.tileX(tile) !== map.tileX(prev) && map.tileY(tile) !== map.tileY(prev);
        total += (diagonal ? DIAGONAL_COST : CARDINAL_COST) * map.weight[tile];
        prev = tile;
    }
    return total;
}

/** Follows `next` from a tile and returns the tiles walked, goal included. */
function walk(field: FlowField, from: number, limit = 10000): number[] {
    const visited: number[] = [];
    let tile = from;
    while (tile !== field.goal) {
        const next = field.next[tile];
        if (next < 0 || visited.length > limit) break;
        visited.push(next);
        tile = next;
    }
    return visited;
}

describe("FlowField", () => {
    it("costs nothing to stand on the goal", () => {
        const map = new TileMap({width: 16, height: 16});
        const goal = map.index(8, 8);
        const field = build(map, goal);

        expect(field.cost[goal]).toBe(0);
        expect(field.next[goal]).toBe(goal);
    });

    it("gets more expensive the further out you start", () => {
        const map = new TileMap({width: 16, height: 16});
        const field = build(map, map.index(8, 8));

        expect(field.cost[map.index(7, 8)]).toBeLessThan(field.cost[map.index(4, 8)]);
        expect(field.cost[map.index(4, 8)]).toBeLessThan(field.cost[map.index(0, 8)]);
    });

    it("leads every reachable tile to the goal", () => {
        const map = new TileMap({width: 20, height: 20});
        map.fillRect(10, 0, 1, 15, IMPASSABLE);
        const goal = map.index(18, 3);
        const field = build(map, goal);

        for (let tile = 0; tile < map.tileCount; tile++) {
            if (!field.reaches(tile) || tile === goal) continue;
            const path = walk(field, tile);
            expect(path.at(-1), `from ${tile}`).toBe(goal);
        }
    });

    /**
     * The strongest check available: a flow field and an A* search must agree
     * on what a route costs. If they disagree, one of them is not optimal, or
     * they charge for tiles differently — a bug that would show up as units
     * taking visibly different routes depending on how they were planned.
     */
    it("agrees with A* on the cost of every route", () => {
        const map = new TileMap({width: 24, height: 24});
        map.fillRect(10, 0, 2, 18, IMPASSABLE);
        map.fillRect(16, 6, 2, 18, IMPASSABLE);
        map.fillRect(4, 12, 4, 2, 3);

        const goal = map.index(21, 2);
        const field = build(map, goal);
        const astar = new AStar(map);
        const out = new Int32Array(map.tileCount);

        const starts = [
            map.index(1, 1),
            map.index(2, 20),
            map.index(13, 22),
            map.index(6, 13),
            map.index(9, 9),
            map.index(19, 20),
        ];

        for (const start of starts) {
            const result = astar.search(start, goal, out);
            expect(result.partial, `start ${start}`).toBe(false);
            expect(result.length, `start ${start}`).toBeGreaterThan(0);
            expect(routeCost(map, start, out, result.length), `start ${start}`).toBe(
                field.cost[start],
            );
        }
    });

    it("marks walled-off tiles unreachable rather than guessing", () => {
        const map = new TileMap({width: 16, height: 16});
        // A sealed 1x1 pocket.
        map.fillRect(4, 4, 3, 1, IMPASSABLE);
        map.fillRect(4, 6, 3, 1, IMPASSABLE);
        map.setWeight(4, 5, IMPASSABLE);
        map.setWeight(6, 5, IMPASSABLE);
        const pocket = map.index(5, 5);

        const field = build(map, map.index(12, 12));

        expect(field.cost[pocket]).toBe(UNREACHABLE);
        expect(field.next[pocket]).toBe(-1);
        expect(field.reaches(pocket)).toBe(false);
    });

    it("reaches nothing when the goal itself is impassable", () => {
        const map = new TileMap({width: 16, height: 16});
        map.setWeight(8, 8, IMPASSABLE);

        const field = build(map, map.index(8, 8));

        expect(field.reaches(map.index(2, 2))).toBe(false);
        expect(field.expanded).toBe(0);
    });

    it("weighs a detour against the cost of crossing expensive ground", () => {
        const map = new TileMap({width: 16, height: 16});
        // Short band: going round it costs ~122, paying 9x for one tile ~160.
        map.fillRect(8, 0, 1, 6, 9);

        const field = build(map, map.index(12, 2));
        const path = walk(field, map.index(4, 2));

        for (const tile of path) {
            if (map.tileX(tile) === 8) expect(map.tileY(tile)).toBeGreaterThanOrEqual(6);
        }
    });

    it("crosses expensive ground when going round would cost more", () => {
        const map = new TileMap({width: 16, height: 16});
        // Tall band: the detour is now the expensive option, so a correct field
        // pays the toll rather than blindly avoiding weight.
        map.fillRect(8, 0, 1, 12, 9);

        const field = build(map, map.index(12, 2));
        const path = walk(field, map.index(4, 2));
        const crossing = path.find((tile) => map.tileX(tile) === 8);

        expect(crossing).toBeDefined();
        expect(map.tileY(crossing!)).toBeLessThan(12);
    });

    it("produces identical fields on repeated builds", () => {
        const map = new TileMap({width: 20, height: 20});
        map.fillRect(9, 0, 1, 14, IMPASSABLE);
        const goal = map.index(17, 4);

        const a = build(map, goal);
        const b = build(map, goal);

        expect(Array.from(b.cost)).toEqual(Array.from(a.cost));
        expect(Array.from(b.next)).toEqual(Array.from(a.next));
        expect(b.expanded).toBe(a.expanded);
    });

    it("goes stale when terrain changes", () => {
        const map = new TileMap({width: 16, height: 16});
        const field = build(map, map.index(8, 8));

        expect(field.isStale(map)).toBe(false);
        map.setWeight(3, 3, IMPASSABLE);
        expect(field.isStale(map)).toBe(true);
    });
});

describe("FlowFieldCache", () => {
    function map(): TileMap {
        return new TileMap({width: 16, height: 16});
    }

    it("returns the same instance for a repeated goal", () => {
        const m = map();
        const cache = new FlowFieldCache(m, 4);
        const goal = m.index(8, 8);

        const first = cache.build(goal);
        expect(cache.build(goal)).toBe(first);
        expect(cache.lastBuilt).toBe(1);
    });

    it("evicts in first-in order when full", () => {
        const m = map();
        const cache = new FlowFieldCache(m, 2);
        const a = m.index(1, 1);
        const b = m.index(2, 2);
        const c = m.index(3, 3);

        cache.build(a);
        cache.build(b);
        // Reading `a` must not save it — eviction is deliberately not LRU, so
        // that reads stay free of state changes.
        cache.peek(a);
        cache.build(c);

        expect(cache.size).toBe(2);
        expect(cache.has(a)).toBe(false);
        expect(cache.has(b)).toBe(true);
        expect(cache.has(c)).toBe(true);
    });

    it("drops a field whose terrain moved under it", () => {
        const m = map();
        const cache = new FlowFieldCache(m, 4);
        const goal = m.index(8, 8);
        cache.build(goal);
        expect(cache.has(goal)).toBe(true);

        m.setWeight(2, 2, IMPASSABLE);

        expect(cache.peek(goal)).toBeUndefined();
        expect(cache.size).toBe(0);
    });

    it("reuses evicted instances instead of reallocating", () => {
        const m = map();
        const cache = new FlowFieldCache(m, 1);

        const first = cache.build(m.index(1, 1));
        cache.build(m.index(2, 2));
        const third = cache.build(m.index(3, 3));

        // Capacity 1, so the third build must be the first field's instance
        // carried round the pool rather than a fresh allocation.
        expect(third).toBe(first);
        expect(third.goal).toBe(m.index(3, 3));
    });
});
