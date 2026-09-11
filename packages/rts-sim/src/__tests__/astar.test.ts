import {describe, expect, it} from "vitest";
import {AStar} from "../sim/grid/astar";
import {CARDINAL_COST, DIAGONAL_COST, IMPASSABLE, TileMap} from "../sim/grid/tile_map";

function open(size = 16): TileMap {
    return new TileMap({width: size, height: size});
}

/** Tile indices a search returns, as (x, y) pairs for readable assertions. */
function coords(map: TileMap, out: Int32Array, length: number): [number, number][] {
    return Array.from({length}, (_, i) => [map.tileX(out[i]), map.tileY(out[i])]);
}

describe("AStar", () => {
    it("walks straight across open ground", () => {
        const map = open();
        const astar = new AStar(map);
        const out = new Int32Array(64);

        const result = astar.search(map.index(2, 2), map.index(6, 2), out);

        expect(result.length).toBe(4);
        expect(coords(map, out, result.length)).toEqual([
            [3, 2],
            [4, 2],
            [5, 2],
            [6, 2],
        ]);
    });

    it("excludes the start tile and ends on the goal", () => {
        const map = open();
        const astar = new AStar(map);
        const out = new Int32Array(64);

        const result = astar.search(map.index(1, 1), map.index(4, 4), out);

        expect(out[0]).not.toBe(map.index(1, 1));
        expect(out[result.length - 1]).toBe(map.index(4, 4));
    });

    it("prefers diagonals, since they cost less than two cardinals", () => {
        const map = open();
        const astar = new AStar(map);
        const out = new Int32Array(64);

        // 4 diagonals (56) beats 4 east + 4 north (80).
        expect(DIAGONAL_COST).toBeLessThan(2 * CARDINAL_COST);
        const result = astar.search(map.index(2, 2), map.index(6, 6), out);

        expect(result.length).toBe(4);
    });

    it("routes around a wall", () => {
        const map = open();
        map.fillRect(8, 0, 1, 12, IMPASSABLE);
        const astar = new AStar(map);
        const out = new Int32Array(128);

        const result = astar.search(map.index(4, 2), map.index(12, 2), out);

        expect(result.length).toBeGreaterThan(8);
        const path = coords(map, out, result.length);
        // Every crossing of the wall column must happen past its open end.
        for (const [x, y] of path) {
            if (x === 8) expect(y).toBeGreaterThanOrEqual(12);
        }
        expect(path.at(-1)).toEqual([12, 2]);
    });

    it("reports no route when the goal is walled in", () => {
        const map = open();
        map.fillRect(10, 4, 3, 1, IMPASSABLE);
        map.fillRect(10, 6, 3, 1, IMPASSABLE);
        map.fillRect(10, 4, 1, 3, IMPASSABLE);
        map.fillRect(12, 4, 1, 3, IMPASSABLE);
        const astar = new AStar(map);
        const out = new Int32Array(256);

        const result = astar.search(map.index(2, 2), map.index(11, 5), out);

        expect(result.length).toBe(0);
        expect(result.expanded).toBeGreaterThan(0);
    });

    it("refuses to cut the corner between two blocked tiles", () => {
        const map = open();
        map.setWeight(3, 2, IMPASSABLE);
        map.setWeight(2, 3, IMPASSABLE);
        const astar = new AStar(map);
        const out = new Int32Array(128);

        const result = astar.search(map.index(2, 2), map.index(3, 3), out);

        // (2,2) -> (3,3) is diagonal, but both orthogonal neighbours are blocked,
        // so the only way through is the long way round.
        expect(result.length).toBeGreaterThan(1);
    });

    it("refuses an impassable start or goal", () => {
        const map = open();
        map.setWeight(5, 5, IMPASSABLE);
        const astar = new AStar(map);
        const out = new Int32Array(64);

        expect(astar.search(map.index(5, 5), map.index(2, 2), out).length).toBe(0);
        expect(astar.search(map.index(2, 2), map.index(5, 5), out).length).toBe(0);
    });

    it("prefers cheap tiles over a shorter route across expensive ones", () => {
        const map = open();
        // A costly band the direct route would otherwise cross.
        map.fillRect(4, 0, 1, 6, 9);
        const astar = new AStar(map);
        const out = new Int32Array(128);

        const result = astar.search(map.index(2, 2), map.index(6, 2), out);
        const path = coords(map, out, result.length);

        // Cheaper to go around the band's end than to pay 9x for a tile.
        const crossings = path.filter(([x]) => x === 4);
        expect(crossings.every(([, y]) => y >= 6)).toBe(true);
    });

    it("truncates a route that outgrows the buffer and says so", () => {
        const map = open(32);
        const astar = new AStar(map);
        const out = new Int32Array(5);

        const result = astar.search(map.index(0, 0), map.index(31, 0), out);

        expect(result.length).toBe(5);
        expect(result.partial).toBe(true);
        expect(coords(map, out, 5)).toEqual([
            [1, 0],
            [2, 0],
            [3, 0],
            [4, 0],
            [5, 0],
        ]);
    });

    it("returns the same route for the same search, run twice", () => {
        const map = open(24);
        map.fillRect(10, 4, 1, 14, IMPASSABLE);
        const astar = new AStar(map);
        const a = new Int32Array(256);
        const b = new Int32Array(256);

        const first = astar.search(map.index(2, 10), map.index(20, 10), a);
        const second = astar.search(map.index(2, 10), map.index(20, 10), b);

        expect(second.length).toBe(first.length);
        expect(Array.from(b)).toEqual(Array.from(a));
    });

    /**
     * On symmetric open ground a huge number of routes tie on cost, which is
     * exactly where an under-specified comparator picks differently on each
     * client. Two instances that have done different work beforehand must still
     * agree — this also covers the generation-stamp reuse leaking state.
     */
    it("agrees between instances with different search histories", () => {
        const map = open(24);
        const fresh = new AStar(map);
        const used = new AStar(map);

        const warmup = new Int32Array(256);
        used.search(map.index(0, 0), map.index(23, 23), warmup);
        used.search(map.index(23, 0), map.index(0, 23), warmup);
        used.search(map.index(5, 9), map.index(18, 3), warmup);

        const a = new Int32Array(256);
        const b = new Int32Array(256);
        const start = map.index(3, 3);
        const goal = map.index(20, 20);

        const one = fresh.search(start, goal, a);
        const two = used.search(start, goal, b);

        expect(two.length).toBe(one.length);
        expect(two.expanded).toBe(one.expanded);
        expect(Array.from(b)).toEqual(Array.from(a));
    });

    it("finds no route to a tile outside the map", () => {
        const map = open();
        const astar = new AStar(map);
        const out = new Int32Array(64);

        expect(astar.search(map.index(2, 2), -1, out).length).toBe(0);
        expect(astar.search(map.index(2, 2), map.tileCount + 5, out).length).toBe(0);
    });
});

describe("TileMap", () => {
    it("round-trips world positions through tile indices", () => {
        const map = new TileMap({width: 16, height: 16, tileSize: 2});

        expect(map.worldToIndex(5, 7)).toBe(map.index(2, 3));
        expect(map.centerX(map.index(2, 3))).toBe(5);
        expect(map.centerY(map.index(2, 3))).toBe(7);
    });

    it("reports positions off the map rather than clamping them", () => {
        const map = open();

        expect(map.worldToIndex(-1, 5)).toBe(-1);
        expect(map.worldToIndex(5, 999)).toBe(-1);
    });

    it("changes its terrain hash only when terrain changes", () => {
        const map = open();
        const before = map.terrainHash();

        map.setWeight(3, 3, 1); // already NORMAL
        expect(map.terrainHash()).toBe(before);

        map.setWeight(3, 3, IMPASSABLE);
        expect(map.terrainHash()).not.toBe(before);
    });
});
