import {describe, expect, it} from "vitest";
import {AStar} from "../sim/grid/astar";
import {PortalGraph} from "../sim/grid/portal_graph";
import {IMPASSABLE, NORMAL, TileMap} from "../sim/grid/tile_map";

function graph(map: TileMap, sectorSize = 8): PortalGraph {
    const g = new PortalGraph(map, sectorSize);
    g.build();
    return g;
}

describe("PortalGraph", () => {
    it("finds one portal per sector boundary on open ground", () => {
        const map = new TileMap({width: 16, height: 16});
        const g = graph(map, 8);

        // 2x2 sectors: one vertical boundary pair and one horizontal pair, each
        // a single open run, and each run makes two nodes.
        expect(g.layout.cols).toBe(2);
        expect(g.layout.rows).toBe(2);
        expect(g.nodes).toBe(8);
    });

    it("splits a boundary into two portals when a wall blocks its middle", () => {
        const open = new TileMap({width: 16, height: 8});
        const walled = new TileMap({width: 16, height: 8});
        // Block the middle of the single vertical boundary at x = 7|8.
        walled.fillRect(7, 3, 2, 2, IMPASSABLE);

        expect(graph(walled, 8).nodes).toBe(graph(open, 8).nodes + 2);
    });

    it("returns just the goal for a destination in the same sector", () => {
        const map = new TileMap({width: 16, height: 16});
        const g = graph(map, 8);
        const out = new Int32Array(64);

        const goal = map.index(5, 5);
        expect(g.route(map.index(2, 2), goal, out)).toBe(1);
        expect(out[0]).toBe(goal);
    });

    it("crosses sectors in order, ending at the goal", () => {
        const map = new TileMap({width: 32, height: 32});
        const g = graph(map, 8);
        const out = new Int32Array(64);

        const start = map.index(2, 2);
        const goal = map.index(29, 29);
        const count = g.route(start, goal, out);

        expect(count).toBeGreaterThan(1);
        expect(out[count - 1]).toBe(goal);

        // Each emitted cell must be in a different sector from the last, and
        // adjacent to it — the route is a walk over neighbouring sectors.
        let previous = g.layout.sectorOfTile(map.tileX(start), map.tileY(start));
        for (let i = 0; i < count; i++) {
            const sector = g.layout.sectorOfTile(map.tileX(out[i]), map.tileY(out[i]));
            if (i < count - 1) expect(sector).not.toBe(previous);
            const dx = Math.abs(g.layout.sectorX(sector) - g.layout.sectorX(previous));
            const dy = Math.abs(g.layout.sectorY(sector) - g.layout.sectorY(previous));
            expect(dx + dy, `step ${i}`).toBeLessThanOrEqual(1);
            previous = sector;
        }
    });

    it("routes through the only gap in a long wall", () => {
        const map = new TileMap({width: 32, height: 32});
        map.fillRect(16, 0, 1, 28, IMPASSABLE);
        const g = graph(map, 8);
        const out = new Int32Array(64);

        const count = g.route(map.index(4, 4), map.index(28, 4), out);
        expect(count).toBeGreaterThan(0);

        // Every emitted cell on the wall's column must be past its open end.
        for (let i = 0; i < count; i++) {
            if (map.tileX(out[i]) === 16) expect(map.tileY(out[i])).toBeGreaterThanOrEqual(28);
        }
    });

    it("reports no route to a sealed pocket", () => {
        const map = new TileMap({width: 32, height: 32});
        map.fillRect(20, 20, 5, 1, IMPASSABLE);
        map.fillRect(20, 24, 5, 1, IMPASSABLE);
        map.fillRect(20, 20, 1, 5, IMPASSABLE);
        map.fillRect(24, 20, 1, 5, IMPASSABLE);
        const g = graph(map, 8);
        const out = new Int32Array(64);

        expect(g.route(map.index(2, 2), map.index(22, 22), out)).toBe(0);
    });

    it("refuses impassable endpoints", () => {
        const map = new TileMap({width: 32, height: 32});
        map.setWeight(5, 5, IMPASSABLE);
        const g = graph(map, 8);
        const out = new Int32Array(64);

        expect(g.route(map.index(5, 5), map.index(20, 20), out)).toBe(0);
        expect(g.route(map.index(20, 20), map.index(5, 5), out)).toBe(0);
    });

    /**
     * The cross-check that matters: the abstract graph must agree with a full
     * grid search about what is reachable. A portal graph that misses a
     * connection strands units with no route where one plainly exists, and
     * nothing else in the stack would notice.
     */
    it("agrees with grid A* on reachability, over a maze", () => {
        const map = new TileMap({width: 48, height: 48});
        for (let i = 0; i < 10; i++) {
            // Staggered walls with alternating gaps.
            const x = 4 + i * 4;
            if (i % 2 === 0) map.fillRect(x, 0, 1, 40, IMPASSABLE);
            else map.fillRect(x, 8, 1, 40, IMPASSABLE);
        }
        map.fillRect(30, 30, 6, 6, IMPASSABLE);

        const g = graph(map, 8);
        const astar = new AStar(map);
        const gridOut = new Int32Array(map.tileCount);
        const graphOut = new Int32Array(256);

        let checked = 0;
        for (let i = 0; i < 400; i++) {
            const start = (i * 7919) % map.tileCount;
            const goal = (i * 6271 + 13) % map.tileCount;
            if (!map.isPassableIndex(start) || !map.isPassableIndex(goal)) continue;
            if (start === goal) continue;
            checked++;

            const gridReaches = astar.search(start, goal, gridOut).length > 0;
            const graphReaches = g.route(start, goal, graphOut) > 0;
            expect(graphReaches, `start ${start} goal ${goal}`).toBe(gridReaches);
        }

        expect(checked).toBeGreaterThan(200);
    });

    it("rebuilds when terrain changes", () => {
        const map = new TileMap({width: 16, height: 16});
        const g = graph(map, 8);
        const before = g.nodes;

        expect(g.isStale).toBe(false);
        map.fillRect(7, 0, 2, 8, IMPASSABLE);
        expect(g.isStale).toBe(true);

        g.ensureFresh();
        expect(g.isStale).toBe(false);
        // That wall sealed the whole left/right boundary of the top sectors.
        expect(g.nodes).toBeLessThan(before);
    });
});

describe("PortalGraph incremental rebuild", () => {
    /** Routes for a fixed set of pairs, as the graph's observable behaviour. */
    function routeFingerprint(g: PortalGraph, map: TileMap): string {
        const out = new Int32Array(256);
        const axis = new Uint8Array(256);
        const parts: string[] = [];

        for (let i = 0; i < 300; i++) {
            const start = (i * 7919) % map.tileCount;
            const goal = (i * 6271 + 13) % map.tileCount;
            if (!map.isPassableIndex(start) || !map.isPassableIndex(goal)) continue;
            const count = g.route(start, goal, out, axis);
            parts.push(
                `${start}>${goal}:${count}:${Array.from(out.subarray(0, count)).join(",")}` +
                    `:${Array.from(axis.subarray(0, count)).join(",")}`,
            );
        }
        return parts.join("|");
    }

    function mazeMap(): TileMap {
        const map = new TileMap({width: 48, height: 48});
        for (let i = 0; i < 8; i++) {
            const x = 4 + i * 5;
            if (i % 2 === 0) map.fillRect(x, 0, 1, 36, IMPASSABLE);
            else map.fillRect(x, 10, 1, 38, IMPASSABLE);
        }
        return map;
    }

    /**
     * The test that makes incremental rebuild trustworthy. An incremental graph
     * that differs from a full one sends units on different routes depending on
     * the order buildings happened to be placed — a desync in multiplayer and a
     * mystery in single player.
     */
    it("matches a full rebuild after a building goes up", () => {
        const incremental = mazeMap();
        const full = mazeMap();
        const a = graph(incremental, 8);
        const b = graph(full, 8);
        expect(routeFingerprint(a, incremental)).toBe(routeFingerprint(b, full));

        // Same change to both: one rebuilt incrementally, one from scratch.
        for (const map of [incremental, full]) map.fillRect(20, 20, 3, 3, IMPASSABLE);
        a.ensureFresh();
        b.build();

        expect(a.nodes).toBe(b.nodes);
        expect(routeFingerprint(a, incremental)).toBe(routeFingerprint(b, full));
    });

    it("matches a full rebuild after a wall is demolished", () => {
        const incremental = mazeMap();
        const full = mazeMap();
        const a = graph(incremental, 8);
        const b = graph(full, 8);

        // Punch a hole through one of the long walls.
        for (const map of [incremental, full]) map.fillRect(9, 14, 1, 6, NORMAL);
        a.ensureFresh();
        b.build();

        expect(a.nodes).toBe(b.nodes);
        expect(routeFingerprint(a, incremental)).toBe(routeFingerprint(b, full));
    });

    it("matches a full rebuild after many scattered changes", () => {
        const incremental = mazeMap();
        const full = mazeMap();
        const a = graph(incremental, 8);
        const b = graph(full, 8);

        // Changes landing in separate sectors on separate updates, which is
        // where a dirty set that under-approximates would show up.
        const edits: [number, number, number][] = [
            [6, 6, IMPASSABLE],
            [30, 2, IMPASSABLE],
            [15, 40, IMPASSABLE],
            [44, 44, IMPASSABLE],
            [9, 25, NORMAL],
            [24, 33, IMPASSABLE],
        ];
        for (const [x, y, weight] of edits) {
            for (const map of [incremental, full]) map.fillRect(x, y, 2, 2, weight);
            a.ensureFresh();
        }
        b.build();

        expect(a.nodes).toBe(b.nodes);
        expect(routeFingerprint(a, incremental)).toBe(routeFingerprint(b, full));
    });

    it("matches a full rebuild when a change seals a sector boundary", () => {
        const incremental = new TileMap({width: 32, height: 32});
        const full = new TileMap({width: 32, height: 32});
        const a = graph(incremental, 8);
        const b = graph(full, 8);

        // A whole boundary closed off, which removes nodes on both sides.
        for (const map of [incremental, full]) map.fillRect(8, 8, 1, 8, IMPASSABLE);
        a.ensureFresh();
        b.build();

        expect(a.nodes).toBe(b.nodes);
        expect(routeFingerprint(a, incremental)).toBe(routeFingerprint(b, full));
    });

    it("still agrees with grid A* on reachability after changes", () => {
        const map = mazeMap();
        const g = graph(map, 8);
        map.fillRect(20, 20, 4, 4, IMPASSABLE);
        map.fillRect(9, 14, 1, 6, NORMAL);
        g.ensureFresh();

        const astar = new AStar(map);
        const gridOut = new Int32Array(map.tileCount);
        const graphOut = new Int32Array(256);

        let checked = 0;
        for (let i = 0; i < 400; i++) {
            const start = (i * 7919) % map.tileCount;
            const goal = (i * 6271 + 13) % map.tileCount;
            if (!map.isPassableIndex(start) || !map.isPassableIndex(goal)) continue;
            if (start === goal) continue;
            checked++;
            expect(g.route(start, goal, graphOut) > 0, `start ${start} goal ${goal}`).toBe(
                astar.search(start, goal, gridOut).length > 0,
            );
        }
        expect(checked).toBeGreaterThan(200);
    });

    it("does nothing when terrain is set to the value it already had", () => {
        const map = mazeMap();
        const g = graph(map, 8);
        const before = g.nodes;

        map.setWeight(3, 3, NORMAL);
        expect(g.isStale).toBe(false);
        expect(g.nodes).toBe(before);
    });
});
