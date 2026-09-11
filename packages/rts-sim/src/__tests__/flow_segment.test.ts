import {describe, expect, it} from "vitest";
import {PortalGraph} from "../sim/grid/portal_graph";
import {IMPASSABLE, TileMap} from "../sim/grid/tile_map";
import {FlowSegment, FlowSegmentCache, UNREACHABLE} from "../sim/path/flow_segment";

const SECTOR = 8;

function setup(
    width = 48,
    height = 48,
): {map: TileMap; graph: PortalGraph; cache: FlowSegmentCache} {
    const map = new TileMap({width, height});
    const graph = new PortalGraph(map, SECTOR);
    graph.build();
    const cache = new FlowSegmentCache(map, graph, 16);
    return {map, graph, cache};
}

/** Walks the flow from a position, as movement does, and returns distance travelled. */
function walkFlow(
    map: TileMap,
    segment: FlowSegment,
    startX: number,
    startY: number,
    stepSize = 0.25,
): {travelled: number; reached: boolean} {
    const dir = new Float64Array(2);
    let x = startX;
    let y = startY;
    let travelled = 0;
    const targetX = map.tileX(segment.target) + 0.5;
    const targetY = map.tileY(segment.target) + 0.5;

    for (let i = 0; i < 4000; i++) {
        const dx = targetX - x;
        const dy = targetY - y;
        if (Math.sqrt(dx * dx + dy * dy) <= stepSize) return {travelled, reached: true};

        const tx = Math.floor(x);
        const ty = Math.floor(y);

        // Inside the target cell, steer at the target point itself — the same
        // final approach `movementSystem` does, since a tile centre is not
        // where the order was given.
        let vx: number;
        let vy: number;
        if (tx === map.tileX(segment.target) && ty === map.tileY(segment.target)) {
            vx = dx;
            vy = dy;
        } else {
            if (!segment.hasFlow(tx, ty)) return {travelled, reached: false};
            segment.directionAt(tx, ty, dir);
            vx = dir[0];
            vy = dir[1];
        }

        const length = Math.sqrt(vx * vx + vy * vy);
        if (length === 0) return {travelled, reached: false};

        x += (vx / length) * stepSize;
        y += (vy / length) * stepSize;
        travelled += stepSize;
    }
    return {travelled, reached: false};
}

describe("FlowSegment", () => {
    it("covers the 3x3 block of sectors around the one it serves", () => {
        const {map, graph, cache} = setup();
        const segment = cache.build(graph.layout.sectorOfTile(20, 20), map.index(22, 22));

        // Serving sector (2,2) spans tiles 16..23; the window adds a sector
        // each way.
        expect([segment.x0, segment.y0, segment.x1, segment.y1]).toEqual([8, 8, 32, 32]);
    });

    it("clips its window at the map edge", () => {
        const {map, graph, cache} = setup();
        const sector = graph.layout.sectorOfTile(2, 2);
        const segment = cache.build(sector, map.index(5, 5));

        expect([segment.x0, segment.y0]).toEqual([0, 0]);
        expect(segment.x1).toBe(16);
    });

    /**
     * The headline property. On open ground a line-of-sight cell must point
     * *exactly* at the target, not at one of eight neighbours — that is what
     * removes the dogleg a plain grid flow field walks.
     */
    it("points line-of-sight cells exactly at the target", () => {
        const {map, graph, cache} = setup();
        const goal = map.index(20, 20);
        const sector = graph.layout.sectorOfTile(20, 20);
        const segment = cache.build(sector, goal);
        const dir = new Float64Array(2);

        expect(segment.target).toBe(goal);

        for (const [x, y] of [
            [16, 16],
            [17, 23],
            [23, 17],
            [22, 21],
        ] as [number, number][]) {
            expect(segment.isLineOfSight(x, y), `${x},${y}`).toBe(true);
            segment.directionAt(x, y, dir);

            const dx = 20 - x;
            const dy = 20 - y;
            const length = Math.sqrt(dx * dx + dy * dy);
            expect(dir[0], `${x},${y} x`).toBeCloseTo(dx / length, 6);
            expect(dir[1], `${x},${y} y`).toBeCloseTo(dy / length, 6);
        }
    });

    it("walks a straight line on open ground, with no excess distance", () => {
        const {map, graph, cache} = setup();
        const goal = map.index(23, 20);
        const segment = cache.build(graph.layout.sectorOfTile(17, 17), goal);

        // A diagonal-ish run is where eight-direction flow loses the most.
        const result = walkFlow(map, segment, 17.5, 17.5);
        const straight = Math.sqrt((23.5 - 17.5) ** 2 + (20.5 - 17.5) ** 2);

        expect(result.reached).toBe(true);
        expect(result.travelled).toBeLessThan(straight * 1.02);
    });

    it("still gives flow to cells in shadow, from Dijkstra", () => {
        const {map, graph, cache} = setup();
        // A wall between the sector's left half and the goal.
        map.fillRect(20, 12, 1, 12, IMPASSABLE);
        graph.build();

        const goal = map.index(23, 20);
        const segment = cache.build(graph.layout.sectorOfTile(17, 20), goal);

        expect(segment.isLineOfSight(17, 20)).toBe(false);
        expect(segment.hasFlow(17, 20)).toBe(true);
        expect(segment.costAt(17, 20)).not.toBe(UNREACHABLE);

        const result = walkFlow(map, segment, 17.5, 20.5);
        expect(result.reached).toBe(true);
    });

    it("refuses line-of-sight across expensive ground, where a straight-line cost would lie", () => {
        const {map, graph, cache} = setup();
        map.fillRect(19, 16, 1, 8, 5);
        graph.build();

        const segment = cache.build(graph.layout.sectorOfTile(17, 20), map.index(23, 20));

        expect(segment.isLineOfSight(17, 20)).toBe(false);
        expect(segment.hasFlow(17, 20)).toBe(true);
    });

    it("aims at the furthest route entry inside the window when the goal is far", () => {
        const {map, graph, cache} = setup();
        const goal = map.index(45, 45);
        const sector = graph.layout.sectorOfTile(2, 2);
        const segment = cache.build(sector, goal);

        expect(segment.target).not.toBe(goal);
        expect(segment.target).toBeGreaterThanOrEqual(0);
        // Upstream as far as the window reaches, not the nearest boundary.
        expect(segment.contains(map.tileX(segment.target), map.tileY(segment.target))).toBe(true);
        expect(map.tileX(segment.target) + map.tileY(segment.target)).toBeGreaterThan(16);
    });

    it("produces identical segments on repeated builds", () => {
        const {map, graph} = setup();
        const a = new FlowSegmentCache(map, graph, 4);
        const b = new FlowSegmentCache(map, graph, 4);
        const sector = graph.layout.sectorOfTile(20, 20);
        const goal = map.index(45, 45);

        const one = a.build(sector, goal);
        const two = b.build(sector, goal);
        const d1 = new Float64Array(2);
        const d2 = new Float64Array(2);

        expect(two.target).toBe(one.target);
        expect(two.expanded).toBe(one.expanded);
        expect(two.losCells).toBe(one.losCells);
        for (let y = one.y0; y < one.y1; y++) {
            for (let x = one.x0; x < one.x1; x++) {
                expect(two.costAt(x, y), `${x},${y}`).toBe(one.costAt(x, y));
                if (!one.hasFlow(x, y)) continue;
                one.directionAt(x, y, d1);
                two.directionAt(x, y, d2);
                expect(Array.from(d2), `${x},${y}`).toEqual(Array.from(d1));
            }
        }
    });

    it("integrates a window, not the map", () => {
        const {map, graph, cache} = setup(64, 64);
        const segment = cache.build(graph.layout.sectorOfTile(32, 32), map.index(60, 60));

        // 3x8 = 24 cells a side, so at most 576 — against 4096 for the map.
        expect(segment.expanded).toBeLessThanOrEqual(24 * 24);
        expect(map.tileCount).toBe(4096);
    });
});

describe("FlowSegmentCache", () => {
    it("shares one segment between every unit in a sector heading the same way", () => {
        const {map, graph, cache} = setup();
        const sector = graph.layout.sectorOfTile(20, 20);
        const goal = map.index(45, 45);

        const first = cache.build(sector, goal);
        expect(cache.build(sector, goal)).toBe(first);
        expect(cache.lastBuilt).toBe(1);
    });

    it("keys on the sector, so a later order through the same ground reuses it", () => {
        const {map, graph, cache} = setup();
        const goal = map.index(45, 45);
        const a = graph.layout.sectorOfTile(20, 20);
        const b = graph.layout.sectorOfTile(28, 28);

        cache.build(a, goal);
        cache.build(b, goal);
        expect(cache.size).toBe(2);
        expect(cache.has(a, goal)).toBe(true);
        expect(cache.has(b, goal)).toBe(true);
    });

    it("drops segments whose terrain moved under them", () => {
        const {map, graph, cache} = setup();
        const sector = graph.layout.sectorOfTile(20, 20);
        const goal = map.index(22, 22);
        cache.build(sector, goal);
        expect(cache.has(sector, goal)).toBe(true);

        map.setWeight(3, 3, IMPASSABLE);
        graph.build();

        expect(cache.peek(sector, goal)).toBeUndefined();
        expect(cache.size).toBe(0);
    });

    it("evicts in first-in order when full", () => {
        const {map, graph} = setup();
        const cache = new FlowSegmentCache(map, graph, 2);
        const goal = map.index(45, 45);
        const sectors = [
            graph.layout.sectorOfTile(4, 4),
            graph.layout.sectorOfTile(12, 12),
            graph.layout.sectorOfTile(20, 20),
        ];

        for (const sector of sectors) cache.build(sector, goal);

        expect(cache.size).toBe(2);
        expect(cache.has(sectors[0], goal)).toBe(false);
        expect(cache.has(sectors[2], goal)).toBe(true);
    });
});
