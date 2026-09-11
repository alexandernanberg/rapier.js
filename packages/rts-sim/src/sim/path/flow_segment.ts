import type {Hasher} from "../../core/hash";
import {TieBrokenHeap} from "../grid/heap";
import {AXIS_X, AXIS_Y, type PortalGraph} from "../grid/portal_graph";
import {CARDINAL_COST, DIAGONAL_COST, NORMAL, type TileMap} from "../grid/tile_map";

/** Integrated cost of a cell no route reaches. */
export const UNREACHABLE = 0x7fffffff;

const NEIGHBOUR_DX = [0, 1, 1, 1, 0, -1, -1, -1];
const NEIGHBOUR_DY = [-1, -1, 0, 1, 1, 1, 0, -1];
const NEIGHBOUR_DIAGONAL = [false, true, false, true, false, true, false, true];

/**
 * A flow field over a window of a few sectors, not the whole map.
 *
 * This is the shape Age of Empires IV describes as *overlapping segmented
 * flow*, and the reason for it is arithmetic: integrating a 1024x1024 grid is
 * "a lot of nodes" when you "only need flow on the way to the destination".
 * A segment covers the 3x3 block of sectors around the one it serves, so
 * neighbouring segments overlap by a full sector and the flow inside the served
 * sector accounts for geometry beyond it.
 *
 * Two passes build it:
 *
 * 1. **Line of sight.** Cells in the served sector with a clear straight run to
 *    the target get the exact direction to it. On open ground that is most of
 *    them, and it removes the direction quantisation that makes grid flow
 *    fields walk visible doglegs.
 * 2. **Dijkstra** from the target and from the LOS frontier, for everything in
 *    shadow. Those cells fall back to eight directions, which is acceptable
 *    because they are the cells hugging an obstacle.
 *
 * Segments are a pure function of (served sector, goal, terrain), so they are
 * cached and never hashed — only the cache's key set is, since which segment a
 * unit reads affects where it walks.
 */
export class FlowSegment {
    /** Sector this segment is valid for. -1 when never built. */
    sector = -1;
    /** Final destination this segment leads toward. */
    goal = -1;
    /** Cell actually integrated from — the furthest route entry inside the window. */
    target = -1;
    terrainRevision = -1;

    /** Window bounds in tile coordinates; `x1`/`y1` exclusive. */
    x0 = 0;
    y0 = 0;
    x1 = 0;
    y1 = 0;

    private windowWidth = 0;
    private cost: Int32Array;
    private dirX: Float32Array;
    private dirY: Float32Array;
    /** 1 where the direction came from a line-of-sight run, 0 from Dijkstra. */
    private los: Uint8Array;
    /** Cells the Dijkstra pass has finalised, so stale heap entries are skipped. */
    private closed: Uint8Array;

    /** Cells the Dijkstra pass finalised. Diagnostics only. */
    expanded = 0;
    /** Cells the LOS pass answered exactly. Diagnostics only. */
    losCells = 0;

    /** Boundary axis per route entry, filled alongside the route. */
    private readonly axisScratch = new Uint8Array(256);

    constructor(capacity: number) {
        this.cost = new Int32Array(capacity);
        this.dirX = new Float32Array(capacity);
        this.dirY = new Float32Array(capacity);
        this.los = new Uint8Array(capacity);
        this.closed = new Uint8Array(capacity);
    }

    private slot(tx: number, ty: number): number {
        return (ty - this.y0) * this.windowWidth + (tx - this.x0);
    }

    contains(tx: number, ty: number): boolean {
        return tx >= this.x0 && ty >= this.y0 && tx < this.x1 && ty < this.y1;
    }

    isStale(map: TileMap): boolean {
        return this.sector === -1 || this.terrainRevision !== map.revision;
    }

    /** True when a unit standing here has a direction to follow. */
    hasFlow(tx: number, ty: number): boolean {
        if (!this.contains(tx, ty)) return false;
        const slot = this.slot(tx, ty);
        return this.cost[slot] !== UNREACHABLE && (this.dirX[slot] !== 0 || this.dirY[slot] !== 0);
    }

    /** Writes the unit direction at a cell into `out` as [x, y]. */
    directionAt(tx: number, ty: number, out: Float64Array): void {
        const slot = this.slot(tx, ty);
        out[0] = this.dirX[slot];
        out[1] = this.dirY[slot];
    }

    costAt(tx: number, ty: number): number {
        if (!this.contains(tx, ty)) return UNREACHABLE;
        return this.cost[this.slot(tx, ty)];
    }

    isLineOfSight(tx: number, ty: number): boolean {
        return this.contains(tx, ty) && this.los[this.slot(tx, ty)] === 1;
    }

    /**
     * Rebuilds this segment to serve `sector` on the way to `goal`.
     *
     * `routeScratch` is borrowed for the abstract route lookup.
     */
    build(
        map: TileMap,
        graph: PortalGraph,
        sector: number,
        goal: number,
        heap: TieBrokenHeap,
        routeScratch: Int32Array,
    ): void {
        const {layout} = graph;

        this.sector = sector;
        this.goal = goal;
        this.terrainRevision = map.revision;
        this.expanded = 0;
        this.losCells = 0;
        this.target = -1;

        // Window: the 3x3 block of sectors around the served one, clipped.
        const sx = layout.sectorX(sector);
        const sy = layout.sectorY(sector);
        this.x0 = Math.max(0, (sx - 1) * layout.sectorSize);
        this.y0 = Math.max(0, (sy - 1) * layout.sectorSize);
        this.x1 = Math.min(map.width, (sx + 2) * layout.sectorSize);
        this.y1 = Math.min(map.height, (sy + 2) * layout.sectorSize);
        this.windowWidth = this.x1 - this.x0;

        const cells = this.windowWidth * (this.y1 - this.y0);
        this.cost.fill(UNREACHABLE, 0, cells);
        this.dirX.fill(0, 0, cells);
        this.dirY.fill(0, 0, cells);
        this.los.fill(0, 0, cells);
        this.closed.fill(0, 0, cells);

        this.target = this.pickTarget(map, graph, sector, goal, routeScratch);
        if (this.target === -1) return;

        this.runLineOfSight(
            map,
            layout.originX(sector),
            layout.originY(sector),
            layout.endX(sector),
            layout.endY(sector),
        );
        this.runDijkstra(map, heap);
    }

    /**
     * Picks the cell to integrate from: the last cell on the abstract route
     * before it leaves this window.
     *
     * Reaching as far upstream as the window allows is what keeps a segment's
     * flow close to what a whole-map field would have produced — aiming only at
     * the portal on the immediate boundary is the "poor accuracy" case.
     *
     * Consulting the route even when the goal is already inside the window
     * matters more than it looks. A goal a few tiles away can still need a
     * detour that leaves the window — the far side of a long wall is the
     * everyday case — and a segment that aimed straight at it would find no
     * route at all and strand the unit against the wall's face. Walking the
     * route and stopping where it exits the window gets both cases right.
     *
     * The route is computed from the sector rather than from any one unit, so
     * a sector split into two pockets by a wall may produce a target useless
     * to units in the other pocket. `Pathfinder.assign` catches that by
     * checking for flow where the unit actually stands.
     */
    private pickTarget(
        map: TileMap,
        graph: PortalGraph,
        sector: number,
        goal: number,
        routeScratch: Int32Array,
    ): number {
        const origin = map.index(
            Math.min(graph.layout.originX(sector), map.width - 1),
            Math.min(graph.layout.originY(sector), map.height - 1),
        );
        const from = map.isPassableIndex(origin)
            ? origin
            : this.anyPassableInSector(map, graph, sector);
        if (from === -1) return -1;

        const count = graph.route(from, goal, routeScratch, this.axisScratch);
        if (count === 0) return -1;

        // Stop at the first entry outside the window rather than scanning for
        // the last one inside it: beyond that point the route is describing
        // ground this segment has not integrated.
        let best = -1;
        let bestAxis = 0;
        for (let i = 0; i < count; i++) {
            const cell = routeScratch[i];
            if (!this.contains(map.tileX(cell), map.tileY(cell))) break;
            best = cell;
            bestAxis = this.axisScratch[i];
        }

        if (best === -1 || best === goal) return best;
        return slideAlongRun(map, graph, best, bestAxis, goal);
    }

    private anyPassableInSector(map: TileMap, graph: PortalGraph, sector: number): number {
        const {layout} = graph;
        for (let ty = layout.originY(sector); ty < layout.endY(sector); ty++) {
            for (let tx = layout.originX(sector); tx < layout.endX(sector); tx++) {
                if (map.isPassable(tx, ty)) return map.index(tx, ty);
            }
        }
        return -1;
    }

    /**
     * Gives every cell of the served sector with a clear line to the target the
     * exact direction to it.
     *
     * Only run over the served sector, not the whole window: units read flow
     * where they stand, and the outer ring exists to make the Dijkstra pass
     * accurate rather than to be walked.
     */
    private runLineOfSight(map: TileMap, x0: number, y0: number, x1: number, y1: number): void {
        const tx = map.tileX(this.target);
        const ty = map.tileY(this.target);

        for (let y = y0; y < y1; y++) {
            for (let x = x0; x < x1; x++) {
                if (!uniformOpen(map, x, y)) continue;
                if (!lineClear(map, x, y, tx, ty)) continue;

                const dx = tx - x;
                const dy = ty - y;
                const distance = Math.sqrt(dx * dx + dy * dy);
                const slot = this.slot(x, y);

                this.los[slot] = 1;
                this.losCells++;
                if (distance === 0) {
                    this.cost[slot] = 0;
                    continue;
                }
                this.cost[slot] = Math.round(distance * CARDINAL_COST);
                this.dirX[slot] = dx / distance;
                this.dirY[slot] = dy / distance;
            }
        }
    }

    /**
     * Integrates everything the LOS pass could not answer.
     *
     * Seeded from the target *and* from every LOS cell, so shadowed cells get
     * costs continuous with the exact region.
     *
     * LOS cells are locked against relaxation. They have to be: `DIAGONAL_COST`
     * is 14 where a true diagonal is 14.142, so the eight-connected metric
     * slightly *underestimates* diagonal distance and would otherwise
     * "improve" on — and overwrite — the exact straight-line answer, putting
     * the doglegs straight back.
     */
    private runDijkstra(map: TileMap, heap: TieBrokenHeap): void {
        heap.clear();

        const targetX = map.tileX(this.target);
        const targetY = map.tileY(this.target);
        if (this.contains(targetX, targetY) && map.isPassable(targetX, targetY)) {
            const slot = this.slot(targetX, targetY);
            if (this.cost[slot] !== 0) this.cost[slot] = 0;
            heap.push(this.target, 0, 0);
        }

        for (let y = this.y0; y < this.y1; y++) {
            for (let x = this.x0; x < this.x1; x++) {
                const slot = this.slot(x, y);
                if (this.los[slot] === 1 && this.cost[slot] !== 0) {
                    heap.push(map.index(x, y), this.cost[slot], 0);
                }
            }
        }

        while (heap.length > 0) {
            const current = heap.pop();
            const cx = map.tileX(current);
            const cy = map.tileY(current);
            const currentSlot = this.slot(cx, cy);
            if (this.closed[currentSlot] === 1) continue;
            this.closed[currentSlot] = 1;
            const currentCost = this.cost[currentSlot];
            this.expanded++;

            const enterCurrent = map.weight[current];

            for (let n = 0; n < 8; n++) {
                const nx = cx + NEIGHBOUR_DX[n];
                const ny = cy + NEIGHBOUR_DY[n];
                if (!this.contains(nx, ny)) continue;
                if (!map.isPassable(nx, ny)) continue;
                if (NEIGHBOUR_DIAGONAL[n] && (!map.isPassable(cx, ny) || !map.isPassable(nx, cy))) {
                    continue;
                }

                const step = NEIGHBOUR_DIAGONAL[n] ? DIAGONAL_COST : CARDINAL_COST;
                const candidate = currentCost + step * enterCurrent;
                const slot = this.slot(nx, ny);
                if (this.los[slot] === 1 || this.closed[slot] === 1) continue;
                if (candidate >= this.cost[slot]) continue;

                this.cost[slot] = candidate;
                // Shadowed cells get eight directions. They hug obstacles,
                // where a straight line would be wrong anyway.
                const inverse = NEIGHBOUR_DIAGONAL[n] ? Math.SQRT1_2 : 1;
                this.dirX[slot] = -NEIGHBOUR_DX[n] * inverse;
                this.dirY[slot] = -NEIGHBOUR_DY[n] * inverse;
                heap.push(map.index(nx, ny), candidate, 0);
            }
        }
    }
}

/**
 * Slides a portal entry along its boundary toward the goal's line.
 *
 * A portal node sits at the *midpoint* of its boundary run, which quietly
 * ruins otherwise straight journeys: a unit at y=32 heading to a goal at y=32
 * was being aimed at y=39, the middle of a 16-cell run, dragging it seven
 * tiles off course and back again — 7% excess distance on open ground.
 *
 * The graph does not need more nodes to fix that. The whole run is passable, so
 * any cell on it is a legal crossing; walk from the node toward the goal's
 * coordinate for as long as both sides of the boundary stay open.
 */
function slideAlongRun(
    map: TileMap,
    graph: PortalGraph,
    entry: number,
    axis: number,
    goal: number,
): number {
    const size = graph.layout.sectorSize;
    const ex = map.tileX(entry);
    const ey = map.tileY(entry);

    if (axis === AXIS_Y) {
        // Vertical boundary: the run goes along y, and the sector it was
        // entered from lies on whichever side the boundary is.
        const near = ex % size === 0 ? ex - 1 : ex + 1;
        const wanted = map.tileY(goal);
        let y = ey;
        while (y !== wanted) {
            const next = y + (wanted > y ? 1 : -1);
            // Stay inside the entry's own sector, and keep both sides open.
            if (((next / size) | 0) !== ((ey / size) | 0)) break;
            if (!map.isPassable(ex, next) || !map.isPassable(near, next)) break;
            y = next;
        }
        return map.index(ex, y);
    }

    if (axis === AXIS_X) {
        const near = ey % size === 0 ? ey - 1 : ey + 1;
        const wanted = map.tileX(goal);
        let x = ex;
        while (x !== wanted) {
            const next = x + (wanted > x ? 1 : -1);
            if (((next / size) | 0) !== ((ex / size) | 0)) break;
            if (!map.isPassable(next, ey) || !map.isPassable(next, near)) break;
            x = next;
        }
        return map.index(x, ey);
    }

    return entry;
}

/** Passable and of baseline cost, so a straight-line cost estimate is valid. */
function uniformOpen(map: TileMap, tx: number, ty: number): boolean {
    return map.inBounds(tx, ty) && map.weight[map.index(tx, ty)] === NORMAL;
}

/**
 * Supercover walk: visits every cell the segment between two cell centres
 * actually crosses.
 *
 * A plain Bresenham walk is *not* a conservative substitute, which is easy to
 * get wrong. From (0,0) to (2,1) Bresenham visits (2,0) — which the line never
 * touches — and misses (1,1), which it does. A line-of-sight test built on it
 * therefore declares a route clear straight through the corner of a building.
 *
 * Integer arithmetic throughout: the displacement between two cell centres is
 * a whole number of cells, so crossing times compare exactly and the walk is
 * bit-identical everywhere.
 */
function lineClear(map: TileMap, x0: number, y0: number, x1: number, y1: number): boolean {
    const dx = x1 - x0;
    const dy = y1 - y0;
    const ax = Math.abs(dx);
    const ay = Math.abs(dy);
    const sx = dx > 0 ? 1 : -1;
    const sy = dy > 0 ? 1 : -1;

    let x = x0;
    let y = y0;
    let ix = 0;
    let iy = 0;

    if (!uniformOpen(map, x, y)) return false;

    while (ix < ax || iy < ay) {
        // Which cell boundary the line reaches first, compared without division.
        const nextX = (2 * ix + 1) * ay;
        const nextY = (2 * iy + 1) * ax;

        if (nextX === nextY) {
            // Straight through a lattice corner. Require both flanking cells
            // open as well, matching the corner rule the planners use, so a
            // unit is never told it can squeeze between two blocked tiles.
            if (!uniformOpen(map, x + sx, y) || !uniformOpen(map, x, y + sy)) return false;
            x += sx;
            y += sy;
            ix++;
            iy++;
        } else if (nextX < nextY) {
            x += sx;
            ix++;
        } else {
            y += sy;
            iy++;
        }

        if (!uniformOpen(map, x, y)) return false;
    }

    return true;
}

/**
 * Fixed-size cache of flow segments, keyed by (served sector, goal).
 *
 * That key is why an army is cheap: every unit in a sector heading to the same
 * place reads one segment, and the segments a route crosses are reusable
 * building blocks for any later order across the same ground.
 *
 * Eviction is first-in-first-out rather than least-recently-used, so reads stay
 * free of state changes — movement reads a segment for every unit, every tick.
 * Dropping one still in use is self-healing: its units re-request.
 */
export class FlowSegmentCache {
    private readonly map: TileMap;
    private readonly graph: PortalGraph;
    private readonly capacity: number;
    private readonly heap: TieBrokenHeap;
    private readonly routeScratch = new Int32Array(256);
    private readonly segments = new Map<number, FlowSegment>();
    private readonly pool: FlowSegment[] = [];
    private readonly windowCapacity: number;

    lastBuilt = 0;
    lastExpanded = 0;

    constructor(map: TileMap, graph: PortalGraph, capacity: number) {
        this.map = map;
        this.graph = graph;
        this.capacity = Math.max(1, capacity);
        const span = 3 * graph.layout.sectorSize;
        this.windowCapacity = span * span;
        this.heap = new TieBrokenHeap(this.windowCapacity * 4);
    }

    private key(sector: number, goal: number): number {
        return sector * this.map.tileCount + goal;
    }

    /** Reads a usable segment without touching cache order. */
    peek(sector: number, goal: number): FlowSegment | undefined {
        const key = this.key(sector, goal);
        const segment = this.segments.get(key);
        if (segment === undefined) return undefined;
        if (segment.isStale(this.map)) {
            this.segments.delete(key);
            this.pool.push(segment);
            return undefined;
        }
        return segment;
    }

    has(sector: number, goal: number): boolean {
        return this.peek(sector, goal) !== undefined;
    }

    /** Builds the segment serving `sector` toward `goal`, evicting if full. */
    build(sector: number, goal: number): FlowSegment {
        const existing = this.peek(sector, goal);
        if (existing !== undefined) return existing;

        this.graph.ensureFresh();

        let segment: FlowSegment;
        if (this.segments.size >= this.capacity) {
            const oldest = this.segments.keys().next().value as number;
            segment = this.segments.get(oldest)!;
            this.segments.delete(oldest);
        } else {
            segment = this.pool.pop() ?? new FlowSegment(this.windowCapacity);
        }

        segment.build(this.map, this.graph, sector, goal, this.heap, this.routeScratch);
        this.segments.set(this.key(sector, goal), segment);

        this.lastBuilt++;
        this.lastExpanded += segment.expanded;
        return segment;
    }

    resetCounters(): void {
        this.lastBuilt = 0;
        this.lastExpanded = 0;
    }

    get size(): number {
        return this.segments.size;
    }

    hashInto(hasher: Hasher): void {
        hasher.writeU32(this.segments.size);
        for (const key of this.segments.keys()) {
            hasher.writeU32(key >>> 0);
        }
    }
}
