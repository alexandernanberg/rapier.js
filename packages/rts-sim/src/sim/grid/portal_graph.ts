import {TieBrokenHeap} from "./heap";
import {SectorLayout} from "./sectors";
import {CARDINAL_COST, DIAGONAL_COST, type TileMap} from "./tile_map";

/**
 * Abstract graph over the grid, after HPA*.
 *
 * Three steps, following Botea/Muller/Schaeffer and the way Age of Empires IV
 * describes its portal graph:
 *
 * 1. divide the grid into sectors
 * 2. detect portals — contiguous runs along a shared sector edge where both
 *    sides are passable — and place a node on each side of each run's middle
 * 3. find portal-to-portal cost inside each sector with a confined search
 *
 * A route over this graph is a handful of node hops rather than thousands of
 * tiles, and — the reason it exists — it tells a flow segment which couple of
 * sectors are worth integrating. Without it every field is a whole-map
 * Dijkstra, which is the single biggest cost in a naive implementation.
 *
 * Rebuilt wholesale when terrain changes. Per-sector incremental rebuild is the
 * obvious refinement and is not done yet — see `buildMs` in the tests for why
 * that is survivable for now.
 */

const NEIGHBOUR_DX = [0, 1, 1, 1, 0, -1, -1, -1];
const NEIGHBOUR_DY = [-1, -1, 0, 1, 1, 1, 0, -1];
const NEIGHBOUR_DIAGONAL = [false, true, false, true, false, true, false, true];

/** No route between two nodes. */
const IMPOSSIBLE = 0x7fffffff;

/** The entry is the goal itself; there is no boundary to slide along. */
export const AXIS_NONE = 0;
/** Entered through a vertical boundary, so the run goes along y. */
export const AXIS_Y = 1;
/** Entered through a horizontal boundary, so the run goes along x. */
export const AXIS_X = 2;

export class PortalGraph {
    readonly map: TileMap;
    readonly layout: SectorLayout;

    /** Tile each node sits on. */
    private nodeCell = new Int32Array(0);
    /** Sector each node belongs to. */
    private nodeSector = new Int32Array(0);
    private nodeCount = 0;
    /** Node indices per sector, as a CSR-style index. */
    private sectorNodeStart = new Int32Array(0);
    private sectorNodes = new Int32Array(0);

    /** CSR adjacency. */
    private edgeStart = new Int32Array(0);
    private edgeTarget = new Int32Array(0);
    private edgeCost = new Int32Array(0);

    /** Terrain revision this graph describes. */
    private builtRevision = -1;

    // Per-sector search scratch, reused across every confined Dijkstra.
    private readonly localCost: Int32Array;
    private readonly localStamp: Int32Array;
    private localGeneration = 0;
    private readonly localHeap: TieBrokenHeap;

    // Route scratch, grown to fit on each build.
    private routeG = new Int32Array(0);
    private routeFrom = new Int32Array(0);
    private routeStamp = new Int32Array(0);
    private routeGeneration = 0;
    private routeHeap = new TieBrokenHeap(0);
    private routeNodes: Int32Array;
    private routeCosts: Int32Array;
    private goalNodes: Int32Array;
    private goalCosts: Int32Array;
    private routePath = new Int32Array(0);

    constructor(map: TileMap, sectorSize: number) {
        this.map = map;
        this.layout = new SectorLayout(map.width, map.height, sectorSize);

        this.localCost = new Int32Array(map.tileCount);
        this.localStamp = new Int32Array(map.tileCount);
        this.localHeap = new TieBrokenHeap(sectorSize * sectorSize * 2 + 8);

        // A sector's boundaries hold at most ceil(size/2) portal runs each.
        const maxSectorNodes = 4 * sectorSize + 8;
        this.routeNodes = new Int32Array(maxSectorNodes);
        this.routeCosts = new Int32Array(maxSectorNodes);
        this.goalNodes = new Int32Array(maxSectorNodes);
        this.goalCosts = new Int32Array(maxSectorNodes);
    }

    get nodes(): number {
        return this.nodeCount;
    }

    get isStale(): boolean {
        return this.builtRevision !== this.map.revision;
    }

    /** Rebuilds if terrain has changed since the last build. */
    ensureFresh(): void {
        if (this.isStale) this.build();
    }

    build(): void {
        this.builtRevision = this.map.revision;
        this.collectPortals();
        this.linkWithinSectors();

        // Two extra slots for the virtual start and goal a query splices in.
        const slots = this.nodeCount + 2;
        if (this.routeG.length < slots) {
            this.routeG = new Int32Array(slots);
            this.routeFrom = new Int32Array(slots);
            this.routeStamp = new Int32Array(slots);
            this.routeHeap = new TieBrokenHeap(slots * 4);
            this.routePath = new Int32Array(slots);
            this.routeGeneration = 0;
        }
    }

    /**
     * Abstract route from `startCell` to `goalCell`.
     *
     * Writes the *entry cell of each sector the route passes through*, in
     * order, ending with `goalCell`, and returns how many. That is precisely
     * what a flow segment needs: pick the entry cell a sector or two ahead and
     * integrate toward it.
     *
     * Returns 0 when no route exists. The search runs over portal nodes, so it
     * expands tens of nodes where a grid A* would expand thousands of tiles.
     */
    route(startCell: number, goalCell: number, out: Int32Array, outAxis?: Uint8Array): number {
        this.ensureFresh();
        const {map, layout} = this;

        if (!map.isPassableIndex(startCell) || !map.isPassableIndex(goalCell)) return 0;
        if (startCell === goalCell) {
            out[0] = goalCell;
            if (outAxis !== undefined) outAxis[0] = AXIS_NONE;
            return 1;
        }

        const startSector = layout.sectorOfTile(map.tileX(startCell), map.tileY(startCell));
        const goalSector = layout.sectorOfTile(map.tileX(goalCell), map.tileY(goalCell));

        if (startSector === goalSector) {
            // Same sector: a confined search settles it without touching the
            // graph, and covers the common case of a short order.
            this.searchWithinSector(startSector, startCell);
            if (this.localCostOf(goalCell) !== IMPOSSIBLE) {
                out[0] = goalCell;
                if (outAxis !== undefined) outAxis[0] = AXIS_NONE;
                return 1;
            }
            // Otherwise the route has to leave the sector and come back, which
            // the graph search below handles.
        }

        const startCount = this.costsToSectorNodes(
            startSector,
            startCell,
            this.routeNodes,
            this.routeCosts,
        );
        const goalCount = this.costsFromSectorNodes(
            goalSector,
            goalCell,
            this.goalNodes,
            this.goalCosts,
        );
        if (startCount === 0 || goalCount === 0) return 0;

        const path = this.searchGraph(startSector, goalSector, goalCell, startCount, goalCount);
        if (path === 0) return 0;

        return this.emitSectorEntries(startSector, goalCell, path, out, outAxis);
    }

    /**
     * A* over portal nodes, with virtual start and goal nodes spliced in.
     *
     * Returns the number of real nodes written to `routePath`, start-first.
     */
    private searchGraph(
        startSector: number,
        goalSector: number,
        goalCell: number,
        startCount: number,
        goalCount: number,
    ): number {
        const {routeG, routeFrom, routeStamp, routeHeap, map} = this;
        this.routeGeneration++;
        const gen = this.routeGeneration;
        routeHeap.clear();

        const virtualGoal = this.nodeCount + 1;
        const goalX = map.tileX(goalCell);
        const goalY = map.tileY(goalCell);

        for (let i = 0; i < startCount; i++) {
            if (this.routeCosts[i] === IMPOSSIBLE) continue;
            const node = this.routeNodes[i];
            routeG[node] = this.routeCosts[i];
            routeFrom[node] = -1;
            routeStamp[node] = gen;
            routeHeap.push(node, this.routeCosts[i] + this.nodeHeuristic(node, goalX, goalY), 0);
        }
        let reached = false;
        while (routeHeap.length > 0) {
            const current = routeHeap.pop();
            if (current === virtualGoal) {
                reached = true;
                break;
            }

            const currentG = routeG[current];
            // Stale heap entry: a cheaper path to this node was found later.
            if (routeStamp[current] !== gen) continue;

            if (this.sectorOfNode(current) === goalSector) {
                const exit = this.goalCostFor(current, goalCount);
                if (exit !== IMPOSSIBLE) {
                    const candidate = currentG + exit;
                    if (routeStamp[virtualGoal] !== gen || candidate < routeG[virtualGoal]) {
                        routeG[virtualGoal] = candidate;
                        routeFrom[virtualGoal] = current;
                        routeStamp[virtualGoal] = gen;
                        routeHeap.push(virtualGoal, candidate, 0);
                    }
                }
            }

            const end = this.edgeStart[current + 1];
            for (let e = this.edgeStart[current]; e < end; e++) {
                const next = this.edgeTarget[e];
                const candidate = currentG + this.edgeCost[e];
                if (routeStamp[next] === gen && candidate >= routeG[next]) continue;
                routeG[next] = candidate;
                routeFrom[next] = current;
                routeStamp[next] = gen;
                routeHeap.push(next, candidate + this.nodeHeuristic(next, goalX, goalY), 0);
            }
        }

        if (!reached) return 0;

        let length = 0;
        for (let node = routeFrom[virtualGoal]; node !== -1; node = routeFrom[node]) {
            this.routePath[length++] = node;
        }
        // Reverse in place: the walk above is goal-first.
        for (let i = 0, j = length - 1; i < j; i++, j--) {
            const tmp = this.routePath[i];
            this.routePath[i] = this.routePath[j];
            this.routePath[j] = tmp;
        }
        return length;
    }

    private goalCostFor(node: number, goalCount: number): number {
        for (let i = 0; i < goalCount; i++) {
            if (this.goalNodes[i] === node) return this.goalCosts[i];
        }
        return IMPOSSIBLE;
    }

    private nodeHeuristic(node: number, goalX: number, goalY: number): number {
        const cell = this.nodeCell[node];
        const dx = Math.abs(this.map.tileX(cell) - goalX);
        const dy = Math.abs(this.map.tileY(cell) - goalY);
        const diagonal = dx < dy ? dx : dy;
        return CARDINAL_COST * (dx + dy) + (DIAGONAL_COST - 2 * CARDINAL_COST) * diagonal;
    }

    /**
     * Turns a node path into one entry cell per sector entered.
     *
     * Consecutive nodes on the path either share a sector (an intra-sector hop)
     * or are the two sides of one portal (a crossing). A sector change marks an
     * entry, and its node's cell is where the route comes in.
     */
    private emitSectorEntries(
        startSector: number,
        goalCell: number,
        pathLength: number,
        out: Int32Array,
        outAxis?: Uint8Array,
    ): number {
        let count = 0;
        let lastSector = startSector;

        for (let i = 0; i < pathLength && count < out.length - 1; i++) {
            const node = this.routePath[i];
            const sector = this.nodeSector[node];
            if (sector === lastSector) continue;

            if (outAxis !== undefined) {
                // Which way the crossed boundary runs, so a caller can slide
                // the entry along it. Sectors differ on exactly one axis.
                outAxis[count] =
                    this.layout.sectorX(sector) !== this.layout.sectorX(lastSector)
                        ? AXIS_Y
                        : AXIS_X;
            }
            out[count++] = this.nodeCell[node];
            lastSector = sector;
        }

        if (outAxis !== undefined) outAxis[count] = AXIS_NONE;
        out[count++] = goalCell;
        return count;
    }

    /**
     * Finds portals on every shared sector edge and creates the two nodes and
     * crossing edge for each.
     *
     * One node per side of a run's middle, rather than one per cell: a 16-cell
     * wide opening does not need 16 graph nodes, and the flow segment inside
     * the sector is what actually decides where a unit crosses.
     */
    private collectPortals(): void {
        const {map, layout} = this;
        const cells: number[] = [];
        const sectors: number[] = [];
        // Crossing edges, as (from, to, cost) triples.
        const crossFrom: number[] = [];
        const crossTo: number[] = [];
        const crossCost: number[] = [];

        const addNode = (cell: number, sector: number): number => {
            cells.push(cell);
            sectors.push(sector);
            return cells.length - 1;
        };

        const addPortal = (ax: number, ay: number, bx: number, by: number): void => {
            const aCell = map.index(ax, ay);
            const bCell = map.index(bx, by);
            const a = addNode(aCell, layout.sectorOfTile(ax, ay));
            const b = addNode(bCell, layout.sectorOfTile(bx, by));
            crossFrom.push(a, b);
            crossTo.push(b, a);
            crossCost.push(CARDINAL_COST * map.weight[bCell], CARDINAL_COST * map.weight[aCell]);
        };

        // Vertical edges: sector (sx, sy) against (sx + 1, sy).
        for (let sy = 0; sy < layout.rows; sy++) {
            for (let sx = 0; sx + 1 < layout.cols; sx++) {
                const boundary = layout.originX(layout.sectorIndex(sx + 1, sy));
                const sector = layout.sectorIndex(sx, sy);
                const y0 = layout.originY(sector);
                const y1 = layout.endY(sector);
                scanRuns(
                    y0,
                    y1,
                    (y) => map.isPassable(boundary - 1, y) && map.isPassable(boundary, y),
                    (mid) => {
                        addPortal(boundary - 1, mid, boundary, mid);
                    },
                );
            }
        }

        // Horizontal edges: sector (sx, sy) against (sx, sy + 1).
        for (let sy = 0; sy + 1 < layout.rows; sy++) {
            for (let sx = 0; sx < layout.cols; sx++) {
                const boundary = layout.originY(layout.sectorIndex(sx, sy + 1));
                const sector = layout.sectorIndex(sx, sy);
                const x0 = layout.originX(sector);
                const x1 = layout.endX(sector);
                scanRuns(
                    x0,
                    x1,
                    (x) => map.isPassable(x, boundary - 1) && map.isPassable(x, boundary),
                    (mid) => {
                        addPortal(mid, boundary - 1, mid, boundary);
                    },
                );
            }
        }

        this.nodeCount = cells.length;
        this.nodeCell = new Int32Array(cells);
        this.nodeSector = new Int32Array(sectors);
        this.indexSectorNodes();
        this.pendingCross = {crossFrom, crossTo, crossCost};
    }

    private pendingCross: {crossFrom: number[]; crossTo: number[]; crossCost: number[]} = {
        crossFrom: [],
        crossTo: [],
        crossCost: [],
    };

    private indexSectorNodes(): void {
        const {layout} = this;
        const counts = new Int32Array(layout.count + 1);
        for (let n = 0; n < this.nodeCount; n++) counts[this.nodeSector[n] + 1]++;
        for (let s = 0; s < layout.count; s++) counts[s + 1] += counts[s];

        const fill = new Int32Array(layout.count);
        const nodes = new Int32Array(this.nodeCount);
        for (let n = 0; n < this.nodeCount; n++) {
            const sector = this.nodeSector[n];
            nodes[counts[sector] + fill[sector]++] = n;
        }

        this.sectorNodeStart = counts;
        this.sectorNodes = nodes;
    }

    /**
     * Adds portal-to-portal edges inside each sector.
     *
     * One confined Dijkstra per node gives exact costs to every other node in
     * its sector. Confined is what makes it cheap: the search cannot leave a
     * sector, so it touches at most `sectorSize^2` cells however large the map.
     */
    private linkWithinSectors(): void {
        const {crossFrom, crossTo, crossCost} = this.pendingCross;
        const from: number[] = crossFrom.slice();
        const to: number[] = crossTo.slice();
        const cost: number[] = crossCost.slice();

        for (let sector = 0; sector < this.layout.count; sector++) {
            const start = this.sectorNodeStart[sector];
            const end = this.sectorNodeStart[sector + 1];
            if (end - start < 2) continue;

            for (let i = start; i < end; i++) {
                const source = this.sectorNodes[i];
                this.searchWithinSector(sector, this.nodeCell[source]);

                for (let j = start; j < end; j++) {
                    if (i === j) continue;
                    const target = this.sectorNodes[j];
                    const reached = this.localCostOf(this.nodeCell[target]);
                    if (reached === IMPOSSIBLE) continue;
                    from.push(source);
                    to.push(target);
                    cost.push(reached);
                }
            }
        }

        this.buildCsr(from, to, cost);
        this.pendingCross = {crossFrom: [], crossTo: [], crossCost: []};
    }

    private buildCsr(from: number[], to: number[], cost: number[]): void {
        const counts = new Int32Array(this.nodeCount + 1);
        for (const f of from) counts[f + 1]++;
        for (let n = 0; n < this.nodeCount; n++) counts[n + 1] += counts[n];

        const fill = new Int32Array(this.nodeCount);
        const target = new Int32Array(from.length);
        const edgeCost = new Int32Array(from.length);
        for (let e = 0; e < from.length; e++) {
            const slot = counts[from[e]] + fill[from[e]]++;
            target[slot] = to[e];
            edgeCost[slot] = cost[e];
        }

        this.edgeStart = counts;
        this.edgeTarget = target;
        this.edgeCost = edgeCost;
    }

    /**
     * Dijkstra from `origin`, confined to `sector`. Results are read back with
     * `localCostOf` until the next call.
     */
    private searchWithinSector(sector: number, origin: number): void {
        const {map, layout, localCost, localStamp, localHeap} = this;
        this.localGeneration++;
        const gen = this.localGeneration;
        localHeap.clear();

        if (!map.isPassableIndex(origin)) return;

        localCost[origin] = 0;
        localStamp[origin] = gen;
        localHeap.push(origin, 0, 0);

        // A tile finalised once; a stale re-push is skipped by cost check.
        while (localHeap.length > 0) {
            const current = localHeap.pop();
            const currentCost = localCost[current];
            const cx = map.tileX(current);
            const cy = map.tileY(current);

            for (let n = 0; n < 8; n++) {
                const nx = cx + NEIGHBOUR_DX[n];
                const ny = cy + NEIGHBOUR_DY[n];
                if (!layout.inSector(sector, nx, ny)) continue;
                if (!map.isPassable(nx, ny)) continue;
                if (NEIGHBOUR_DIAGONAL[n] && (!map.isPassable(cx, ny) || !map.isPassable(nx, cy))) {
                    continue;
                }

                const neighbour = map.index(nx, ny);
                const step = NEIGHBOUR_DIAGONAL[n] ? DIAGONAL_COST : CARDINAL_COST;
                const candidate = currentCost + step * map.weight[neighbour];

                if (localStamp[neighbour] === gen && candidate >= localCost[neighbour]) continue;
                localStamp[neighbour] = gen;
                localCost[neighbour] = candidate;
                localHeap.push(neighbour, candidate, 0);
            }
        }
    }

    private localCostOf(cell: number): number {
        return this.localStamp[cell] === this.localGeneration ? this.localCost[cell] : IMPOSSIBLE;
    }

    /** Node indices belonging to a sector. */
    nodesInSector(sector: number, out: Int32Array): number {
        const start = this.sectorNodeStart[sector];
        const end = this.sectorNodeStart[sector + 1];
        const count = Math.min(end - start, out.length);
        for (let i = 0; i < count; i++) out[i] = this.sectorNodes[start + i];
        return count;
    }

    cellOfNode(node: number): number {
        return this.nodeCell[node];
    }

    sectorOfNode(node: number): number {
        return this.nodeSector[node];
    }

    /** Cost from `cell` to every node in its own sector, via a confined search. */
    costsToSectorNodes(sector: number, cell: number, nodes: Int32Array, out: Int32Array): number {
        const count = this.nodesInSector(sector, nodes);
        this.searchWithinSector(sector, cell);
        for (let i = 0; i < count; i++) {
            out[i] = this.localCostOf(this.nodeCell[nodes[i]]);
        }
        return count;
    }

    /** Cost from each node in `sector` to `cell`, via a confined search. */
    costsFromSectorNodes(sector: number, cell: number, nodes: Int32Array, out: Int32Array): number {
        // Weights make the grid directed, so this is not simply the transpose
        // of `costsToSectorNodes`; search from each node instead.
        const count = this.nodesInSector(sector, nodes);
        for (let i = 0; i < count; i++) {
            this.searchWithinSector(sector, this.nodeCell[nodes[i]]);
            out[i] = this.localCostOf(cell);
        }
        return count;
    }
}

/**
 * Calls `onRun` with the middle index of each contiguous run in [lo, hi) where
 * `passable` holds.
 */
function scanRuns(
    lo: number,
    hi: number,
    passable: (i: number) => boolean,
    onRun: (middle: number) => void,
): void {
    let runStart = -1;
    for (let i = lo; i <= hi; i++) {
        const open = i < hi && passable(i);
        if (open && runStart === -1) {
            runStart = i;
        } else if (!open && runStart !== -1) {
            onRun((runStart + i - 1) >> 1);
            runStart = -1;
        }
    }
}
