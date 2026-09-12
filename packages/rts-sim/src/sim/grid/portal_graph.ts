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
 * sectors are worth integrating at all.
 *
 * **Rebuilds are incremental.** A whole-graph rebuild costs 20ms on a 128x128
 * map and 368ms on 512x512, and in an RTS terrain changes every time a building
 * completes, so paying that per change is not an option. Node indices are
 * therefore *stable*: every sector owns a fixed arena, subdivided by which of
 * its four boundaries a node belongs to, so one boundary can be recomputed
 * without disturbing anything else. A change relinks only the sectors whose
 * node sets actually moved.
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

const DIR_NORTH = 0;
const DIR_EAST = 1;
const DIR_SOUTH = 2;
const DIR_WEST = 3;

export class PortalGraph {
    readonly map: TileMap;
    readonly layout: SectorLayout;

    /** Node slots per boundary direction, per sector. */
    private readonly perDir: number;
    /** Node slots per sector: four boundaries' worth. */
    private readonly slotsPerSector: number;
    /** Edge slots per node: its portal twin plus every other node in its sector. */
    private readonly edgesPerNode: number;

    private readonly alive: Uint8Array;
    private readonly nodeCell: Int32Array;
    private readonly nodeSectorOf: Int32Array;

    private readonly edgeCount: Int32Array;
    private readonly edgeTarget: Int32Array;
    private readonly edgeCost: Int32Array;

    /** Sectors needing a relink, as flags so iteration order is unambiguous. */
    private readonly dirtySector: Uint8Array;
    private dirtyCount = 0;
    private builtRevision = -1;
    private readonly dirtyRegion = new Int32Array(4);

    // Per-sector search scratch, reused across every confined Dijkstra.
    private readonly localCost: Int32Array;
    private readonly localStamp: Int32Array;
    private localGeneration = 0;
    private readonly localHeap: TieBrokenHeap;

    // Route scratch.
    private readonly routeG: Int32Array;
    private readonly routeFrom: Int32Array;
    private readonly routeStamp: Int32Array;
    private routeGeneration = 0;
    private readonly routeHeap: TieBrokenHeap;
    private readonly routeNodes: Int32Array;
    private readonly routeCosts: Int32Array;
    private readonly goalNodes: Int32Array;
    private readonly goalCosts: Int32Array;
    private readonly routePath: Int32Array;

    constructor(map: TileMap, sectorSize: number) {
        this.map = map;
        this.layout = new SectorLayout(map.width, map.height, sectorSize);

        // Runs along a boundary are separated by at least one blocked cell, so
        // a boundary of `sectorSize` cells holds at most half that many.
        this.perDir = Math.ceil(sectorSize / 2);
        this.slotsPerSector = 4 * this.perDir;
        this.edgesPerNode = this.slotsPerSector;

        const slots = this.layout.count * this.slotsPerSector;
        this.alive = new Uint8Array(slots);
        this.nodeCell = new Int32Array(slots);
        this.nodeSectorOf = new Int32Array(slots);
        this.edgeCount = new Int32Array(slots);
        this.edgeTarget = new Int32Array(slots * this.edgesPerNode);
        this.edgeCost = new Int32Array(slots * this.edgesPerNode);

        this.dirtySector = new Uint8Array(this.layout.count);

        this.localCost = new Int32Array(map.tileCount);
        this.localStamp = new Int32Array(map.tileCount);
        this.localHeap = new TieBrokenHeap(sectorSize * sectorSize * 2 + 8);

        // Two extra slots for the virtual goal a query splices in.
        this.routeG = new Int32Array(slots + 2);
        this.routeFrom = new Int32Array(slots + 2);
        this.routeStamp = new Int32Array(slots + 2);
        this.routeHeap = new TieBrokenHeap((slots + 2) * 4);
        this.routeNodes = new Int32Array(this.slotsPerSector);
        this.routeCosts = new Int32Array(this.slotsPerSector);
        this.goalNodes = new Int32Array(this.slotsPerSector);
        this.goalCosts = new Int32Array(this.slotsPerSector);
        this.routePath = new Int32Array(slots + 2);
    }

    /** Live node count. O(slots); for tests and diagnostics, not hot paths. */
    get nodes(): number {
        let count = 0;
        for (let i = 0; i < this.alive.length; i++) count += this.alive[i];
        return count;
    }

    get isStale(): boolean {
        return this.builtRevision !== this.map.revision;
    }

    /** Rebuilds whatever terrain changes have invalidated. */
    ensureFresh(): void {
        if (!this.isStale) return;

        if (this.builtRevision === -1 || !this.map.consumeDirtyRegion(this.dirtyRegion)) {
            this.build();
            return;
        }

        const {layout} = this;
        const minSx = Math.max(0, ((this.dirtyRegion[0] / layout.sectorSize) | 0) - 1);
        const minSy = Math.max(0, ((this.dirtyRegion[1] / layout.sectorSize) | 0) - 1);
        const maxSx = Math.min(
            layout.cols - 1,
            ((this.dirtyRegion[2] / layout.sectorSize) | 0) + 1,
        );
        const maxSy = Math.min(
            layout.rows - 1,
            ((this.dirtyRegion[3] / layout.sectorSize) | 0) + 1,
        );

        // One sector beyond the change on every side: a portal is shared, so
        // altering a boundary moves nodes in the sector across it too.
        for (let sy = minSy; sy <= maxSy; sy++) {
            for (let sx = minSx; sx <= maxSx; sx++) {
                this.markDirty(layout.sectorIndex(sx, sy));
            }
        }

        this.rebuildDirty();
    }

    /** Full rebuild. Marks every sector dirty and runs the same path. */
    build(): void {
        this.map.consumeDirtyRegion(this.dirtyRegion);
        this.alive.fill(0);
        for (let sector = 0; sector < this.layout.count; sector++) this.markDirty(sector);
        this.rebuildDirty();
    }

    private markDirty(sector: number): void {
        if (this.dirtySector[sector] === 1) return;
        this.dirtySector[sector] = 1;
        this.dirtyCount++;
    }

    /**
     * Recomputes portals on every boundary touching a dirty sector, then
     * relinks the sectors whose node sets changed.
     *
     * Boundaries are visited in sector-index order so the result never depends
     * on the order changes arrived in.
     */
    private rebuildDirty(): void {
        this.builtRevision = this.map.revision;
        if (this.dirtyCount === 0) return;

        const {layout, dirtySector} = this;

        // Every boundary has a canonical owner — its west/north sector — so
        // each is rebuilt exactly once.
        for (let sector = 0; sector < layout.count; sector++) {
            if (dirtySector[sector] === 0) continue;

            const sx = layout.sectorX(sector);
            const sy = layout.sectorY(sector);

            if (sx + 1 < layout.cols) this.rebuildBoundary(sector, DIR_EAST);
            if (sy + 1 < layout.rows) this.rebuildBoundary(sector, DIR_SOUTH);
            // A dirty sector's west and north boundaries are owned by its
            // neighbours, which may not themselves be dirty.
            if (sx > 0 && dirtySector[layout.sectorIndex(sx - 1, sy)] === 0) {
                this.rebuildBoundary(layout.sectorIndex(sx - 1, sy), DIR_EAST);
            }
            if (sy > 0 && dirtySector[layout.sectorIndex(sx, sy - 1)] === 0) {
                this.rebuildBoundary(layout.sectorIndex(sx, sy - 1), DIR_SOUTH);
            }
        }

        // Relink anything dirty, plus the far side of any boundary it shares.
        for (let sector = 0; sector < layout.count; sector++) {
            if (dirtySector[sector] === 0) continue;
            this.relinkSector(sector);

            const sx = layout.sectorX(sector);
            const sy = layout.sectorY(sector);
            for (const neighbour of [
                sx > 0 ? layout.sectorIndex(sx - 1, sy) : -1,
                sx + 1 < layout.cols ? layout.sectorIndex(sx + 1, sy) : -1,
                sy > 0 ? layout.sectorIndex(sx, sy - 1) : -1,
                sy + 1 < layout.rows ? layout.sectorIndex(sx, sy + 1) : -1,
            ]) {
                if (neighbour !== -1 && dirtySector[neighbour] === 0) {
                    this.relinkSector(neighbour);
                }
            }
        }

        this.dirtySector.fill(0);
        this.dirtyCount = 0;
    }

    private slotBase(sector: number, dir: number): number {
        return sector * this.slotsPerSector + dir * this.perDir;
    }

    /**
     * Recomputes the portals on one boundary, writing a node into each side's
     * slot group for that direction.
     *
     * `dir` is `DIR_EAST` or `DIR_SOUTH`; those two cover every boundary once.
     */
    private rebuildBoundary(sector: number, dir: number): void {
        const {map, layout} = this;
        const east = dir === DIR_EAST;
        const other = east
            ? layout.sectorIndex(layout.sectorX(sector) + 1, layout.sectorY(sector))
            : layout.sectorIndex(layout.sectorX(sector), layout.sectorY(sector) + 1);
        const oppositeDir = east ? DIR_WEST : DIR_NORTH;

        const nearBase = this.slotBase(sector, dir);
        const farBase = this.slotBase(other, oppositeDir);
        for (let i = 0; i < this.perDir; i++) {
            this.killNode(nearBase + i);
            this.killNode(farBase + i);
        }

        const boundary = east ? layout.originX(other) : layout.originY(other);
        const lo = east ? layout.originY(sector) : layout.originX(sector);
        const hi = east ? layout.endY(sector) : layout.endX(sector);

        let slot = 0;
        let runStart = -1;
        for (let i = lo; i <= hi; i++) {
            const open =
                i < hi &&
                (east
                    ? map.isPassable(boundary - 1, i) && map.isPassable(boundary, i)
                    : map.isPassable(i, boundary - 1) && map.isPassable(i, boundary));

            if (open && runStart === -1) {
                runStart = i;
            } else if (!open && runStart !== -1) {
                if (slot < this.perDir) {
                    const middle = (runStart + i - 1) >> 1;
                    const nearCell = east
                        ? map.index(boundary - 1, middle)
                        : map.index(middle, boundary - 1);
                    const farCell = east
                        ? map.index(boundary, middle)
                        : map.index(middle, boundary);
                    this.linkPortal(
                        nearBase + slot,
                        sector,
                        nearCell,
                        farBase + slot,
                        other,
                        farCell,
                    );
                    slot++;
                }
                runStart = -1;
            }
        }
    }

    private killNode(node: number): void {
        this.alive[node] = 0;
        this.edgeCount[node] = 0;
    }

    /** Creates the two nodes of one portal and the edge crossing between them. */
    private linkPortal(
        near: number,
        nearSector: number,
        nearCell: number,
        far: number,
        farSector: number,
        farCell: number,
    ): void {
        const {map} = this;

        this.alive[near] = 1;
        this.nodeCell[near] = nearCell;
        this.nodeSectorOf[near] = nearSector;
        this.alive[far] = 1;
        this.nodeCell[far] = farCell;
        this.nodeSectorOf[far] = farSector;

        // Slot 0 of each node is always its twin, so a relink can reset to one
        // edge and keep the crossing.
        this.edgeCount[near] = 1;
        this.edgeTarget[near * this.edgesPerNode] = far;
        this.edgeCost[near * this.edgesPerNode] = CARDINAL_COST * map.weight[farCell];

        this.edgeCount[far] = 1;
        this.edgeTarget[far * this.edgesPerNode] = near;
        this.edgeCost[far * this.edgesPerNode] = CARDINAL_COST * map.weight[nearCell];
    }

    /**
     * Recomputes portal-to-portal edges inside one sector.
     *
     * One confined Dijkstra per node gives exact costs to every other node in
     * the sector. Confined is what makes it cheap: the search cannot leave, so
     * it touches at most `sectorSize^2` cells however large the map.
     */
    private relinkSector(sector: number): void {
        const base = sector * this.slotsPerSector;

        for (let i = 0; i < this.slotsPerSector; i++) {
            const node = base + i;
            // Drop intra-sector edges, keep the portal crossing in slot 0.
            if (this.alive[node] === 1) this.edgeCount[node] = 1;
        }

        for (let i = 0; i < this.slotsPerSector; i++) {
            const source = base + i;
            if (this.alive[source] === 0) continue;
            this.searchWithinSector(sector, this.nodeCell[source]);

            for (let j = 0; j < this.slotsPerSector; j++) {
                if (i === j) continue;
                const target = base + j;
                if (this.alive[target] === 0) continue;
                const reached = this.localCostOf(this.nodeCell[target]);
                if (reached === IMPOSSIBLE) continue;

                const slot = this.edgeCount[source];
                if (slot >= this.edgesPerNode) break;
                this.edgeTarget[source * this.edgesPerNode + slot] = target;
                this.edgeCost[source * this.edgesPerNode + slot] = reached;
                this.edgeCount[source] = slot + 1;
            }
        }
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
        const base = sector * this.slotsPerSector;
        let count = 0;
        for (let i = 0; i < this.slotsPerSector && count < out.length; i++) {
            if (this.alive[base + i] === 1) out[count++] = base + i;
        }
        return count;
    }

    cellOfNode(node: number): number {
        return this.nodeCell[node];
    }

    sectorOfNode(node: number): number {
        return this.nodeSectorOf[node];
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

    /**
     * Abstract route from `startCell` to `goalCell`.
     *
     * Writes the *entry cell of each sector the route passes through*, in
     * order, ending with `goalCell`, and returns how many. That is precisely
     * what a flow segment needs: pick the entry a sector or two ahead and
     * integrate toward it. When `outAxis` is given it receives which way each
     * crossed boundary runs, so a caller can slide the entry along it.
     *
     * Returns 0 when no route exists. The search expands tens of portal nodes
     * where a grid A* would expand thousands of tiles.
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

        const path = this.searchGraph(goalSector, goalCell, startCount, goalCount);
        if (path === 0) return 0;

        return this.emitSectorEntries(startSector, goalCell, path, out, outAxis);
    }

    /**
     * A* over portal nodes, with a virtual goal spliced in.
     *
     * Returns the number of real nodes written to `routePath`, start-first.
     */
    private searchGraph(
        goalSector: number,
        goalCell: number,
        startCount: number,
        goalCount: number,
    ): number {
        const {routeG, routeFrom, routeStamp, routeHeap, map} = this;
        this.routeGeneration++;
        const gen = this.routeGeneration;
        routeHeap.clear();

        const virtualGoal = this.routeG.length - 1;
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
            if (routeStamp[current] !== gen) continue;
            const currentG = routeG[current];

            if (this.nodeSectorOf[current] === goalSector) {
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

            const edges = this.edgeCount[current];
            const base = current * this.edgesPerNode;
            for (let e = 0; e < edges; e++) {
                const next = this.edgeTarget[base + e];
                const candidate = currentG + this.edgeCost[base + e];
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
            const sector = this.nodeSectorOf[node];
            if (sector === lastSector) continue;

            if (outAxis !== undefined) {
                // Which way the crossed boundary runs. Sectors differ on
                // exactly one axis, so this is unambiguous.
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
}
