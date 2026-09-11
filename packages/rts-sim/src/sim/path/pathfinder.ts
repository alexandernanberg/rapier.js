import {entityExists} from "bitecs";
import type {Hasher} from "../../core/hash";
import type {TileMap} from "../grid/tile_map";
import {PathState} from "../components";
import {PortalGraph} from "../grid/portal_graph";
import {idOf, type SimWorld} from "../world";
import {FlowSegmentCache} from "./flow_segment";

/**
 * Route requests, answered with flow segments.
 *
 * There is only one mechanism now. An earlier version chose between a per-unit
 * A* search and a whole-map flow field depending on how many units shared a
 * destination, which existed only because a whole-map field was expensive
 * enough to need rationing. Once a field is bounded to a window of sectors it
 * costs less than a single long A* search, so the choice — and the threshold
 * that drove it — disappears. Age of Empires IV reports 0.247ms for a single
 * unit's flow on a 1024x1024 map for the same reason.
 *
 * The budget is a *count* of segments built per tick, never a time slice: "as
 * many as fit in 3ms" makes the simulation a function of how fast the machine
 * is, which desyncs on the first slow frame. Reading an already-cached segment
 * costs no budget at all, which is what makes a whole army nearly free.
 */
export class Pathfinder {
    readonly graph: PortalGraph;
    readonly segments: FlowSegmentCache;

    /** FIFO of pending requests, as parallel arrays. */
    private eids: number[] = [];
    private goals: number[] = [];
    /** Position in `eids` per entity, so a re-order replaces rather than piles up. */
    private readonly indexByEid = new Map<number, number>();
    /**
     * Requests still waiting. Tracked rather than derived from `eids.length`,
     * which also counts tombstones left by `cancel`.
     */
    private liveCount = 0;

    /** Requests resolved on the most recent tick, cached or built. */
    lastServiced = 0;
    /** Segments built on the most recent tick. */
    lastBuilt = 0;
    /** Cells integrated building them. */
    lastExpanded = 0;

    constructor(map: TileMap, sectorSize: number, segmentCapacity: number) {
        this.graph = new PortalGraph(map, sectorSize);
        this.graph.build();
        this.segments = new FlowSegmentCache(map, this.graph, segmentCapacity);
    }

    /** Units waiting for a segment. Excludes cancelled slots. */
    get pending(): number {
        return this.liveCount;
    }

    /**
     * Queues a request, or retargets one already queued.
     *
     * Retargeting keeps the original queue position: a player who re-clicks
     * should not be able to jump the queue ahead of someone who asked first.
     */
    request(eid: number, goalTile: number): void {
        const existing = this.indexByEid.get(eid);
        if (existing !== undefined) {
            this.goals[existing] = goalTile;
            return;
        }
        this.indexByEid.set(eid, this.eids.length);
        this.eids.push(eid);
        this.goals.push(goalTile);
        this.liveCount++;
    }

    cancel(eid: number): void {
        const index = this.indexByEid.get(eid);
        if (index === undefined) return;
        // Tombstone rather than splice: shifting would move every later
        // request's index and cost O(n) per cancel.
        this.eids[index] = -1;
        this.indexByEid.delete(eid);
        this.liveCount--;
    }

    /** Spends this tick's segment budget, cheapest requests first. */
    service(world: SimWorld): void {
        const budget = world.config.segmentBudget;
        this.segments.resetCounters();
        this.lastServiced = 0;
        this.lastBuilt = 0;
        this.lastExpanded = 0;

        if (this.eids.length === 0) return;

        let built = 0;
        let cursor = 0;

        while (cursor < this.eids.length) {
            const eid = this.eids[cursor];
            const goal = this.goals[cursor];

            if (eid === -1) {
                cursor++; // cancelled; already off the live count
                continue;
            }

            // Died between asking and being served. Consumes a slot but no
            // budget, since no work runs.
            if (!entityExists(world, eid)) {
                this.consume(eid);
                cursor++;
                continue;
            }

            const sector = this.sectorOf(world, eid);
            if (sector === -1) {
                this.fail(world, eid);
                this.consume(eid);
                this.lastServiced++;
                cursor++;
                continue;
            }

            if (!this.segments.has(sector, goal)) {
                if (built >= budget) {
                    // Budget spent. Leave the rest queued so FIFO order — and
                    // therefore fairness between players — is preserved.
                    break;
                }
                this.segments.build(sector, goal);
                built++;
            }

            this.assign(world, eid, sector, goal);
            this.consume(eid);
            this.lastServiced++;
            cursor++;
        }

        if (cursor > 0) {
            this.eids = this.eids.slice(cursor);
            this.goals = this.goals.slice(cursor);
            this.reindex();
        }

        this.lastBuilt = built;
        this.lastExpanded = this.segments.lastExpanded;
    }

    private sectorOf(world: SimWorld, eid: number): number {
        const {Position} = world.stores;
        const id = idOf(world, eid);
        const tx = world.map.worldToTileX(Position.x[id]);
        const ty = world.map.worldToTileY(Position.y[id]);
        if (!world.map.inBounds(tx, ty)) return -1;
        return this.graph.layout.sectorOfTile(tx, ty);
    }

    /** Points a unit at a segment, or fails it when that segment cannot help. */
    private assign(world: SimWorld, eid: number, sector: number, goal: number): void {
        const {Path, Position} = world.stores;
        const id = idOf(world, eid);
        const segment = this.segments.peek(sector, goal);
        const tx = world.map.worldToTileX(Position.x[id]);
        const ty = world.map.worldToTileY(Position.y[id]);

        if (segment === undefined || !segment.hasFlow(tx, ty)) {
            // No flow where the unit stands: walled off, or the goal is
            // unreachable from here.
            this.fail(world, eid);
            return;
        }

        Path.state[id] = PathState.Flow;
        Path.sector[id] = sector;
        Path.goal[id] = goal;
    }

    private fail(world: SimWorld, eid: number): void {
        const {Path} = world.stores;
        const id = idOf(world, eid);
        Path.state[id] = PathState.Failed;
        Path.sector[id] = -1;
    }

    private consume(eid: number): void {
        this.indexByEid.delete(eid);
        this.liveCount--;
    }

    private reindex(): void {
        this.indexByEid.clear();
        for (let i = 0; i < this.eids.length; i++) {
            const eid = this.eids[i];
            if (eid !== -1) this.indexByEid.set(eid, i);
        }
    }

    /**
     * Hashes the pending queue and the segment cache's key set.
     *
     * Both live outside the component stores, and both change how a unit gets
     * routed: two clients with a differently-ordered backlog, or a different
     * set of cached segments, walk differently a few ticks later. The segments
     * themselves are a pure function of terrain and goal, and terrain is
     * already hashed, so only the keys need to be.
     */
    hashInto(hasher: Hasher): void {
        hasher.writeU32(this.eids.length);
        for (let i = 0; i < this.eids.length; i++) {
            hasher.writeU32(this.eids[i] >>> 0);
            hasher.writeU32(this.goals[i] >>> 0);
        }
        this.segments.hashInto(hasher);
    }
}
