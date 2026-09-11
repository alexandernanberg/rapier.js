import {entityExists} from "bitecs";
import type {Hasher} from "../../core/hash";
import type {TileMap} from "../grid/tile_map";
import {MAX_PATH, PathState} from "../components";
import {AStar} from "../grid/astar";
import {idOf, type SimWorld} from "../world";
import {FlowFieldCache} from "./flow_field";

/**
 * Route requests, and the policy for how each one gets answered.
 *
 * Three tiers, cheapest first:
 *
 * 1. **A cached flow field** — free, so it costs no budget at all. This is what
 *    makes a 200-unit army move for the price of one computation.
 * 2. **Building a flow field** — worth it once `flowFieldThreshold` units share
 *    a destination, bounded by `flowFieldBudget` per tick.
 * 3. **A single A\* search** — for scattered destinations, bounded by
 *    `pathBudget` per tick.
 *
 * Every budget is a *count*, never a time slice: "as many as fit in 3ms" makes
 * the simulation a function of how fast the machine is, which desyncs on the
 * first slow frame. A fixed K means a busy moment costs pathing latency rather
 * than correctness.
 *
 * The pending queue persists across ticks, so it is simulation state and gets
 * hashed — as does the cache's key set, since which tier a unit lands in
 * depends on what is cached.
 */
export class Pathfinder {
    private readonly astar: AStar;
    private readonly scratch: Int32Array;
    readonly flows: FlowFieldCache;

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
    /** Reused across ticks so the grouping pass allocates nothing. */
    private readonly goalCounts = new Map<number, number>();

    /** Requests resolved on the most recent tick, by any tier. */
    lastServiced = 0;
    /** A* searches run on the most recent tick. */
    lastSearches = 0;
    /** Flow fields built on the most recent tick. */
    lastFields = 0;
    /** Tiles expanded by searches and field builds together. */
    lastExpanded = 0;

    constructor(map: TileMap, flowFieldCapacity: number) {
        this.astar = new AStar(map);
        this.scratch = new Int32Array(MAX_PATH);
        this.flows = new FlowFieldCache(map, flowFieldCapacity);
    }

    /** Units waiting for a route. Excludes cancelled slots. */
    get pending(): number {
        return this.liveCount;
    }

    /**
     * Queues a route request, or retargets one already queued.
     *
     * Retargeting keeps the original queue position: a player who re-clicks
     * should not be able to jump the pathfinding queue ahead of someone who
     * asked first.
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

    /** Spends this tick's pathfinding budgets, cheapest tier first. */
    service(world: SimWorld): void {
        const {pathBudget, flowFieldBudget, flowFieldThreshold} = world.config;
        this.flows.resetCounters();
        this.lastServiced = 0;
        this.lastSearches = 0;
        this.lastFields = 0;
        this.lastExpanded = 0;

        if (this.eids.length === 0) return;

        this.countGoals(world);

        let searches = 0;
        let fields = 0;
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

            if (this.flows.has(goal)) {
                this.assignFlow(world, eid, goal);
            } else if (
                (this.goalCounts.get(goal) ?? 0) >= flowFieldThreshold &&
                fields < flowFieldBudget
            ) {
                this.flows.build(goal);
                fields++;
                this.assignFlow(world, eid, goal);
            } else if (searches < pathBudget) {
                this.runSearch(world, eid, goal);
                searches++;
            } else {
                // Both budgets spent. Leave the rest queued so FIFO order — and
                // therefore fairness between players — is preserved.
                break;
            }

            this.consume(eid);
            this.lastServiced++;
            cursor++;
        }

        if (cursor > 0) {
            this.eids = this.eids.slice(cursor);
            this.goals = this.goals.slice(cursor);
            this.reindex();
        }

        this.lastSearches = searches;
        this.lastFields = fields;
        this.lastExpanded += this.flows.lastExpanded;
    }

    /**
     * Counts live requests per destination, so the policy can tell an army
     * sharing a rally point from a handful of scattered scouts.
     */
    private countGoals(world: SimWorld): void {
        this.goalCounts.clear();
        for (let i = 0; i < this.eids.length; i++) {
            const eid = this.eids[i];
            if (eid === -1 || !entityExists(world, eid)) continue;
            const goal = this.goals[i];
            this.goalCounts.set(goal, (this.goalCounts.get(goal) ?? 0) + 1);
        }
    }

    private consume(eid: number): void {
        this.indexByEid.delete(eid);
        this.liveCount--;
    }

    /** Points a unit at a flow field, or fails it if the field cannot reach it. */
    private assignFlow(world: SimWorld, eid: number, goal: number): void {
        const {Path, Position} = world.stores;
        const id = idOf(world, eid);
        const field = this.flows.peek(goal);
        const start = world.map.worldToIndex(Position.x[id], Position.y[id]);

        if (field === undefined || start === -1 || !field.reaches(start)) {
            Path.state[id] = PathState.Failed;
            Path.length[id] = 0;
            Path.cursor[id] = 0;
            return;
        }

        // A field needs no waypoints: movement reads it fresh each tick, from
        // wherever the unit actually stands.
        Path.state[id] = PathState.Flow;
        Path.length[id] = 0;
        Path.cursor[id] = 0;
    }

    private runSearch(world: SimWorld, eid: number, goal: number): void {
        const {Path, Position} = world.stores;
        const id = idOf(world, eid);
        const start = world.map.worldToIndex(Position.x[id], Position.y[id]);

        if (start === -1) {
            Path.state[id] = PathState.Failed;
            Path.length[id] = 0;
            Path.cursor[id] = 0;
            return;
        }

        if (start === goal) {
            // Already standing on the goal tile; the final approach to the exact
            // order position is the movement system's job.
            Path.state[id] = PathState.Active;
            Path.length[id] = 0;
            Path.cursor[id] = 0;
            return;
        }

        const result = this.astar.search(start, goal, this.scratch);
        this.lastExpanded += result.expanded;

        if (result.length === 0) {
            Path.state[id] = PathState.Failed;
            Path.length[id] = 0;
            Path.cursor[id] = 0;
            return;
        }

        const base = id * MAX_PATH;
        for (let i = 0; i < result.length; i++) {
            Path.tiles[base + i] = this.scratch[i];
        }
        Path.length[id] = result.length;
        Path.cursor[id] = 0;
        Path.state[id] = PathState.Active;
    }

    private reindex(): void {
        this.indexByEid.clear();
        for (let i = 0; i < this.eids.length; i++) {
            const eid = this.eids[i];
            if (eid !== -1) this.indexByEid.set(eid, i);
        }
    }

    /**
     * Hashes the pending queue and the cache's key set.
     *
     * Both are state living outside any component. Two clients holding the same
     * units but a differently-ordered backlog, or a different set of cached
     * fields, will route differently a few ticks later. Hashing turns that into
     * an immediate mismatch instead of a mysterious one.
     */
    hashInto(hasher: Hasher): void {
        hasher.writeU32(this.eids.length);
        for (let i = 0; i < this.eids.length; i++) {
            hasher.writeU32(this.eids[i] >>> 0);
            hasher.writeU32(this.goals[i] >>> 0);
        }
        this.flows.hashInto(hasher);
    }
}
