import {entityExists} from "bitecs";
import type {Hasher} from "../../core/hash";
import type {TileMap} from "../grid/tile_map";
import {MAX_PATH, PathState} from "../components";
import {AStar} from "../grid/astar";
import {idOf, type SimWorld} from "../world";

/**
 * Budgeted pathfinding.
 *
 * Requests queue up and a fixed number are serviced per tick. The budget is a
 * *count*, never a time slice: "as many as fit in 3ms" makes the simulation a
 * function of how fast the machine is, which is a desync on the first slow
 * frame. A fixed K means every client services exactly the same requests on
 * exactly the same ticks, and a busy moment shows up as pathing latency rather
 * than divergence.
 *
 * The pending queue persists across ticks, so it is simulation state and gets
 * hashed — see `hashInto`.
 */
export class PathQueue {
    private readonly astar: AStar;
    private readonly scratch: Int32Array;

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

    /** Requests serviced on the most recent tick. Diagnostics only. */
    lastServiced = 0;
    /** Tiles expanded on the most recent tick. Diagnostics only. */
    lastExpanded = 0;

    constructor(map: TileMap) {
        this.astar = new AStar(map);
        this.scratch = new Int32Array(MAX_PATH);
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

    /** Services up to `budget` requests, writing routes into the Path component. */
    service(world: SimWorld, budget: number): void {
        const {stores} = world;
        const {Path, Position} = stores;
        const map = world.map;

        let serviced = 0;
        let expanded = 0;
        let cursor = 0;

        while (cursor < this.eids.length && serviced < budget) {
            const eid = this.eids[cursor];
            const goal = this.goals[cursor];
            cursor++;

            if (eid === -1) continue; // cancelled; already off the live count
            this.indexByEid.delete(eid);
            this.liveCount--;

            // Died between asking and being served. Consumes a slot but not
            // budget, since no search runs.
            if (!entityExists(world, eid)) continue;

            const id = idOf(world, eid);
            const start = map.worldToIndex(Position.x[id], Position.y[id]);
            serviced++;

            if (start === -1) {
                Path.state[id] = PathState.Failed;
                Path.length[id] = 0;
                Path.cursor[id] = 0;
                continue;
            }
            if (start === goal) {
                // Already standing on the goal tile; the final approach to the
                // exact order position is the movement system's job.
                Path.state[id] = PathState.Active;
                Path.length[id] = 0;
                Path.cursor[id] = 0;
                continue;
            }

            const result = this.astar.search(start, goal, this.scratch);
            expanded += result.expanded;

            if (result.length === 0) {
                Path.state[id] = PathState.Failed;
                Path.length[id] = 0;
                Path.cursor[id] = 0;
                continue;
            }

            const base = id * MAX_PATH;
            for (let i = 0; i < result.length; i++) {
                Path.tiles[base + i] = this.scratch[i];
            }
            Path.length[id] = result.length;
            Path.cursor[id] = 0;
            Path.state[id] = PathState.Active;
        }

        // Drop everything consumed this tick in one go.
        if (cursor > 0) {
            this.eids = this.eids.slice(cursor);
            this.goals = this.goals.slice(cursor);
            this.reindex();
        }

        this.lastServiced = serviced;
        this.lastExpanded = expanded;
    }

    private reindex(): void {
        this.indexByEid.clear();
        for (let i = 0; i < this.eids.length; i++) {
            const eid = this.eids[i];
            if (eid !== -1) this.indexByEid.set(eid, i);
        }
    }

    /**
     * Hashes the pending queue.
     *
     * This queue is state that lives outside any component: two clients holding
     * the same units but a differently-ordered backlog will path differently a
     * few ticks later. Hashing it turns that into an immediate mismatch instead
     * of a mysterious one.
     */
    hashInto(hasher: Hasher): void {
        hasher.writeU32(this.eids.length);
        for (let i = 0; i < this.eids.length; i++) {
            hasher.writeU32(this.eids[i] >>> 0);
            hasher.writeU32(this.goals[i] >>> 0);
        }
    }
}
