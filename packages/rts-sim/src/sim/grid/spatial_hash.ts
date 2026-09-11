/**
 * Uniform-grid spatial index for neighbour queries.
 *
 * A quadtree would be the wrong shape here: units are roughly evenly spread
 * over a bounded map and the map is already a grid, so a flat bucket array
 * beats a tree on both lookup and rebuild cost.
 *
 * Rebuilt from scratch each tick by counting sort — count per cell, prefix sum,
 * scatter — which touches only preallocated arrays. Incremental maintenance
 * would be cheaper in theory and much easier to get subtly wrong.
 */

export class SpatialHash {
    readonly cellSize: number;
    readonly cols: number;
    readonly rows: number;

    private readonly cellStart: Int32Array;
    private readonly cellFill: Int32Array;
    private readonly items: Int32Array;
    private readonly itemX: Float64Array;
    private readonly itemY: Float64Array;
    private count = 0;

    constructor(width: number, height: number, cellSize: number, capacity: number) {
        this.cellSize = cellSize;
        this.cols = Math.max(1, Math.ceil(width / cellSize));
        this.rows = Math.max(1, Math.ceil(height / cellSize));
        const cells = this.cols * this.rows;

        this.cellStart = new Int32Array(cells + 1);
        this.cellFill = new Int32Array(cells);
        this.items = new Int32Array(capacity);
        this.itemX = new Float64Array(capacity);
        this.itemY = new Float64Array(capacity);
    }

    private cellOf(x: number, y: number): number {
        let cx = Math.floor(x / this.cellSize);
        let cy = Math.floor(y / this.cellSize);
        cx = cx < 0 ? 0 : cx >= this.cols ? this.cols - 1 : cx;
        cy = cy < 0 ? 0 : cy >= this.rows ? this.rows - 1 : cy;
        return cy * this.cols + cx;
    }

    /**
     * Rebuilds the index from a list of entities and their positions.
     *
     * `entities` is expected in the caller's iteration order, and that order is
     * preserved within each cell — so neighbour lists come back in a stable
     * order, which is what keeps the separation pass reproducible.
     */
    build(
        entities: ArrayLike<number>,
        rawIds: Int32Array,
        count: number,
        xs: Float64Array,
        ys: Float64Array,
    ): void {
        const {cellStart, cellFill, items, itemX, itemY} = this;
        const cells = cellFill.length;

        cellStart.fill(0);
        cellFill.fill(0);
        this.count = count;

        // Pass 1: how many land in each cell.
        for (let i = 0; i < count; i++) {
            const id = rawIds[i];
            cellStart[this.cellOf(xs[id], ys[id]) + 1]++;
        }

        // Pass 2: prefix sum to cell offsets.
        for (let c = 0; c < cells; c++) {
            cellStart[c + 1] += cellStart[c];
        }

        // Pass 3: scatter, preserving input order inside each cell.
        for (let i = 0; i < count; i++) {
            const eid = entities[i];
            const id = rawIds[i];
            const x = xs[id];
            const y = ys[id];
            const cell = this.cellOf(x, y);
            const slot = cellStart[cell] + cellFill[cell]++;
            items[slot] = eid;
            itemX[slot] = x;
            itemY[slot] = y;
        }
    }

    /**
     * Writes entities within `radius` of (x, y) into `out`, returning how many.
     *
     * Stops at `out.length`. A truncated neighbour list is deterministic — the
     * cells are visited in a fixed order — but it does mean a unit in an
     * extreme pile-up sees only its first N neighbours.
     */
    collect(x: number, y: number, radius: number, out: Int32Array): number {
        const {cellStart, items, itemX, itemY} = this;
        const minCx = clampIndex(Math.floor((x - radius) / this.cellSize), this.cols);
        const maxCx = clampIndex(Math.floor((x + radius) / this.cellSize), this.cols);
        const minCy = clampIndex(Math.floor((y - radius) / this.cellSize), this.rows);
        const maxCy = clampIndex(Math.floor((y + radius) / this.cellSize), this.rows);
        const r2 = radius * radius;

        let found = 0;
        for (let cy = minCy; cy <= maxCy; cy++) {
            for (let cx = minCx; cx <= maxCx; cx++) {
                const cell = cy * this.cols + cx;
                const end = cellStart[cell + 1];
                for (let s = cellStart[cell]; s < end; s++) {
                    const dx = itemX[s] - x;
                    const dy = itemY[s] - y;
                    if (dx * dx + dy * dy > r2) continue;
                    if (found >= out.length) return found;
                    out[found++] = items[s];
                }
            }
        }
        return found;
    }

    get size(): number {
        return this.count;
    }
}

function clampIndex(v: number, limit: number): number {
    return v < 0 ? 0 : v >= limit ? limit - 1 : v;
}
