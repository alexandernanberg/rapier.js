/**
 * The tile grid.
 *
 * Terrain is a uniform grid rather than a navmesh: an RTS map already thinks in
 * tiles, buildings occupy whole tiles, and a flat array is both the fastest
 * thing to path over and the easiest to hash.
 *
 * Weights are small integers, and every path cost is an integer multiple of
 * them. That is deliberate — integer costs make "is this path cheaper" an exact
 * comparison, so two clients can never disagree about which of two equal-looking
 * routes wins.
 */

/** Cost of a cardinal step across a weight-1 tile. */
export const CARDINAL_COST = 10;
/** Cost of a diagonal step across a weight-1 tile — 10 * sqrt(2), rounded. */
export const DIAGONAL_COST = 14;

export const IMPASSABLE = 0;
export const NORMAL = 1;

export interface TileMapOptions {
    readonly width: number;
    readonly height: number;
    /** World units per tile. */
    readonly tileSize?: number;
}

export class TileMap {
    readonly width: number;
    readonly height: number;
    readonly tileSize: number;
    /** Movement weight per tile. 0 is impassable; higher is slower going. */
    readonly weight: Uint8Array;

    /**
     * Bumped whenever a tile's weight changes. Anything derived from the
     * terrain — a cached digest, a flow field — compares against this to know
     * it has gone stale.
     */
    revision = 0;

    private cachedRevision = -1;
    private cachedHash = 0;

    constructor(options: TileMapOptions) {
        this.width = options.width;
        this.height = options.height;
        this.tileSize = options.tileSize ?? 1;
        this.weight = new Uint8Array(this.width * this.height).fill(NORMAL);
    }

    get tileCount(): number {
        return this.width * this.height;
    }

    index(tx: number, ty: number): number {
        return ty * this.width + tx;
    }

    tileX(index: number): number {
        return index % this.width;
    }

    tileY(index: number): number {
        return (index / this.width) | 0;
    }

    inBounds(tx: number, ty: number): boolean {
        return tx >= 0 && ty >= 0 && tx < this.width && ty < this.height;
    }

    isPassable(tx: number, ty: number): boolean {
        return this.inBounds(tx, ty) && this.weight[this.index(tx, ty)] !== IMPASSABLE;
    }

    isPassableIndex(index: number): boolean {
        return index >= 0 && index < this.weight.length && this.weight[index] !== IMPASSABLE;
    }

    setWeight(tx: number, ty: number, weight: number): void {
        if (!this.inBounds(tx, ty)) return;
        const index = this.index(tx, ty);
        if (this.weight[index] === weight) return;
        this.weight[index] = weight;
        this.revision++;
    }

    /** Marks a rectangle — a building footprint, a cliff, water, rough ground. */
    fillRect(tx: number, ty: number, w: number, h: number, weight: number): void {
        for (let y = ty; y < ty + h; y++) {
            for (let x = tx; x < tx + w; x++) {
                this.setWeight(x, y, weight);
            }
        }
    }

    /** `Math.floor` is exactly specified, so this is safe in sim code. */
    worldToTileX(x: number): number {
        return Math.floor(x / this.tileSize);
    }

    worldToTileY(y: number): number {
        return Math.floor(y / this.tileSize);
    }

    /** Tile index for a world position, or -1 when outside the map. */
    worldToIndex(x: number, y: number): number {
        const tx = this.worldToTileX(x);
        const ty = this.worldToTileY(y);
        return this.inBounds(tx, ty) ? this.index(tx, ty) : -1;
    }

    centerX(index: number): number {
        return (this.tileX(index) + 0.5) * this.tileSize;
    }

    centerY(index: number): number {
        return (this.tileY(index) + 0.5) * this.tileSize;
    }

    /**
     * Digest of the terrain, so a client on a modified map fails the checksum.
     *
     * Cached against a revision counter rather than recomputed: terrain changes
     * when a building goes up or a tree falls, not every tick, and walking a
     * 128x128 map per checksum would cost more than the tick it verifies.
     */
    terrainHash(): number {
        if (this.cachedRevision === this.revision) return this.cachedHash;

        let h = 0x811c9dc5;
        h = Math.imul(h ^ this.width, 0x01000193) >>> 0;
        h = Math.imul(h ^ this.height, 0x01000193) >>> 0;
        for (let i = 0; i < this.weight.length; i++) {
            h = Math.imul(h ^ this.weight[i], 0x01000193) >>> 0;
        }

        this.cachedHash = h;
        this.cachedRevision = this.revision;
        return h;
    }
}
