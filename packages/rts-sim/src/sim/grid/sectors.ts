/**
 * Sector partitioning of the tile grid.
 *
 * Sectors are what bound the cost of everything above the raw grid: the portal
 * graph has one node cluster per sector, and a flow segment integrates a couple
 * of sectors rather than the whole map. On a 128x128 map with 16-cell sectors
 * that is 512 cells per segment instead of 16 384 — the difference between
 * 0.1ms and 4.4ms of integration.
 */
export class SectorLayout {
    readonly sectorSize: number;
    readonly cols: number;
    readonly rows: number;
    readonly width: number;
    readonly height: number;

    constructor(width: number, height: number, sectorSize: number) {
        this.width = width;
        this.height = height;
        this.sectorSize = sectorSize;
        this.cols = Math.ceil(width / sectorSize);
        this.rows = Math.ceil(height / sectorSize);
    }

    get count(): number {
        return this.cols * this.rows;
    }

    sectorX(sector: number): number {
        return sector % this.cols;
    }

    sectorY(sector: number): number {
        return (sector / this.cols) | 0;
    }

    sectorIndex(sx: number, sy: number): number {
        return sy * this.cols + sx;
    }

    /** Sector containing a tile, by tile coordinates. */
    sectorOfTile(tx: number, ty: number): number {
        return this.sectorIndex((tx / this.sectorSize) | 0, (ty / this.sectorSize) | 0);
    }

    originX(sector: number): number {
        return this.sectorX(sector) * this.sectorSize;
    }

    originY(sector: number): number {
        return this.sectorY(sector) * this.sectorSize;
    }

    /** Exclusive upper bound, clamped to the map for a ragged final sector. */
    endX(sector: number): number {
        return Math.min(this.originX(sector) + this.sectorSize, this.width);
    }

    endY(sector: number): number {
        return Math.min(this.originY(sector) + this.sectorSize, this.height);
    }

    inSector(sector: number, tx: number, ty: number): boolean {
        return (
            tx >= this.originX(sector) &&
            ty >= this.originY(sector) &&
            tx < this.endX(sector) &&
            ty < this.endY(sector)
        );
    }
}
