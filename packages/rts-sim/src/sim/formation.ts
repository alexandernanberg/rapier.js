/**
 * Formation shapes.
 *
 * A shape is a pure function from a member count to a set of slot offsets in
 * *formation-local* space, where +x is the direction of travel and +y is to its
 * right. Movement rotates those offsets by the leader's facing, so a shape is
 * written once and works at any heading.
 *
 * Adding a shape is a value here plus a branch in `layoutSlots`. Two exist so
 * the seam is exercised rather than merely claimed; the others a real RTS wants
 * — wedge, column, staggered ranks, a hollow box for siege — are the same kind
 * of function and need nothing else to change.
 */
export const FormationShape = {
    /** Rows across the direction of travel, centred on the leader. */
    Block: 0,
    /** A single rank abreast, centred on the leader. */
    Line: 1,
} as const;

export type FormationShapeValue = (typeof FormationShape)[keyof typeof FormationShape];

/** Largest member count a shape will lay out. Beyond this, slots repeat. */
export const MAX_FORMATION = 512;

/**
 * Writes `count` slot offsets into `out` as x,y pairs, in formation-local space.
 *
 * Slots are returned in assignment order: callers hand them to members sorted by
 * handle, so the layout never depends on selection order.
 */
export function layoutSlots(
    shape: number,
    count: number,
    spacing: number,
    out: Float64Array,
): void {
    switch (shape) {
        case FormationShape.Line:
            layoutLine(count, spacing, out);
            return;
        case FormationShape.Block:
        default:
            layoutBlock(count, spacing, out);
            return;
    }
}

/**
 * A square-ish block, centred on the leader.
 *
 * Columns run across the direction of travel and rows along it, so a marching
 * block is as wide as it is deep rather than a long thin queue.
 */
function layoutBlock(count: number, spacing: number, out: Float64Array): void {
    const columns = Math.max(1, Math.ceil(Math.sqrt(count)));
    const rows = Math.ceil(count / columns);

    for (let i = 0; i < count; i++) {
        const column = i % columns;
        const row = (i / columns) | 0;
        // Local +x is forward, so rows step along x and columns across y.
        out[i * 2] = (row - (rows - 1) / 2) * spacing;
        out[i * 2 + 1] = (column - (columns - 1) / 2) * spacing;
    }
}

/** One rank abreast, centred on the leader. */
function layoutLine(count: number, spacing: number, out: Float64Array): void {
    for (let i = 0; i < count; i++) {
        out[i * 2] = 0;
        out[i * 2 + 1] = (i - (count - 1) / 2) * spacing;
    }
}
