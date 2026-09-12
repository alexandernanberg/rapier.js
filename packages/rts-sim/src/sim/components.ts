/**
 * Component stores.
 *
 * We own every backing array — bitECS 0.4 only tracks which entity has which
 * component, it never touches the layout. That is the whole reason to use it:
 * snapshot hashing and the eventual worker handoff are just reads of arrays we
 * allocated.
 *
 * Stores are created per-world rather than at module scope so two simulations
 * can run side by side in one process, which is what replay verification does.
 *
 * Arrays are indexed by the *raw* entity id, not the entity handle — see
 * `idOf` in `world.ts`. bitECS packs a generation counter into the handle.
 */

export type FieldKind = "f64" | "i32" | "u8";

export interface FieldSpec {
    readonly name: string;
    readonly kind: FieldKind;
    /**
     * Slots per entity. Omit for a scalar. An array field is laid out
     * contiguously, so entity `id` owns `[id * stride, (id + 1) * stride)`.
     */
    readonly stride?: number;
    /**
     * Scalar field on the same component holding how many of this field's
     * slots are meaningful. Hashing reads only that prefix, so the unused tail
     * of a path buffer cannot produce a phantom desync — and hashing a path
     * costs its length rather than its capacity.
     */
    readonly lengthField?: string;
}

export interface ComponentSpec {
    readonly name: string;
    readonly fields: readonly FieldSpec[];
}

/**
 * `Path.state` — a unit always has the component; this says what is in it.
 *
 * There are no waypoints. A unit holds the *sector* whose flow segment it is
 * following, and reads a direction from that segment at whatever tile it
 * currently stands on. Nothing to truncate, nothing to re-plan, and a unit
 * shoved aside by separation recovers for free.
 */
export const PathState = {
    /** No segment. The unit walks straight at its order position meanwhile. */
    None: 0,
    /** Following the segment named by `Path.sector`. */
    Flow: 1,
    /** No route from here; the unit stops asking. */
    Failed: 2,
} as const;

export type PathStateValue = (typeof PathState)[keyof typeof PathState];

/**
 * The single source of truth for what counts as simulation state.
 *
 * Snapshotting and hashing are driven from this list, so a component that is
 * missing here is invisible to desync detection — a silent, months-later class
 * of bug. `components.test.ts` fails the typecheck if `Stores` and this list
 * drift apart.
 *
 * State that does not live in a component — the tick, the rng, the pathfinder's
 * pending queue — is hashed explicitly in `snapshot.ts`. Anything stateful that
 * appears in neither place is invisible.
 */
export const COMPONENT_SPECS = [
    {
        name: "Position",
        fields: [
            {name: "x", kind: "f64"},
            {name: "y", kind: "f64"},
        ],
    },
    {
        name: "Velocity",
        fields: [
            {name: "x", kind: "f64"},
            {name: "y", kind: "f64"},
        ],
    },
    {
        name: "MoveTarget",
        fields: [
            {name: "x", kind: "f64"},
            {name: "y", kind: "f64"},
        ],
    },
    {
        name: "Path",
        fields: [
            {name: "state", kind: "u8"},
            /** Destination tile. */
            {name: "goal", kind: "i32"},
            /** Sector whose flow segment this unit follows; -1 for none. */
            {name: "sector", kind: "i32"},
        ],
    },
    {
        name: "Facing",
        fields: [{name: "angle", kind: "f64"}],
    },
    {
        /**
         * Offset from the order position this unit should end up at, so a group
         * ordered to one point arrives as a block instead of fighting over a
         * single tile. Zero means no formation.
         *
         * Deliberately an offset rather than its own destination: the whole
         * group keeps one flow goal, so they still share one segment per sector.
         * Giving each unit its own goal tile would turn one integration into
         * forty.
         */
        name: "Formation",
        fields: [
            {name: "offsetX", kind: "f64"},
            {name: "offsetY", kind: "f64"},
        ],
    },
    {
        name: "Speed",
        fields: [{name: "value", kind: "f64"}],
    },
    {
        name: "Radius",
        fields: [{name: "value", kind: "f64"}],
    },
    {
        name: "Health",
        fields: [
            {name: "current", kind: "i32"},
            {name: "max", kind: "i32"},
        ],
    },
    {
        name: "Owner",
        fields: [{name: "player", kind: "u8"}],
    },
    {
        name: "UnitKind",
        fields: [{name: "kind", kind: "u8"}],
    },
] as const satisfies readonly ComponentSpec[];

/**
 * Widened view of `COMPONENT_SPECS` for runtime iteration.
 *
 * The `as const` form above keeps the literal field names that the type-level
 * drift guard in `components.test.ts` needs, but it also narrows each entry to
 * exactly the keys it was written with — so `spec.fields[i].stride` is not even
 * a property on a scalar field's type. This alias restores the declared shape.
 */
export const SPECS: readonly ComponentSpec[] = COMPONENT_SPECS;

export interface Stores {
    Position: {x: Float64Array; y: Float64Array};
    Velocity: {x: Float64Array; y: Float64Array};
    MoveTarget: {x: Float64Array; y: Float64Array};
    Path: {state: Uint8Array; goal: Int32Array; sector: Int32Array};
    Facing: {angle: Float64Array};
    Formation: {offsetX: Float64Array; offsetY: Float64Array};
    Speed: {value: Float64Array};
    Radius: {value: Float64Array};
    Health: {current: Int32Array; max: Int32Array};
    Owner: {player: Uint8Array};
    UnitKind: {kind: Uint8Array};
}

export type StoreName = keyof Stores;

export type TypedArray = Float64Array | Int32Array | Uint8Array;

/**
 * `capacity` bounds concurrent entities, not total spawns — bitECS recycles
 * ids, so the raw index stays dense at the high-water mark.
 */
export function createStores(capacity: number): Stores {
    const stores = {} as Record<string, Record<string, TypedArray>>;
    for (const spec of SPECS) {
        const store: Record<string, TypedArray> = {};
        for (const field of spec.fields) {
            store[field.name] = allocate(field.kind, capacity * (field.stride ?? 1));
        }
        stores[spec.name] = store;
    }
    return stores as unknown as Stores;
}

function allocate(kind: FieldKind, length: number): TypedArray {
    switch (kind) {
        case "f64":
            return new Float64Array(length);
        case "i32":
            return new Int32Array(length);
        case "u8":
            return new Uint8Array(length);
    }
}

/** Unit archetypes. Content tuning belongs in data, not here — this is a stub. */
export const UnitKindId = {
    Villager: 0,
    Militia: 1,
    Archer: 2,
} as const;

export type UnitKindIdValue = (typeof UnitKindId)[keyof typeof UnitKindId];

export interface UnitStats {
    readonly speed: number;
    readonly health: number;
    readonly radius: number;
}

export const UNIT_STATS: Readonly<Record<number, UnitStats>> = {
    [UnitKindId.Villager]: {speed: 2.6, health: 40, radius: 0.35},
    [UnitKindId.Militia]: {speed: 3.4, health: 60, radius: 0.4},
    [UnitKindId.Archer]: {speed: 3.0, health: 35, radius: 0.35},
};
