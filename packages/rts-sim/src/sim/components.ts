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

/** Longest path a unit can hold. Longer routes are re-planned on arrival. */
export const MAX_PATH = 48;

/** `Path.state` — a unit always has the component; this says what is in it. */
export const PathState = {
    /** No route wanted. */
    None: 0,
    /** Queued with the pathfinder, not yet serviced. */
    Pending: 1,
    /** Holding a route and following it. */
    Active: 2,
    /** The pathfinder found no route; the unit stops asking. */
    Failed: 3,
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
            {name: "goal", kind: "i32"},
            {name: "length", kind: "i32"},
            {name: "cursor", kind: "i32"},
            {name: "tiles", kind: "i32", stride: MAX_PATH, lengthField: "length"},
        ],
    },
    {
        name: "Facing",
        fields: [{name: "angle", kind: "f64"}],
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
    Path: {
        state: Uint8Array;
        goal: Int32Array;
        length: Int32Array;
        cursor: Int32Array;
        tiles: Int32Array;
    };
    Facing: {angle: Float64Array};
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
