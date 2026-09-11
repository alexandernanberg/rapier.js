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

export interface ComponentSpec {
    readonly name: string;
    readonly fields: readonly (readonly [name: string, kind: FieldKind])[];
}

/**
 * The single source of truth for what counts as simulation state.
 *
 * Snapshotting and hashing are driven from this list, so a component that is
 * missing here is invisible to desync detection — a silent, months-later class
 * of bug. `components.test.ts` fails if `Stores` and this list drift apart.
 */
export const COMPONENT_SPECS = [
    {
        name: "Position",
        fields: [
            ["x", "f64"],
            ["y", "f64"],
        ],
    },
    {
        name: "Velocity",
        fields: [
            ["x", "f64"],
            ["y", "f64"],
        ],
    },
    {
        name: "MoveTarget",
        fields: [
            ["x", "f64"],
            ["y", "f64"],
        ],
    },
    {name: "Facing", fields: [["angle", "f64"]]},
    {name: "Speed", fields: [["value", "f64"]]},
    {
        name: "Health",
        fields: [
            ["current", "i32"],
            ["max", "i32"],
        ],
    },
    {name: "Owner", fields: [["player", "u8"]]},
    {name: "UnitKind", fields: [["kind", "u8"]]},
] as const satisfies readonly ComponentSpec[];

export interface Stores {
    Position: {x: Float64Array; y: Float64Array};
    Velocity: {x: Float64Array; y: Float64Array};
    MoveTarget: {x: Float64Array; y: Float64Array};
    Facing: {angle: Float64Array};
    Speed: {value: Float64Array};
    Health: {current: Int32Array; max: Int32Array};
    Owner: {player: Uint8Array};
    UnitKind: {kind: Uint8Array};
}

export type StoreName = keyof Stores;

/**
 * `capacity` bounds concurrent entities, not total spawns — bitECS recycles
 * ids, so the raw index stays dense at the high-water mark.
 */
export function createStores(capacity: number): Stores {
    const stores = {} as Record<string, Record<string, Float64Array | Int32Array | Uint8Array>>;
    for (const spec of COMPONENT_SPECS) {
        const store: Record<string, Float64Array | Int32Array | Uint8Array> = {};
        for (const [field, kind] of spec.fields) {
            store[field] = allocate(kind, capacity);
        }
        stores[spec.name] = store;
    }
    return stores as unknown as Stores;
}

function allocate(kind: FieldKind, capacity: number): Float64Array | Int32Array | Uint8Array {
    switch (kind) {
        case "f64":
            return new Float64Array(capacity);
        case "i32":
            return new Int32Array(capacity);
        case "u8":
            return new Uint8Array(capacity);
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
}

export const UNIT_STATS: Readonly<Record<number, UnitStats>> = {
    [UnitKindId.Villager]: {speed: 2.6, health: 40},
    [UnitKindId.Militia]: {speed: 3.4, health: 60},
    [UnitKindId.Archer]: {speed: 3.0, health: 35},
};
