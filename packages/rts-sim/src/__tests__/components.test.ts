import {describe, expect, it} from "vitest";
import {COMPONENT_SPECS, createStores, type StoreName, type Stores} from "../sim/components";

/**
 * Compile-time guard against the two declarations drifting apart.
 *
 * `COMPONENT_SPECS` drives hashing and snapshotting; `Stores` is what systems
 * actually read. A component present in one and not the other is state that
 * silently escapes desync detection, so `tsc --noEmit` must reject it. These
 * types are unused at runtime by design.
 */
type Equal<A, B> =
    (<T>() => T extends A ? 1 : 2) extends <T>() => T extends B ? 1 : 2 ? true : false;
type Assert<T extends true> = T;

type SpecNames = (typeof COMPONENT_SPECS)[number]["name"];
type _NamesMatch = Assert<Equal<SpecNames, StoreName>>;

type SpecFor<N extends SpecNames> = Extract<(typeof COMPONENT_SPECS)[number], {name: N}>;
type SpecFields<N extends SpecNames> = SpecFor<N>["fields"][number][0];
type _PositionFields = Assert<Equal<SpecFields<"Position">, keyof Stores["Position"]>>;
type _HealthFields = Assert<Equal<SpecFields<"Health">, keyof Stores["Health"]>>;
type _OwnerFields = Assert<Equal<SpecFields<"Owner">, keyof Stores["Owner"]>>;

describe("component specs", () => {
    it("declares each component exactly once", () => {
        const names = COMPONENT_SPECS.map((s) => s.name);
        expect(new Set(names).size).toBe(names.length);
    });

    it("declares each field of a component exactly once", () => {
        for (const spec of COMPONENT_SPECS) {
            const fields = spec.fields.map(([name]) => name);
            expect(new Set(fields).size, spec.name).toBe(fields.length);
        }
    });

    it("allocates a backing array of the declared type for every field", () => {
        const capacity = 64;
        const stores = createStores(capacity) as unknown as Record<
            string,
            Record<string, Float64Array | Int32Array | Uint8Array>
        >;

        expect(Object.keys(stores).sort()).toEqual(COMPONENT_SPECS.map((s) => s.name).sort());

        const constructors = {f64: Float64Array, i32: Int32Array, u8: Uint8Array} as const;

        for (const spec of COMPONENT_SPECS) {
            const store = stores[spec.name];
            expect(Object.keys(store).sort(), spec.name).toEqual(
                spec.fields.map(([name]) => name).sort(),
            );

            for (const [field, kind] of spec.fields) {
                expect(store[field], `${spec.name}.${field}`).toBeInstanceOf(constructors[kind]);
                expect(store[field].length, `${spec.name}.${field}`).toBe(capacity);
            }
        }
    });

    it("gives each world its own arrays, so two sims cannot alias", () => {
        const a = createStores(8);
        const b = createStores(8);

        a.Position.x[0] = 42;
        expect(b.Position.x[0]).toBe(0);
    });
});
