import {describe, expect, it} from "vitest";
import {
    COMPONENT_SPECS,
    createStores,
    SPECS,
    type FieldKind,
    type StoreName,
    type Stores,
} from "../sim/components";

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
type SpecFields<N extends SpecNames> = SpecFor<N>["fields"][number]["name"];
type _PositionFields = Assert<Equal<SpecFields<"Position">, keyof Stores["Position"]>>;
type _HealthFields = Assert<Equal<SpecFields<"Health">, keyof Stores["Health"]>>;
type _OwnerFields = Assert<Equal<SpecFields<"Owner">, keyof Stores["Owner"]>>;
type _PathFields = Assert<Equal<SpecFields<"Path">, keyof Stores["Path"]>>;

describe("component specs", () => {
    it("declares each component exactly once", () => {
        const names = COMPONENT_SPECS.map((s) => s.name);
        expect(new Set(names).size).toBe(names.length);
    });

    it("declares each field of a component exactly once", () => {
        for (const spec of SPECS) {
            const fields = spec.fields.map((field) => field.name);
            expect(new Set(fields).size, spec.name).toBe(fields.length);
        }
    });

    it("points every lengthField at a scalar field of the same component", () => {
        for (const spec of SPECS) {
            for (const field of spec.fields) {
                if (field.lengthField === undefined) continue;
                const target = spec.fields.find((f) => f.name === field.lengthField);
                expect(target, `${spec.name}.${field.name}`).toBeDefined();
                expect(target!.stride, `${spec.name}.${field.lengthField}`).toBeUndefined();
            }
        }
    });

    it("allocates a backing array of the declared type for every field", () => {
        const capacity = 64;
        const stores = createStores(capacity) as unknown as Record<
            string,
            Record<string, Float64Array | Int32Array | Uint8Array>
        >;

        expect(Object.keys(stores).sort()).toEqual(SPECS.map((spec) => spec.name).sort());

        const constructors: Record<FieldKind, unknown> = {
            f64: Float64Array,
            i32: Int32Array,
            u8: Uint8Array,
        };

        for (const spec of SPECS) {
            const store = stores[spec.name];
            expect(Object.keys(store).sort(), spec.name).toEqual(
                spec.fields.map((field) => field.name).sort(),
            );

            for (const field of spec.fields) {
                const array = store[field.name];
                const label = `${spec.name}.${field.name}`;
                expect(array, label).toBeInstanceOf(constructors[field.kind] as never);
                expect(array.length, label).toBe(capacity * (field.stride ?? 1));
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
