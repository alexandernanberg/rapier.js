/**
 * Component schemas. One declaration drives storage layout, and later would also
 * drive the editor inspector, serialization, and network deltas.
 */

export type FieldType = "f32" | "u32";

export const CTOR = {
    f32: Float32Array,
    u32: Uint32Array,
} as const;

export interface ComponentDef {
    id: number;
    name: string;
    fields: ReadonlyArray<readonly [string, FieldType]>;
}

let nextId = 0;

export function component(name: string, fields: Record<string, FieldType>): ComponentDef {
    return {
        id: nextId++,
        name,
        fields: Object.entries(fields) as Array<[string, FieldType]>,
    };
}

/** A component carrying no data — presence is the whole signal. */
export function tag(name: string): ComponentDef {
    return {id: nextId++, name, fields: []};
}

export const vec3 = {x: "f32", y: "f32", z: "f32"} as const;
