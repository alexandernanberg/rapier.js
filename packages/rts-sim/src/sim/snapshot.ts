import {getAllEntities, hasComponent} from "bitecs";
import {Hasher} from "../core/hash";
import {SPECS, type FieldSpec, type StoreName, type TypedArray} from "./components";
import {idOf, versionOf, type SimWorld} from "./world";

type RawStore = Record<string, TypedArray>;

/**
 * Hashes the whole world in a canonical order.
 *
 * Deliberately *not* in query order. Query iteration order is an artefact of
 * the ECS's internal storage; sorting by raw entity id instead means the hash
 * describes the state and nothing else. Two consequences worth having: the hash
 * cannot drift because a query's insertion history differed, and it stays
 * comparable across a change of ECS library.
 *
 * Everything stateful has to be in here. Components come from
 * `COMPONENT_SPECS`; the tick, the rng, the terrain and the pathfinder's
 * pending queue are hashed explicitly because they live outside the component
 * stores.
 *
 * Cost is O(entities x components), so compare checksums every N ticks in a
 * real match rather than every tick. Tests hash every tick, which is what lets
 * them name the exact tick a divergence appeared.
 */
export function hashWorld(world: SimWorld, hasher = new Hasher()): string {
    hasher.reset();
    hasher.writeU32(world.tick);
    hasher.writeU32Array(world.rng.state);
    hasher.writeU32(world.map.terrainHash());
    // The pathfinder's pending queue and its flow-field cache both live outside
    // the component stores, and both change how a unit gets routed.
    world.paths.hashInto(hasher);

    const entities = sortedEntities(world);

    for (let i = 0; i < entities.length; i++) {
        const eid = entities[i];
        const id = idOf(world, eid);
        hasher.writeU32(id);
        hasher.writeU32(versionOf(world, eid));

        for (let c = 0; c < SPECS.length; c++) {
            const spec = SPECS[c];
            const store = world.stores[spec.name as StoreName] as RawStore;
            if (!hasComponent(world, eid, store)) continue;

            hasher.writeU32(c);
            for (const field of spec.fields) {
                const array = store[field.name];
                if (field.stride === undefined) {
                    writeValue(hasher, field, array[id]);
                    continue;
                }

                const valid = validLength(store, field, id);
                const base = id * field.stride;
                hasher.writeU32(valid);
                for (let k = 0; k < valid; k++) {
                    writeValue(hasher, field, array[base + k]);
                }
            }
        }
    }

    return hasher.digest();
}

function writeValue(hasher: Hasher, field: FieldSpec, value: number): void {
    if (field.kind === "f64") hasher.writeF64(value);
    else hasher.writeU32(value);
}

/**
 * How many slots of an array field are meaningful.
 *
 * Clamped to the stride: a length field that has gone out of range is a bug
 * elsewhere, and reading past the entity's slice would silently mix in its
 * neighbour's data.
 */
function validLength(store: RawStore, field: FieldSpec, id: number): number {
    const stride = field.stride ?? 1;
    if (field.lengthField === undefined) return stride;
    const raw = store[field.lengthField][id];
    return raw < 0 ? 0 : raw > stride ? stride : raw;
}

export interface EntitySnapshot {
    readonly id: number;
    readonly version: number;
    readonly components: Record<string, Record<string, number | number[]>>;
}

/**
 * Readable dump of the world, in the same canonical order as the hash.
 *
 * This is the debugging counterpart to `hashWorld`: when a replay diverges the
 * hash tells you *which tick*, and diffing two of these tells you which entity
 * and which field. Do not call it on the hot path.
 */
export function describeWorld(world: SimWorld): EntitySnapshot[] {
    const out: EntitySnapshot[] = [];

    for (const eid of sortedEntities(world)) {
        const id = idOf(world, eid);
        const components: Record<string, Record<string, number | number[]>> = {};

        for (const spec of SPECS) {
            const store = world.stores[spec.name as StoreName] as RawStore;
            if (!hasComponent(world, eid, store)) continue;

            const fields: Record<string, number | number[]> = {};
            for (const field of spec.fields) {
                const array = store[field.name];
                if (field.stride === undefined) {
                    fields[field.name] = array[id];
                    continue;
                }
                const valid = validLength(store, field, id);
                const base = id * field.stride;
                fields[field.name] = Array.from(array.subarray(base, base + valid));
            }
            components[spec.name] = fields;
        }

        out.push({id, version: versionOf(world, eid), components});
    }

    return out;
}

export interface Divergence {
    readonly entityId: number;
    readonly component: string;
    /** Array fields are reported per slot, as `tiles[3]`. */
    readonly field: string;
    readonly left: number;
    readonly right: number;
}

/** First-level explanation of why two worlds disagree. */
export function diffWorlds(left: SimWorld, right: SimWorld): Divergence[] {
    const a = describeWorld(left);
    const b = describeWorld(right);
    const byId = new Map(b.map((entity) => [entity.id, entity]));
    const out: Divergence[] = [];

    for (const entity of a) {
        const other = byId.get(entity.id);
        if (other === undefined) {
            out.push({entityId: entity.id, component: "*", field: "exists", left: 1, right: 0});
            continue;
        }

        for (const [name, fields] of Object.entries(entity.components)) {
            const otherFields = other.components[name];
            for (const [field, value] of Object.entries(fields)) {
                compareField(out, entity.id, name, field, value, otherFields?.[field]);
            }
        }
    }

    return out;
}

function compareField(
    out: Divergence[],
    entityId: number,
    component: string,
    field: string,
    left: number | number[] | undefined,
    right: number | number[] | undefined,
): void {
    if (Array.isArray(left) || Array.isArray(right)) {
        const a = Array.isArray(left) ? left : [];
        const b = Array.isArray(right) ? right : [];
        const len = Math.max(a.length, b.length);
        for (let i = 0; i < len; i++) {
            if (!Object.is(a[i], b[i])) {
                out.push({
                    entityId,
                    component,
                    field: `${field}[${i}]`,
                    left: a[i] ?? NaN,
                    right: b[i] ?? NaN,
                });
            }
        }
        return;
    }

    if (!Object.is(left, right)) {
        out.push({entityId, component, field, left: left ?? NaN, right: right ?? NaN});
    }
}

/** Live entities, ordered by raw id — the canonical iteration order. */
function sortedEntities(world: SimWorld): number[] {
    const entities = getAllEntities(world).slice();
    entities.sort((x, y) => idOf(world, x) - idOf(world, y));
    return entities;
}
