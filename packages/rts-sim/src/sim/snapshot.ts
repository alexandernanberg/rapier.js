import {getAllEntities, hasComponent} from "bitecs";
import {Hasher} from "../core/hash";
import {COMPONENT_SPECS, type StoreName} from "./components";
import {idOf, versionOf, type SimWorld} from "./world";

/**
 * Hashes the whole world in a canonical order.
 *
 * Deliberately *not* in query order. Query iteration order is an artefact of
 * the ECS's internal storage; sorting by raw entity id instead means the hash
 * describes the state and nothing else. Two consequences worth having: the hash
 * cannot drift because a query's insertion history differed, and it stays
 * comparable across a change of ECS library.
 *
 * Cost is O(entities x components), so compare checksums every N ticks in a
 * real match rather than every tick. Tests hash every tick, which is what makes
 * them able to name the exact tick a divergence appeared.
 */
export function hashWorld(world: SimWorld, hasher = new Hasher()): string {
    hasher.reset();
    hasher.writeU32(world.tick);
    hasher.writeU32Array(world.rng.state);

    const entities = sortedEntities(world);

    for (let i = 0; i < entities.length; i++) {
        const eid = entities[i];
        const id = idOf(world, eid);
        hasher.writeU32(id);
        hasher.writeU32(versionOf(world, eid));

        for (let c = 0; c < COMPONENT_SPECS.length; c++) {
            const spec = COMPONENT_SPECS[c];
            const store = world.stores[spec.name as StoreName] as Record<
                string,
                Float64Array | Int32Array | Uint8Array
            >;
            if (!hasComponent(world, eid, store)) continue;

            hasher.writeU32(c);
            for (const [field, kind] of spec.fields) {
                const value = store[field][id];
                if (kind === "f64") hasher.writeF64(value);
                else hasher.writeU32(value);
            }
        }
    }

    return hasher.digest();
}

export interface EntitySnapshot {
    readonly id: number;
    readonly version: number;
    readonly components: Record<string, Record<string, number>>;
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
        const components: Record<string, Record<string, number>> = {};

        for (const spec of COMPONENT_SPECS) {
            const store = world.stores[spec.name as StoreName] as Record<
                string,
                Float64Array | Int32Array | Uint8Array
            >;
            if (!hasComponent(world, eid, store)) continue;

            const fields: Record<string, number> = {};
            for (const [field] of spec.fields) fields[field] = store[field][id];
            components[spec.name] = fields;
        }

        out.push({id, version: versionOf(world, eid), components});
    }

    return out;
}

export interface Divergence {
    readonly entityId: number;
    readonly component: string;
    readonly field: string;
    readonly left: number;
    readonly right: number;
}

/** First-level explanation of why two worlds disagree. */
export function diffWorlds(left: SimWorld, right: SimWorld): Divergence[] {
    const a = describeWorld(left);
    const b = describeWorld(right);
    const byId = new Map(b.map((e) => [e.id, e]));
    const out: Divergence[] = [];

    for (const entity of a) {
        const other = byId.get(entity.id);
        if (other === undefined) {
            out.push({
                entityId: entity.id,
                component: "*",
                field: "exists",
                left: 1,
                right: 0,
            });
            continue;
        }

        for (const [name, fields] of Object.entries(entity.components)) {
            const otherFields = other.components[name];
            for (const [field, value] of Object.entries(fields)) {
                const otherValue = otherFields?.[field];
                if (otherValue === undefined || !Object.is(value, otherValue)) {
                    out.push({
                        entityId: entity.id,
                        component: name,
                        field,
                        left: value,
                        right: otherValue ?? NaN,
                    });
                }
            }
        }
    }

    return out;
}

/** Live entities, ordered by raw id — the canonical iteration order. */
function sortedEntities(world: SimWorld): number[] {
    const entities = getAllEntities(world).slice();
    entities.sort((x, y) => idOf(world, x) - idOf(world, y));
    return entities;
}
