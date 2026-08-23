/**
 * Components shared across subsystems.
 *
 * `Transform` is the one every backend reads and physics writes. It carries a
 * full TRS because the renderer needs scale and physics does not — keeping them
 * in one component avoids a second query and a second archetype split.
 */

import {component, tag} from "./schema.ts";

export const Transform = component("Transform", {
    tx: "f32",
    ty: "f32",
    tz: "f32",
    rx: "f32",
    ry: "f32",
    rz: "f32",
    rw: "f32",
    sx: "f32",
    sy: "f32",
    sz: "f32",
});

/**
 * What to draw. Both fields are opaque ids owned by the render backend —
 * never an Object3D, never a GPUBuffer. This is the whole reason a backend can
 * be swapped: gameplay never learns what a mesh actually is.
 */
export const Renderable = component("Renderable", {
    mesh: "u32",
    material: "u32",
});

/** Physics handle. Gameplay must not write Transform on entities that have this. */
export const RigidBodyRef = component("RigidBodyRef", {handle: "u32"});

/** Excluded from extraction. A tag, so hiding is an archetype move, not a branch. */
export const Hidden = tag("Hidden");

export function identityTransform(): Record<string, number> {
    return {tx: 0, ty: 0, tz: 0, rx: 0, ry: 0, rz: 0, rw: 1, sx: 1, sy: 1, sz: 1};
}
