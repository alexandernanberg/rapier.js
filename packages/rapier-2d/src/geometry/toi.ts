import {Vector, VectorOps} from "../math";
import {RawShapeCastHit, RawColliderShapeCastHit} from "../raw";
import {scratch} from "../scratch";
import {Collider} from "./collider";
import {ColliderSet} from "./collider_set";

/**
 * The intersection between a ray and a collider.
 */
export class ShapeCastHit {
    /**
     * The time of impact of the two shapes.
     */
    time_of_impact = 0;
    /**
     * The local-space contact point on the first shape, at
     * the time of impact.
     */
    witness1: Vector = VectorOps.zeros();
    /**
     * The local-space contact point on the second shape, at
     * the time of impact.
     */
    witness2: Vector = VectorOps.zeros();
    /**
     * The local-space normal on the first shape, at
     * the time of impact.
     */
    normal1: Vector = VectorOps.zeros();
    /**
     * The local-space normal on the second shape, at
     * the time of impact.
     */
    normal2: Vector = VectorOps.zeros();

    /**
     * Reads a shape-cast hit from its raw representation.
     *
     * @param raw - The raw hit. It is always freed before returning.
     * @param target - The object the result is written into.
     */
    public static fromRaw(
        colliderSet: ColliderSet | null,
        raw: RawShapeCastHit,
        target: ShapeCastHit,
    ): ShapeCastHit | null {
        if (!raw) return null;

        raw.getComponents();
        const s = scratch();
        raw.free();

        target.time_of_impact = s[0];
        target.witness1 = VectorOps.set(target.witness1, s[1], s[2]);
        target.witness2 = VectorOps.set(target.witness2, s[3], s[4]);
        target.normal1 = VectorOps.set(target.normal1, s[5], s[6]);
        target.normal2 = VectorOps.set(target.normal2, s[7], s[8]);
        return target;
    }
}

/**
 * The intersection between a ray and a collider.
 */
export class ColliderShapeCastHit extends ShapeCastHit {
    /**
     * The handle of the collider hit by the ray.
     */
    collider!: Collider;

    /**
     * Reads a collider shape-cast hit from its raw representation.
     *
     * @param raw - The raw hit. It is always freed before returning.
     * @param target - The object the result is written into.
     */
    public static fromRaw(
        colliderSet: ColliderSet,
        raw: RawColliderShapeCastHit,
        target: ColliderShapeCastHit,
    ): ColliderShapeCastHit | null {
        if (!raw) return null;

        const collider = colliderSet.get(raw.colliderHandle())!;
        raw.getComponents();
        const s = scratch();
        raw.free();

        target.collider = collider;
        target.time_of_impact = s[0];
        target.witness1 = VectorOps.set(target.witness1, s[1], s[2]);
        target.witness2 = VectorOps.set(target.witness2, s[3], s[4]);
        target.normal1 = VectorOps.set(target.normal1, s[5], s[6]);
        target.normal2 = VectorOps.set(target.normal2, s[7], s[8]);
        return target;
    }
}
