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
    time_of_impact: number;
    /**
     * The local-space contact point on the first shape, at
     * the time of impact.
     */
    witness1: Vector;
    /**
     * The local-space contact point on the second shape, at
     * the time of impact.
     */
    witness2: Vector;
    /**
     * The local-space normal on the first shape, at
     * the time of impact.
     */
    normal1: Vector;
    /**
     * The local-space normal on the second shape, at
     * the time of impact.
     */
    normal2: Vector;

    constructor(
        time_of_impact: number,
        witness1: Vector,
        witness2: Vector,
        normal1: Vector,
        normal2: Vector,
    ) {
        this.time_of_impact = time_of_impact;
        this.witness1 = witness1;
        this.witness2 = witness2;
        this.normal1 = normal1;
        this.normal2 = normal2;
    }

    /**
     * Reads a shape-cast hit from its raw representation.
     *
     * @param raw - The raw hit. It is always freed before returning.
     * @param target - Optional target object to write the result to (avoids allocation).
     */
    public static fromRaw(
        colliderSet: ColliderSet | null,
        raw: RawShapeCastHit,
        target?: ShapeCastHit,
    ): ShapeCastHit | null {
        if (!raw) return null;

        raw.getComponents();
        const s = scratch();
        raw.free();

        const result =
            target ??
            new ShapeCastHit(
                0,
                VectorOps.zeros(),
                VectorOps.zeros(),
                VectorOps.zeros(),
                VectorOps.zeros(),
            );

        result.time_of_impact = s[0];
        result.witness1 = VectorOps.set(result.witness1, s[1], s[2], s[3]);
        result.witness2 = VectorOps.set(result.witness2, s[4], s[5], s[6]);
        result.normal1 = VectorOps.set(result.normal1, s[7], s[8], s[9]);
        result.normal2 = VectorOps.set(result.normal2, s[10], s[11], s[12]);
        return result;
    }
}

/**
 * The intersection between a ray and a collider.
 */
export class ColliderShapeCastHit extends ShapeCastHit {
    /**
     * The handle of the collider hit by the ray.
     */
    collider: Collider;

    constructor(
        collider: Collider,
        time_of_impact: number,
        witness1: Vector,
        witness2: Vector,
        normal1: Vector,
        normal2: Vector,
    ) {
        super(time_of_impact, witness1, witness2, normal1, normal2);
        this.collider = collider;
    }

    /**
     * Reads a collider shape-cast hit from its raw representation.
     *
     * @param raw - The raw hit. It is always freed before returning.
     * @param target - Optional target object to write the result to (avoids allocation).
     */
    public static fromRaw(
        colliderSet: ColliderSet,
        raw: RawColliderShapeCastHit,
        target?: ColliderShapeCastHit,
    ): ColliderShapeCastHit | null {
        if (!raw) return null;

        const collider = colliderSet.get(raw.colliderHandle())!;
        raw.getComponents();
        const s = scratch();
        raw.free();

        const result =
            target ??
            new ColliderShapeCastHit(
                collider,
                0,
                VectorOps.zeros(),
                VectorOps.zeros(),
                VectorOps.zeros(),
                VectorOps.zeros(),
            );

        result.collider = collider;
        result.time_of_impact = s[0];
        result.witness1 = VectorOps.set(result.witness1, s[1], s[2], s[3]);
        result.witness2 = VectorOps.set(result.witness2, s[4], s[5], s[6]);
        result.normal1 = VectorOps.set(result.normal1, s[7], s[8], s[9]);
        result.normal2 = VectorOps.set(result.normal2, s[10], s[11], s[12]);
        return result;
    }
}
