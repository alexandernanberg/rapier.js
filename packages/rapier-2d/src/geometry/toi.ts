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

    public static fromRaw(
        colliderSet: ColliderSet | null,
        raw: RawShapeCastHit,
    ): ShapeCastHit | null {
        if (!raw) return null;

        raw.getComponents();
        const s = scratch();
        raw.free();

        const result = new ShapeCastHit(
            s[0],
            VectorOps.zeros(),
            VectorOps.zeros(),
            VectorOps.zeros(),
            VectorOps.zeros(),
        );
        result.witness1.x = s[1];
        result.witness1.y = s[2];
        result.witness2.x = s[3];
        result.witness2.y = s[4];
        result.normal1.x = s[5];
        result.normal1.y = s[6];
        result.normal2.x = s[7];
        result.normal2.y = s[8];
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

    public static fromRaw(
        colliderSet: ColliderSet,
        raw: RawColliderShapeCastHit,
    ): ColliderShapeCastHit | null {
        if (!raw) return null;

        const collider = colliderSet.get(raw.colliderHandle())!;
        raw.getComponents();
        const s = scratch();
        raw.free();

        const result = new ColliderShapeCastHit(
            collider,
            s[0],
            VectorOps.zeros(),
            VectorOps.zeros(),
            VectorOps.zeros(),
            VectorOps.zeros(),
        );
        result.witness1.x = s[1];
        result.witness1.y = s[2];
        result.witness2.x = s[3];
        result.witness2.y = s[4];
        result.normal1.x = s[5];
        result.normal1.y = s[6];
        result.normal2.x = s[7];
        result.normal2.y = s[8];
        return result;
    }
}
