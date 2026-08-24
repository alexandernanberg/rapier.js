import {Vector, VectorOps} from "../math";
import {RawPointProjection} from "../raw";
import {Collider} from "./collider";
import {FeatureType} from "./feature";

/**
 * The projection of a point on a collider.
 */
export class PointProjection {
    /**
     * The projection of the point on the collider.
     */
    point: Vector = VectorOps.zeros();
    /**
     * Is the point inside of the collider?
     */
    isInside = false;

    /**
     * Reads a point projection from its raw representation.
     *
     * @param raw - The raw projection. It is always freed before returning.
     * @param target - The object the result is written into.
     */
    public static fromRaw(
        raw: RawPointProjection,
        target: PointProjection,
    ): PointProjection | null {
        if (!raw) return null;

        VectorOps.fromRaw(raw.point(), target.point);
        target.isInside = raw.isInside();
        raw.free();

        return target;
    }
}

/**
 * The projection of a point on a collider (includes the collider handle).
 */
export class PointColliderProjection {
    /**
     * The collider hit by the ray.
     */
    collider!: Collider;
    /**
     * The projection of the point on the collider.
     */
    point: Vector = VectorOps.zeros();
    /**
     * Is the point inside of the collider?
     */
    isInside = false;

    /**
     * The type of the geometric feature the point was projected on.
     */
    featureType = FeatureType.Unknown;

    /**
     * The id of the geometric feature the point was projected on.
     */
    featureId: number | undefined = undefined;
}
