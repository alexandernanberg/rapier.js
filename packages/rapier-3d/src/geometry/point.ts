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
    point: Vector;
    /**
     * Is the point inside of the collider?
     */
    isInside: boolean;

    constructor(point: Vector, isInside: boolean) {
        this.point = point;
        this.isInside = isInside;
    }

    /**
     * Reads a point projection from its raw representation.
     *
     * @param raw - The raw projection. It is always freed before returning.
     * @param target - Optional target object to write the result to (avoids allocation).
     */
    public static fromRaw(
        raw: RawPointProjection,
        target?: PointProjection,
    ): PointProjection | null {
        if (!raw) return null;

        const point = VectorOps.fromRaw(raw.point(), target?.point)!;
        const isInside = raw.isInside();
        raw.free();

        if (!target) return new PointProjection(point, isInside);

        target.point = point;
        target.isInside = isInside;
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
    collider: Collider;
    /**
     * The projection of the point on the collider.
     */
    point: Vector;
    /**
     * Is the point inside of the collider?
     */
    isInside: boolean;

    /**
     * The type of the geometric feature the point was projected on.
     */
    featureType = FeatureType.Unknown;

    /**
     * The id of the geometric feature the point was projected on.
     */
    featureId: number | undefined = undefined;

    constructor(
        collider: Collider,
        point: Vector,
        isInside: boolean,
        featureType?: FeatureType,
        featureId?: number,
    ) {
        this.collider = collider;
        this.point = point;
        this.isInside = isInside;
        if (featureId !== undefined) this.featureId = featureId;
        if (featureType !== undefined) this.featureType = featureType;
    }
}
