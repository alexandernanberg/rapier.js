import {Vector, VectorOps} from "../math";
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
     * Reads a point projection out of the scratch buffer, as written by the WASM
     * `projectPoint` calls: `point, isInside`.
     *
     * @param buf - The scratch buffer.
     * @param target - Optional target object to write the result to (avoids allocation).
     */
    public static fromBuffer(buf: Float32Array, target?: PointProjection): PointProjection {
        const isInside = buf[2] !== 0;
        if (!target) return new PointProjection(VectorOps.new(buf[0], buf[1]), isInside);

        target.point = VectorOps.set(target.point, buf[0], buf[1]);
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
