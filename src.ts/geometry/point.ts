import { ColliderHandle } from "./collider";
import { Vector, VectorOps } from "../math";
import { RawPointColliderProjection, RawPointProjection } from "../raw";


/**
 * The projection of a point on a collider.
 */
export class PointProjection {
    /**
     * The projection of the point on the collider.
     */
    point: Vector
    /**
     * Is the point inside of the collider?
     */
    isInside: boolean

    constructor(point: Vector, isInside: boolean) {
        this.point = point;
        this.isInside = isInside;
    }

    public static fromRaw(raw: RawPointProjection): PointProjection {
        if (!raw)
            return null;

        const result = new PointProjection(
            VectorOps.fromRaw(raw.point()),
            raw.isInside()
        );
        raw.free();
        return result;
    }
}

/**
 * The projection of a point on a collider (includes the collider handle).
 */
export class PointColliderProjection {
    /**
     * The handle of the collider hit by the ray.
     */
    colliderHandle: ColliderHandle
    /**
     * The projection of the point on the collider.
     */
    point: Vector
    /**
     * Is the point inside of the collider?
     */
    isInside: boolean

    constructor(colliderHandle: ColliderHandle, point: Vector, isInside: boolean) {
        this.colliderHandle = colliderHandle;
        this.point = point;
        this.isInside = isInside;
    }

    public static fromRaw(raw: RawPointColliderProjection): PointColliderProjection {
        if (!raw)
            return null;

        const result = new PointColliderProjection(
            raw.colliderHandle(),
            VectorOps.fromRaw(raw.point()),
            raw.isInside()
        );
        raw.free();
        return result;
    }
}