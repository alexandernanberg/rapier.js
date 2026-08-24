import {Vector, VectorOps} from "../math";
import {RawRayIntersection} from "../raw";
import {Collider} from "./collider";
import {FeatureType} from "./feature";

/**
 * A ray. This is a directed half-line.
 */
export class Ray {
    /**
     * The starting point of the ray.
     */
    public origin: Vector;
    /**
     * The direction of propagation of the ray.
     */
    public dir: Vector;

    /**
     * Builds a ray from its origin and direction.
     *
     * @param origin - The ray's starting point.
     * @param dir - The ray's direction of propagation.
     */
    constructor(origin: Vector, dir: Vector) {
        this.origin = origin;
        this.dir = dir;
    }

    /**
     * The point at parameter `t` along this ray.
     *
     * @param t - The ray parameter.
     * @param target - The object the result is written into.
     */
    public pointAt(t: number, target: Vector): Vector {
        target.x = this.origin.x + this.dir.x * t;
        target.y = this.origin.y + this.dir.y * t;
        target.z = this.origin.z + this.dir.z * t;
        return target;
    }
}

/**
 * The intersection between a ray and a collider.
 */
export class RayIntersection {
    /**
     * The time-of-impact of the ray with the collider.
     *
     * The hit point is obtained from the ray's origin and direction: `origin + dir * timeOfImpact`.
     */
    timeOfImpact = 0;
    /**
     * The normal of the collider at the hit point.
     */
    normal: Vector = VectorOps.zeros();

    /**
     * The type of the geometric feature the point was projected on.
     */
    featureType = FeatureType.Unknown;

    /**
     * The id of the geometric feature the point was projected on.
     */
    featureId: number | undefined = undefined;

    /**
     * Reads a ray intersection from its raw representation.
     *
     * @param raw - The raw intersection. It is always freed before returning.
     * @param target - The object the result is written into.
     */
    public static fromRaw(
        raw: RawRayIntersection,
        target: RayIntersection,
    ): RayIntersection | null {
        if (!raw) return null;

        target.timeOfImpact = raw.time_of_impact();
        VectorOps.fromRaw(raw.normal(), target.normal);
        target.featureType = raw.featureType() as number as FeatureType;
        target.featureId = raw.featureId();
        raw.free();

        return target;
    }
}

/**
 * The intersection between a ray and a collider (includes the collider handle).
 */
export class RayColliderIntersection {
    /**
     * The collider hit by the ray.
     */
    collider!: Collider;
    /**
     * The time-of-impact of the ray with the collider.
     *
     * The hit point is obtained from the ray's origin and direction: `origin + dir * timeOfImpact`.
     */
    timeOfImpact = 0;
    /**
     * The normal of the collider at the hit point.
     */
    normal: Vector = VectorOps.zeros();

    /**
     * The type of the geometric feature the point was projected on.
     */
    featureType = FeatureType.Unknown;

    /**
     * The id of the geometric feature the point was projected on.
     */
    featureId: number | undefined = undefined;
}

/**
 * The time of impact between a ray and a collider.
 */
export class RayColliderHit {
    /**
     * The handle of the collider hit by the ray.
     */
    collider!: Collider;
    /**
     * The time-of-impact of the ray with the collider.
     *
     * The hit point is obtained from the ray's origin and direction: `origin + dir * timeOfImpact`.
     */
    timeOfImpact = 0;
}
