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

    public pointAt(t: number): Vector {
        return {
            x: this.origin.x + this.dir.x * t,
            y: this.origin.y + this.dir.y * t,
        };
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
    timeOfImpact: number;
    /**
     * The normal of the collider at the hit point.
     */
    normal: Vector;

    /**
     * The type of the geometric feature the point was projected on.
     */
    featureType = FeatureType.Unknown;

    /**
     * The id of the geometric feature the point was projected on.
     */
    featureId: number | undefined = undefined;

    constructor(
        timeOfImpact: number,
        normal: Vector,
        featureType?: FeatureType,
        featureId?: number,
    ) {
        this.timeOfImpact = timeOfImpact;
        this.normal = normal;
        if (featureId !== undefined) this.featureId = featureId;
        if (featureType !== undefined) this.featureType = featureType;
    }

    /**
     * Reads a ray intersection from its raw representation.
     *
     * @param raw - The raw intersection. It is always freed before returning.
     * @param target - Optional target object to write the result to (avoids allocation).
     */
    public static fromRaw(
        raw: RawRayIntersection,
        target?: RayIntersection,
    ): RayIntersection | null {
        if (!raw) return null;

        const timeOfImpact = raw.time_of_impact();
        const normal = VectorOps.fromRaw(raw.normal(), target?.normal)!;
        const featureType = raw.featureType() as number as FeatureType;
        const featureId = raw.featureId();
        raw.free();

        if (!target) return new RayIntersection(timeOfImpact, normal, featureType, featureId);

        target.timeOfImpact = timeOfImpact;
        target.normal = normal;
        target.featureType = featureType;
        target.featureId = featureId;
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
    collider: Collider;
    /**
     * The time-of-impact of the ray with the collider.
     *
     * The hit point is obtained from the ray's origin and direction: `origin + dir * timeOfImpact`.
     */
    timeOfImpact: number;
    /**
     * The normal of the collider at the hit point.
     */
    normal: Vector;

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
        timeOfImpact: number,
        normal: Vector,
        featureType?: FeatureType,
        featureId?: number,
    ) {
        this.collider = collider;
        this.timeOfImpact = timeOfImpact;
        this.normal = normal;
        if (featureId !== undefined) this.featureId = featureId;
        if (featureType !== undefined) this.featureType = featureType;
    }
}

/**
 * The time of impact between a ray and a collider.
 */
export class RayColliderHit {
    /**
     * The handle of the collider hit by the ray.
     */
    collider: Collider;
    /**
     * The time-of-impact of the ray with the collider.
     *
     * The hit point is obtained from the ray's origin and direction: `origin + dir * timeOfImpact`.
     */
    timeOfImpact: number;

    constructor(collider: Collider, timeOfImpact: number) {
        this.collider = collider;
        this.timeOfImpact = timeOfImpact;
    }
}
