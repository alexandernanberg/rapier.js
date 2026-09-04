import {Vector, VectorOps} from "../math";
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
            z: this.origin.z + this.dir.z * t,
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
     * Reads a ray intersection out of the scratch buffer, as written by the WASM
     * `castRayAndGetNormal` calls: `timeOfImpact, normal, featureType, featureId`.
     *
     * @param buf - The scratch buffer (`f32` view).
     * @param u32 - The same buffer viewed as `u32`s, for the feature type and id.
     * @param target - Optional target object to write the result to (avoids allocation).
     */
    public static fromBuffer(
        buf: Float32Array,
        u32: Uint32Array,
        target?: RayIntersection,
    ): RayIntersection {
        const featureType = u32[4] as FeatureType;
        const featureId = u32[5] === 0xffffffff ? undefined : u32[5];

        if (!target) {
            return new RayIntersection(
                buf[0],
                VectorOps.new(buf[1], buf[2], buf[3]),
                featureType,
                featureId,
            );
        }

        target.timeOfImpact = buf[0];
        target.normal = VectorOps.set(target.normal, buf[1], buf[2], buf[3]);
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
