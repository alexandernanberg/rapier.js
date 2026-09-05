import {RigidBodyHandle, RigidBodySet} from "../dynamics";
import {Rotation, Vector, VectorOps} from "../math";
import {QueryFilterFlags} from "../pipeline";
import {RawBroadPhase, wasmMemory} from "../raw";
import {scratch} from "../scratch";
import {unpackBufferInfo} from "../transform_buffer";
import {ColliderHandle} from "./collider";
import {ColliderSet} from "./collider_set";
import {FeatureType} from "./feature";
import {InteractionGroups} from "./interaction_groups";
import {NarrowPhase} from "./narrow_phase";
import {PointColliderProjection} from "./point";
import {Ray, RayColliderHit, RayColliderIntersection} from "./ray";
import {Shape} from "./shape";
import {ColliderShapeCastHit} from "./toi";

/**
 * The broad-phase used for coarse collision-detection.
 *
 * To avoid leaking WASM resources, this MUST be freed manually with `broadPhase.free()`
 * once you are done using it.
 */
export class BroadPhase {
    raw: RawBroadPhase;
    /**
     * View over the WASM-side scratch buffer query results are written to.
     * Its address never changes, so it only ever has to be re-created after
     * WASM memory growth detached it.
     */
    private resultsView: Float64Array | null = null;
    private resultsPtr = 0;
    private resultsLen = 0;
    private wasmMemory: WebAssembly.Memory | null = null;

    /**
     * Release the WASM memory occupied by this broad-phase.
     */
    public free() {
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined!;
        this.resultsView = null;
        this.resultsPtr = 0;
    }

    constructor(raw?: RawBroadPhase) {
        this.raw = raw || new RawBroadPhase();
    }

    /**
     * Returns a usable view over the query result buffer, re-creating it if
     * WASM memory growth detached it.
     */
    private results(): Float64Array {
        const view = this.resultsView;
        if (view !== null && view.byteLength !== 0) return view;

        if (this.resultsPtr === 0) {
            const info = unpackBufferInfo(this.raw.queryResultBufferInfo());
            this.resultsPtr = info.ptr;
            this.resultsLen = info.len;
            this.wasmMemory = wasmMemory() as unknown as WebAssembly.Memory;
        }

        return (this.resultsView = new Float64Array(
            this.wasmMemory!.buffer,
            this.resultsPtr,
            this.resultsLen,
        ));
    }

    /** Reads the ray intersection currently held by the result buffer. */
    private rayIntersectionFromResults(
        colliders: ColliderSet,
        target?: RayColliderIntersection,
    ): RayColliderIntersection {
        const r = this.results();

        if (!target) {
            return new RayColliderIntersection(
                colliders.get(r[0])!,
                r[1],
                VectorOps.new(r[2], r[3]),
                r[4] as FeatureType,
                r[5] < 0 ? undefined : r[5],
            );
        }

        target.collider = colliders.get(r[0])!;
        target.timeOfImpact = r[1];
        target.normal = VectorOps.set(target.normal, r[2], r[3]);
        target.featureType = r[4] as FeatureType;
        target.featureId = r[5] < 0 ? undefined : r[5];
        return target;
    }

    /** Reads the point projection currently held by the result buffer. */
    private pointProjectionFromResults(
        colliders: ColliderSet,
        target?: PointColliderProjection,
    ): PointColliderProjection {
        const r = this.results();

        if (!target) {
            return new PointColliderProjection(
                colliders.get(r[0])!,
                VectorOps.new(r[1], r[2]),
                r[3] !== 0,
                r[4] as FeatureType,
                r[5] < 0 ? undefined : r[5],
            );
        }

        target.collider = colliders.get(r[0])!;
        target.point = VectorOps.set(target.point, r[1], r[2]);
        target.isInside = r[3] !== 0;
        target.featureType = r[4] as FeatureType;
        target.featureId = r[5] < 0 ? undefined : r[5];
        return target;
    }

    /**
     * Find the closest intersection between a ray and a set of collider.
     *
     * @param colliders - The set of colliders taking part in this pipeline.
     * @param ray - The ray to cast.
     * @param maxToi - The maximum time-of-impact that can be reported by this cast. This effectively
     *   limits the length of the ray to `ray.dir.norm() * maxToi`.
     * @param solid - If `false` then the ray will attempt to hit the boundary of a shape, even if its
     *   origin already lies inside of a shape. In other terms, `true` implies that all shapes are plain,
     *   whereas `false` implies that all shapes are hollow for this ray-cast.
     * @param groups - Used to filter the colliders that can or cannot be hit by the ray.
     * @param filter - The callback to filter out which collider will be hit.
     */
    public castRay(
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        ray: Ray,
        maxToi: number,
        solid: boolean,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: ColliderHandle,
        filterExcludeRigidBody?: RigidBodyHandle,
        filterPredicate?: (collider: ColliderHandle) => boolean,
        target?: RayColliderHit,
    ): RayColliderHit | null {
        const hit = this.raw.castRay(
            narrowPhase.raw,
            bodies.raw,
            colliders.raw,
            ray.origin.x,
            ray.origin.y,
            ray.dir.x,
            ray.dir.y,
            maxToi,
            solid,
            filterFlags ?? 0,
            filterGroups,
            filterExcludeCollider,
            filterExcludeRigidBody,
            filterPredicate as unknown as Function,
        );

        colliders.rethrowCallbackError();
        if (!hit) return null;

        const r = this.results();
        if (!target) return new RayColliderHit(colliders.get(r[0])!, r[1]);
        target.collider = colliders.get(r[0])!;
        target.timeOfImpact = r[1];
        return target;
    }

    /**
     * Find the closest intersection between a ray and a set of collider.
     *
     * This also computes the normal at the hit point.
     * @param colliders - The set of colliders taking part in this pipeline.
     * @param ray - The ray to cast.
     * @param maxToi - The maximum time-of-impact that can be reported by this cast. This effectively
     *   limits the length of the ray to `ray.dir.norm() * maxToi`.
     * @param solid - If `false` then the ray will attempt to hit the boundary of a shape, even if its
     *   origin already lies inside of a shape. In other terms, `true` implies that all shapes are plain,
     *   whereas `false` implies that all shapes are hollow for this ray-cast.
     * @param groups - Used to filter the colliders that can or cannot be hit by the ray.
     * @param target - Optional target object to write the result to (avoids allocation).
     */
    public castRayAndGetNormal(
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        ray: Ray,
        maxToi: number,
        solid: boolean,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: ColliderHandle,
        filterExcludeRigidBody?: RigidBodyHandle,
        filterPredicate?: (collider: ColliderHandle) => boolean,
        target?: RayColliderIntersection,
    ): RayColliderIntersection | null {
        const hit = this.raw.castRayAndGetNormal(
            narrowPhase.raw,
            bodies.raw,
            colliders.raw,
            ray.origin.x,
            ray.origin.y,
            ray.dir.x,
            ray.dir.y,
            maxToi,
            solid,
            filterFlags ?? 0,
            filterGroups,
            filterExcludeCollider,
            filterExcludeRigidBody,
            filterPredicate as unknown as Function,
        );

        colliders.rethrowCallbackError();
        if (!hit) return null;

        return this.rayIntersectionFromResults(colliders, target);
    }

    /**
     * Cast a ray and collects all the intersections between a ray and the scene.
     *
     * @param colliders - The set of colliders taking part in this pipeline.
     * @param ray - The ray to cast.
     * @param maxToi - The maximum time-of-impact that can be reported by this cast. This effectively
     *   limits the length of the ray to `ray.dir.norm() * maxToi`.
     * @param solid - If `false` then the ray will attempt to hit the boundary of a shape, even if its
     *   origin already lies inside of a shape. In other terms, `true` implies that all shapes are plain,
     *   whereas `false` implies that all shapes are hollow for this ray-cast.
     * @param groups - Used to filter the colliders that can or cannot be hit by the ray.
     * @param callback - The callback called once per hit (in no particular order) between a ray and a collider.
     *   If this callback returns `false`, then the cast will stop and no further hits will be detected/reported.
     */
    public intersectionsWithRay(
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        ray: Ray,
        maxToi: number,
        solid: boolean,
        callback: (intersect: RayColliderIntersection) => boolean,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: ColliderHandle,
        filterExcludeRigidBody?: RigidBodyHandle,
        filterPredicate?: (collider: ColliderHandle) => boolean,
    ) {
        // Each hit is written to the result buffer right before this is called.
        const rawCallback = colliders.guardCallback(() =>
            callback(this.rayIntersectionFromResults(colliders)),
        );

        this.raw.intersectionsWithRay(
            narrowPhase.raw,
            bodies.raw,
            colliders.raw,
            ray.origin.x,
            ray.origin.y,
            ray.dir.x,
            ray.dir.y,
            maxToi,
            solid,
            rawCallback,
            filterFlags ?? 0,
            filterGroups,
            filterExcludeCollider,
            filterExcludeRigidBody,
            filterPredicate as unknown as Function,
        );
        colliders.rethrowCallbackError();
    }

    /**
     * Gets the handle of up to one collider intersecting the given shape.
     *
     * @param colliders - The set of colliders taking part in this pipeline.
     * @param shapePos - The position of the shape used for the intersection test.
     * @param shapeRot - The orientation of the shape used for the intersection test.
     * @param shape - The shape used for the intersection test.
     * @param groups - The bit groups and filter associated to the ray, in order to only
     *   hit the colliders with collision groups compatible with the ray's group.
     */
    public intersectionWithShape(
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        shapePos: Vector,
        shapeRot: Rotation,
        shape: Shape,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: ColliderHandle,
        filterExcludeRigidBody?: RigidBodyHandle,
        filterPredicate?: (collider: ColliderHandle) => boolean,
    ): ColliderHandle | null {
        const rawShape = shape.intoRaw();
        let result: ColliderHandle | undefined;
        try {
            result = this.raw.intersectionWithShape(
                narrowPhase.raw,
                bodies.raw,
                colliders.raw,
                shapePos.x,
                shapePos.y,
                shapeRot,
                rawShape,
                filterFlags ?? 0,
                filterGroups,
                filterExcludeCollider,
                filterExcludeRigidBody,
                filterPredicate as unknown as Function,
            );
        } finally {
            rawShape.free();
        }
        colliders.rethrowCallbackError();
        return result ?? null;
    }

    /**
     * Find the projection of a point on the closest collider.
     *
     * @param colliders - The set of colliders taking part in this pipeline.
     * @param point - The point to project.
     * @param solid - If this is set to `true` then the collider shapes are considered to
     *   be plain (if the point is located inside of a plain shape, its projection is the point
     *   itself). If it is set to `false` the collider shapes are considered to be hollow
     *   (if the point is located inside of an hollow shape, it is projected on the shape's
     *   boundary).
     * @param groups - The bit groups and filter associated to the point to project, in order to only
     *   project on colliders with collision groups compatible with the ray's group.
     * @param target - Optional target object to write the result to (avoids allocation).
     */
    public projectPoint(
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        point: Vector,
        solid: boolean,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: ColliderHandle,
        filterExcludeRigidBody?: RigidBodyHandle,
        filterPredicate?: (collider: ColliderHandle) => boolean,
        target?: PointColliderProjection,
    ): PointColliderProjection | null {
        const hit = this.raw.projectPoint(
            narrowPhase.raw,
            bodies.raw,
            colliders.raw,
            point.x,
            point.y,
            solid,
            filterFlags ?? 0,
            filterGroups,
            filterExcludeCollider,
            filterExcludeRigidBody,
            filterPredicate as unknown as Function,
        );

        colliders.rethrowCallbackError();
        if (!hit) return null;

        return this.pointProjectionFromResults(colliders, target);
    }

    /**
     * Find the projection of a point on the closest collider.
     *
     * @param colliders - The set of colliders taking part in this pipeline.
     * @param point - The point to project.
     * @param groups - The bit groups and filter associated to the point to project, in order to only
     *   project on colliders with collision groups compatible with the ray's group.
     * @param target - Optional target object to write the result to (avoids allocation).
     */
    public projectPointAndGetFeature(
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        point: Vector,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: ColliderHandle,
        filterExcludeRigidBody?: RigidBodyHandle,
        filterPredicate?: (collider: ColliderHandle) => boolean,
        target?: PointColliderProjection,
    ): PointColliderProjection | null {
        const hit = this.raw.projectPointAndGetFeature(
            narrowPhase.raw,
            bodies.raw,
            colliders.raw,
            point.x,
            point.y,
            filterFlags ?? 0,
            filterGroups,
            filterExcludeCollider,
            filterExcludeRigidBody,
            filterPredicate as unknown as Function,
        );

        colliders.rethrowCallbackError();
        if (!hit) return null;

        return this.pointProjectionFromResults(colliders, target);
    }

    /**
     * Find all the colliders containing the given point.
     *
     * @param colliders - The set of colliders taking part in this pipeline.
     * @param point - The point used for the containment test.
     * @param groups - The bit groups and filter associated to the point to test, in order to only
     *   test on colliders with collision groups compatible with the ray's group.
     * @param callback - A function called with the handles of each collider with a shape
     *   containing the `point`.
     */
    public intersectionsWithPoint(
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        point: Vector,
        callback: (handle: ColliderHandle) => boolean,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: ColliderHandle,
        filterExcludeRigidBody?: RigidBodyHandle,
        filterPredicate?: (collider: ColliderHandle) => boolean,
    ) {
        this.raw.intersectionsWithPoint(
            narrowPhase.raw,
            bodies.raw,
            colliders.raw,
            point.x,
            point.y,
            callback,
            filterFlags ?? 0,
            filterGroups,
            filterExcludeCollider,
            filterExcludeRigidBody,
            filterPredicate as unknown as Function,
        );
        colliders.rethrowCallbackError();
    }

    /**
     * Casts a shape at a constant linear velocity and retrieve the first collider it hits.
     * This is similar to ray-casting except that we are casting a whole shape instead of
     * just a point (the ray origin).
     *
     * @param colliders - The set of colliders taking part in this pipeline.
     * @param shapePos - The initial position of the shape to cast.
     * @param shapeRot - The initial rotation of the shape to cast.
     * @param shapeVel - The constant velocity of the shape to cast (i.e. the cast direction).
     * @param shape - The shape to cast.
     * @param targetDistance − If the shape moves closer to this distance from a collider, a hit
     *                       will be returned.
     * @param maxToi - The maximum time-of-impact that can be reported by this cast. This effectively
     *   limits the distance traveled by the shape to `shapeVel.norm() * maxToi`.
     * @param stopAtPenetration - If set to `false`, the linear shape-cast won’t immediately stop if
     *   the shape is penetrating another shape at its starting point **and** its trajectory is such
     *   that it’s on a path to exit that penetration state.
     * @param groups - The bit groups and filter associated to the shape to cast, in order to only
     *   test on colliders with collision groups compatible with this group.
     * @param target - Optional target object to write the result to (avoids allocation).
     */
    public castShape(
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        shapePos: Vector,
        shapeRot: Rotation,
        shapeVel: Vector,
        shape: Shape,
        targetDistance: number,
        maxToi: number,
        stopAtPenetration: boolean,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: ColliderHandle,
        filterExcludeRigidBody?: RigidBodyHandle,
        filterPredicate?: (collider: ColliderHandle) => boolean,
        target?: ColliderShapeCastHit,
    ): ColliderShapeCastHit | null {
        const rawShape = shape.intoRaw();
        let handle: ColliderHandle | undefined;
        try {
            // On a hit, the components are in the scratch buffer.
            handle = this.raw.castShape(
                narrowPhase.raw,
                bodies.raw,
                colliders.raw,
                shapePos.x,
                shapePos.y,
                shapeRot,
                shapeVel.x,
                shapeVel.y,
                rawShape,
                targetDistance,
                maxToi,
                stopAtPenetration,
                filterFlags ?? 0,
                filterGroups,
                filterExcludeCollider,
                filterExcludeRigidBody,
                filterPredicate as unknown as Function,
            );
            // Read out before the shape is freed: every scratch reader copies its
            // payload out before the next WASM call, whatever that call is.
            if (handle !== undefined) {
                target = ColliderShapeCastHit.fromBufferWithCollider(
                    colliders.get(handle)!,
                    scratch(),
                    target,
                );
            }
        } finally {
            rawShape.free();
        }
        colliders.rethrowCallbackError();
        if (handle === undefined) return null;
        return target!;
    }

    /**
     * Retrieve all the colliders intersecting the given shape.
     *
     * @param colliders - The set of colliders taking part in this pipeline.
     * @param shapePos - The position of the shape to test.
     * @param shapeRot - The orientation of the shape to test.
     * @param shape - The shape to test.
     * @param groups - The bit groups and filter associated to the shape to test, in order to only
     *   test on colliders with collision groups compatible with this group.
     * @param callback - A function called with the handles of each collider intersecting the `shape`.
     */
    public intersectionsWithShape(
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        shapePos: Vector,
        shapeRot: Rotation,
        shape: Shape,
        callback: (handle: ColliderHandle) => boolean,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: ColliderHandle,
        filterExcludeRigidBody?: RigidBodyHandle,
        filterPredicate?: (collider: ColliderHandle) => boolean,
    ) {
        const rawShape = shape.intoRaw();
        try {
            this.raw.intersectionsWithShape(
                narrowPhase.raw,
                bodies.raw,
                colliders.raw,
                shapePos.x,
                shapePos.y,
                shapeRot,
                rawShape,
                callback,
                filterFlags ?? 0,
                filterGroups,
                filterExcludeCollider,
                filterExcludeRigidBody,
                filterPredicate as unknown as Function,
            );
        } finally {
            rawShape.free();
        }
        colliders.rethrowCallbackError();
    }

    /**
     * Finds the handles of all the colliders with an AABB intersecting the given AABB.
     *
     * @param aabbCenter - The center of the AABB to test.
     * @param aabbHalfExtents - The half-extents of the AABB to test.
     * @param callback - The callback that will be called with the handles of all the colliders
     *                   currently intersecting the given AABB.
     */
    public collidersWithAabbIntersectingAabb(
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        aabbCenter: Vector,
        aabbHalfExtents: Vector,
        callback: (handle: ColliderHandle) => boolean,
    ) {
        this.raw.collidersWithAabbIntersectingAabb(
            narrowPhase.raw,
            bodies.raw,
            colliders.raw,
            aabbCenter.x,
            aabbCenter.y,
            aabbHalfExtents.x,
            aabbHalfExtents.y,
            callback,
        );
        colliders.rethrowCallbackError();
    }
}
