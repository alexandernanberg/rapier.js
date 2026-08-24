import {RigidBodySet} from "../dynamics";
import {Vector, VectorOps} from "../math";
import {RawNarrowPhase, RawContactManifold} from "../raw";
import {scratch} from "../scratch";
import {ColliderHandle} from "./collider";

/**
 * The narrow-phase used for precise collision-detection.
 *
 * To avoid leaking WASM resources, this MUST be freed manually with `narrowPhase.free()`
 * once you are done using it.
 */
export class NarrowPhase {
    raw: RawNarrowPhase;
    tempManifold: TempContactManifold;

    /**
     * Release the WASM memory occupied by this narrow-phase.
     */
    public free() {
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined!;
    }

    constructor(raw?: RawNarrowPhase) {
        this.raw = raw || new RawNarrowPhase();
        this.tempManifold = new TempContactManifold(null!);
    }

    /**
     * Enumerates all the colliders potentially in contact with the given collider.
     *
     * @param collider1 - The second collider involved in the contact.
     * @param f - Closure that will be called on each collider that is in contact with `collider1`.
     */
    public contactPairsWith(collider1: ColliderHandle, f: (collider2: ColliderHandle) => void) {
        this.raw.contact_pairs_with(collider1, f);
    }

    /**
     * Enumerates all the colliders intersecting the given colliders, assuming one of them
     * is a sensor.
     */
    public intersectionPairsWith(
        collider1: ColliderHandle,
        f: (collider2: ColliderHandle) => void,
    ) {
        this.raw.intersection_pairs_with(collider1, f);
    }

    /**
     * Iterates through all the contact manifolds between the given pair of colliders.
     *
     * @param collider1 - The first collider involved in the contact.
     * @param collider2 - The second collider involved in the contact.
     * @param bodies - The set of rigid-bodies the colliders are attached to. Solver contacts are
     *                 anchored in body-local space, so this is needed to read them back in world-space.
     * @param f - Closure that will be called on each contact manifold between the two colliders. If the second argument
     *            passed to this closure is `true`, then the contact manifold data is flipped, i.e., methods like `localNormal1`
     *            actually apply to the `collider2` and fields like `localNormal2` apply to the `collider1`.
     */
    public contactPair(
        collider1: ColliderHandle,
        collider2: ColliderHandle,
        bodies: RigidBodySet,
        f: (manifold: TempContactManifold, flipped: boolean) => void,
    ) {
        const rawPair = this.raw.contact_pair(collider1, collider2);

        if (!!rawPair) {
            const flipped = rawPair.collider1() != collider1;
            this.tempManifold.bodies = bodies;

            let i;
            for (i = 0; i < rawPair.numContactManifolds(); ++i) {
                this.tempManifold.raw = rawPair.contactManifold(i)!;
                if (!!this.tempManifold.raw) {
                    f(this.tempManifold, flipped);
                }

                // SAFETY: The RawContactManifold stores a raw pointer that will be invalidated
                //         at the next timestep. So we must be sure to free the pair here
                //         to avoid unsoundness in the Rust code.
                this.tempManifold.free();
            }
            rawPair.free();
        }
    }

    /**
     * Returns `true` if `collider1` and `collider2` intersect and at least one of them is a sensor.
     * @param collider1 − The first collider involved in the intersection.
     * @param collider2 − The second collider involved in the intersection.
     */
    public intersectionPair(collider1: ColliderHandle, collider2: ColliderHandle): boolean {
        return this.raw.intersection_pair(collider1, collider2);
    }
}

export class TempContactManifold {
    raw: RawContactManifold;
    /** The bodies the manifold's solver contacts are anchored to. */
    bodies: RigidBodySet;

    public free() {
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined!;
    }

    constructor(raw: RawContactManifold, bodies?: RigidBodySet) {
        this.raw = raw;
        this.bodies = bodies!;
    }

    public normal(target: Vector): Vector {
        this.raw.normal();
        return VectorOps.fromBuffer(scratch(), target);
    }

    public localNormal1(target: Vector): Vector {
        this.raw.local_n1();
        return VectorOps.fromBuffer(scratch(), target);
    }

    public localNormal2(target: Vector): Vector {
        this.raw.local_n2();
        return VectorOps.fromBuffer(scratch(), target);
    }

    /**
     * The user-defined 32-bit integer attached to this manifold by a
     * `PhysicsHooks.modifySolverContacts` hook. Preserved across steps, and `0`
     * if no hook ever set it.
     */
    public userData(): number {
        return this.raw.user_data();
    }

    public subshape1(): number {
        return this.raw.subshape1();
    }

    public subshape2(): number {
        return this.raw.subshape2();
    }

    public numContacts(): number {
        return this.raw.num_contacts();
    }

    public localContactPoint1(i: number, target: Vector): Vector | null {
        if (!this.raw.contact_local_p1(i)) return null;
        return VectorOps.fromBuffer(scratch(), target);
    }

    public localContactPoint2(i: number, target: Vector): Vector | null {
        if (!this.raw.contact_local_p2(i)) return null;
        return VectorOps.fromBuffer(scratch(), target);
    }

    public contactDist(i: number): number {
        return this.raw.contact_dist(i);
    }

    public contactFid1(i: number): number {
        return this.raw.contact_fid1(i);
    }

    public contactFid2(i: number): number {
        return this.raw.contact_fid2(i);
    }

    public contactImpulse(i: number): number {
        return this.raw.contact_impulse(i);
    }

    public contactTangentImpulse(i: number): number {
        return this.raw.contact_tangent_impulse(i);
    }

    public numSolverContacts(): number {
        return this.raw.num_solver_contacts();
    }

    /**
     * The contact point on the first body's surface.
     *
     * This is expressed in that body's center-of-mass-centered local frame, or in
     * world-space when the first side has no solver body (no rigid-body, or world-attached
     * by dominance — fixed bodies included).
     */
    public solverContactAnchor1(i: number, target: Vector): Vector | null {
        if (!this.raw.solver_contact_anchor1(i)) return null;
        return VectorOps.fromBuffer(scratch(), target);
    }

    /**
     * The contact point on the second body's surface, expressed like
     * {@link solverContactAnchor1}.
     */
    public solverContactAnchor2(i: number, target: Vector): Vector | null {
        if (!this.raw.solver_contact_anchor2(i)) return null;
        return VectorOps.fromBuffer(scratch(), target);
    }

    /**
     * The world-space contact point the solver acts on, midway between both surfaces.
     *
     * Returns `null` if `i` is out of bounds.
     */
    public solverContactPoint(i: number, target: Vector): Vector | null {
        if (!this.raw.solver_contact_point(this.bodies.raw, i)) return null;
        return VectorOps.fromBuffer(scratch(), target);
    }

    public solverContactDist(i: number): number {
        return this.raw.solver_contact_dist(i);
    }

    /**
     * The effective friction coefficient of this manifold's contacts.
     *
     * Friction is stored per-manifold, so it is identical for every contact of this manifold.
     */
    public friction(): number {
        return this.raw.friction();
    }

    /**
     * The effective restitution coefficient of this manifold's contacts.
     *
     * Restitution is stored per-manifold, so it is identical for every contact of this manifold.
     */
    public restitution(): number {
        return this.raw.restitution();
    }

    public solverContactTangentVelocity(i: number, target: Vector): Vector | null {
        if (!this.raw.solver_contact_tangent_velocity(i)) return null;
        return VectorOps.fromBuffer(scratch(), target);
    }
}
