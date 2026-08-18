import {RigidBodyHandle} from "../dynamics";
import {ColliderHandle} from "../geometry";
import {Vector, VectorOps} from "../math";
import {RawContactModificationContext} from "../raw";
import {scratch} from "../scratch";

export enum ActiveHooks {
    NONE = 0,
    FILTER_CONTACT_PAIRS = 0b0001,
    FILTER_INTERSECTION_PAIRS = 0b0010,
    MODIFY_SOLVER_CONTACTS = 0b0100,
}

export enum SolverFlags {
    EMPTY = 0b000,
    COMPUTE_IMPULSE = 0b001,
}

/**
 * The contacts the solver is about to see for one contact manifold, handed to
 * `PhysicsHooks.modifySolverContacts` so they can be adjusted or discarded.
 *
 * The context is only valid for the duration of that call: the same object is
 * reused for every manifold, and outside of the hook every getter reads zero and
 * every setter does nothing. Don't hold on to it.
 */
export class ContactModificationContext {
    raw: RawContactModificationContext;

    constructor(raw?: RawContactModificationContext) {
        this.raw = raw || new RawContactModificationContext();
    }

    /**
     * Release the WASM memory occupied by this context.
     */
    public free() {
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined!;
    }

    /**
     * Is a contact-modification hook call currently in progress?
     *
     * Everything below only reads or writes anything while this is `true`.
     */
    public isActive(): boolean {
        return this.raw.isActive();
    }

    /**
     * The first collider involved in the contact.
     */
    public collider1(): ColliderHandle {
        return this.raw.collider1()!;
    }

    /**
     * The second collider involved in the contact.
     */
    public collider2(): ColliderHandle {
        return this.raw.collider2()!;
    }

    /**
     * The rigid-body the first collider is attached to, or `null` if it has none.
     */
    public rigidBody1(): RigidBodyHandle | null {
        return this.raw.rigidBody1() ?? null;
    }

    /**
     * The rigid-body the second collider is attached to, or `null` if it has none.
     */
    public rigidBody2(): RigidBodyHandle | null {
        return this.raw.rigidBody2() ?? null;
    }

    /**
     * The contact normal, pointing from the first collider towards the second one.
     *
     * @param target - Optional target object to write the result to (avoids allocation).
     */
    public normal(target?: Vector): Vector {
        this.raw.normal();
        return VectorOps.fromBuffer(scratch(), target);
    }

    /**
     * Overrides the contact normal. It must be a unit vector, pointing from the
     * first collider towards the second one.
     */
    public setNormal(normal: Vector) {
        const rawNormal = VectorOps.intoRaw(normal);
        this.raw.setNormal(rawNormal);
        rawNormal.free();
    }

    /**
     * The friction coefficient applied to every contact of this manifold.
     */
    public friction(): number {
        return this.raw.friction();
    }

    /**
     * Overrides the friction coefficient applied to every contact of this manifold.
     */
    public setFriction(friction: number) {
        this.raw.setFriction(friction);
    }

    /**
     * The restitution coefficient applied to every contact of this manifold.
     */
    public restitution(): number {
        return this.raw.restitution();
    }

    /**
     * Overrides the restitution coefficient applied to every contact of this manifold.
     */
    public setRestitution(restitution: number) {
        this.raw.setRestitution(restitution);
    }

    /**
     * The user-defined 32-bit integer attached to this manifold.
     *
     * It is preserved from one step to the next, and can be read back from the
     * narrow-phase with `TempContactManifold.userData()`.
     */
    public userData(): number {
        return this.raw.userData();
    }

    /**
     * Sets the user-defined 32-bit integer attached to this manifold.
     */
    public setUserData(userData: number) {
        this.raw.setUserData(userData);
    }

    /**
     * The number of contacts the solver will see for this manifold.
     */
    public numSolverContacts(): number {
        return this.raw.numSolverContacts();
    }

    /**
     * Removes the `i`-th solver contact so the solver ignores it entirely.
     *
     * The last contact is swapped into `i`, so indices shift: iterate backwards
     * when removing more than one contact.
     */
    public removeSolverContact(i: number) {
        this.raw.removeSolverContact(i);
    }

    /**
     * Removes every solver contact of this manifold, which disables the contact
     * response while the collision is still reported by the event queue.
     */
    public clearSolverContacts() {
        this.raw.clearSolverContacts();
    }

    /**
     * The world-space contact point of the `i`-th solver contact, on the first
     * collider's surface.
     *
     * @param target - Optional target object to write the result to (avoids allocation).
     */
    public solverContactPoint1(i: number, target?: Vector): Vector | null {
        if (!this.raw.solverContactPoint1(i)) return null;
        return VectorOps.fromBuffer(scratch(), target);
    }

    /**
     * The world-space contact point of the `i`-th solver contact, on the second
     * collider's surface.
     *
     * @param target - Optional target object to write the result to (avoids allocation).
     */
    public solverContactPoint2(i: number, target?: Vector): Vector | null {
        if (!this.raw.solverContactPoint2(i)) return null;
        return VectorOps.fromBuffer(scratch(), target);
    }

    /**
     * Moves the world-space contact point of the `i`-th solver contact on the first
     * collider's surface.
     */
    public setSolverContactPoint1(i: number, point: Vector) {
        const rawPoint = VectorOps.intoRaw(point);
        this.raw.setSolverContactPoint1(i, rawPoint);
        rawPoint.free();
    }

    /**
     * Moves the world-space contact point of the `i`-th solver contact on the second
     * collider's surface.
     */
    public setSolverContactPoint2(i: number, point: Vector) {
        const rawPoint = VectorOps.intoRaw(point);
        this.raw.setSolverContactPoint2(i, rawPoint);
        rawPoint.free();
    }

    /**
     * The separation of the `i`-th solver contact: negative means penetration.
     */
    public solverContactDist(i: number): number {
        return this.raw.solverContactDist(i);
    }

    /**
     * Overrides the separation of the `i`-th solver contact.
     */
    public setSolverContactDist(i: number, dist: number) {
        this.raw.setSolverContactDist(i, dist);
    }

    /**
     * The tangent (surface) velocity of the `i`-th solver contact.
     *
     * @param target - Optional target object to write the result to (avoids allocation).
     */
    public solverContactTangentVelocity(i: number, target?: Vector): Vector | null {
        if (!this.raw.solverContactTangentVelocity(i)) return null;
        return VectorOps.fromBuffer(scratch(), target);
    }

    /**
     * Sets the tangent (surface) velocity of the `i`-th solver contact, which is what
     * makes a collider act like a conveyor belt.
     */
    public setSolverContactTangentVelocity(i: number, velocity: Vector) {
        const rawVelocity = VectorOps.intoRaw(velocity);
        this.raw.setSolverContactTangentVelocity(i, rawVelocity);
        rawVelocity.free();
    }
}

export interface PhysicsHooks {
    /**
     * Function that determines if contacts computation should happen between two colliders, and how the
     * constraints solver should behave for these contacts.
     *
     * This will only be executed and taken into account if at least one of the involved colliders contains the
     * `ActiveHooks.FILTER_CONTACT_PAIR` flag in its active hooks.
     *
     * @param collider1 − Handle of the first collider involved in the potential contact.
     * @param collider2 − Handle of the second collider involved in the potential contact.
     * @param body1 − Handle of the first body involved in the potential contact.
     * @param body2 − Handle of the second body involved in the potential contact.
     */
    filterContactPair?(
        collider1: ColliderHandle,
        collider2: ColliderHandle,
        body1: RigidBodyHandle,
        body2: RigidBodyHandle,
    ): SolverFlags | null;

    /**
     * Function that determines if intersection computation should happen between two colliders (where at least
     * one is a sensor).
     *
     * This will only be executed and taken into account if `one of the involved colliders contains the
     * `ActiveHooks.FILTER_INTERSECTION_PAIR` flag in its active hooks.
     *
     * @param collider1 − Handle of the first collider involved in the potential contact.
     * @param collider2 − Handle of the second collider involved in the potential contact.
     * @param body1 − Handle of the first body involved in the potential contact.
     * @param body2 − Handle of the second body involved in the potential contact.
     */
    filterIntersectionPair?(
        collider1: ColliderHandle,
        collider2: ColliderHandle,
        body1: RigidBodyHandle,
        body2: RigidBodyHandle,
    ): boolean;

    /**
     * Function that can adjust the contacts the solver is about to use, for one contact
     * manifold at a time: their friction and restitution, the contact normal, the
     * surface velocity (for conveyor belts), or which contacts the solver sees at all.
     *
     * This will only be executed and taken into account if at least one of the involved
     * colliders contains the `ActiveHooks.MODIFY_SOLVER_CONTACTS` flag in its active hooks.
     *
     * @param context − The contacts of the manifold being modified. It is only valid for
     *                  the duration of this call.
     */
    modifySolverContacts?(context: ContactModificationContext): void;
}
