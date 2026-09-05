import {
    IntegrationParameters,
    ImpulseJointSet,
    MultibodyJointSet,
    RigidBodySet,
    CCDSolver,
    IslandManager,
} from "../dynamics";
import {BroadPhase, ColliderSet, NarrowPhase} from "../geometry";
import {Vector, VectorOps} from "../math";
import {RawPhysicsPipeline, RawVector} from "../raw";
import {EventQueue} from "./event_queue";
import {ContactModificationContext, PhysicsHooks, SolverFlags} from "./physics_hooks";

type FilterContactPair = NonNullable<PhysicsHooks["filterContactPair"]>;
type FilterIntersectionPair = NonNullable<PhysicsHooks["filterIntersectionPair"]>;
type ModifySolverContacts = NonNullable<PhysicsHooks["modifySolverContacts"]>;

export class PhysicsPipeline {
    raw: RawPhysicsPipeline;
    private contactModification: ContactModificationContext | null = null;
    // The hook functions handed to WASM, each wrapped once per user function (see
    // `guardHooks`) and cached by the identity of the function it wraps, so that a
    // hooks object whose methods get reassigned keeps working.
    private hooksObject: PhysicsHooks | null = null;
    private filterContactPairSource: FilterContactPair | undefined = undefined;
    private filterContactPair: FilterContactPair | undefined = undefined;
    private filterIntersectionPairSource: FilterIntersectionPair | undefined = undefined;
    private filterIntersectionPair: FilterIntersectionPair | undefined = undefined;
    private modifySolverContactsSource: ModifySolverContacts | undefined = undefined;
    private modifySolverContacts: (() => void) | undefined = undefined;
    // The first exception a hook threw during the current step. Rust cannot let it
    // through the boundary (it used to drop it, and treat the pair as filtered
    // out); it is kept here for `rethrowHookError` to throw once the step is done.
    private hookError: unknown = undefined;
    private hookFailed = false;
    private cachedGravity: RawVector | null = null;
    private lastGravityX = 0;
    private lastGravityY = 0;
    private lastGravityZ = 0;

    public free() {
        if (this.contactModification) {
            this.contactModification.free();
            this.contactModification = null;
        }
        this.hooksObject = null;
        this.filterContactPair = this.filterContactPairSource = undefined;
        this.filterIntersectionPair = this.filterIntersectionPairSource = undefined;
        this.modifySolverContacts = this.modifySolverContactsSource = undefined;
        if (this.cachedGravity) {
            this.cachedGravity.free();
            this.cachedGravity = null;
        }
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined as unknown as RawPhysicsPipeline;
    }

    constructor(raw?: RawPhysicsPipeline) {
        this.raw = raw || new RawPhysicsPipeline();
    }

    /**
     * The zero-argument callback WASM invokes for `PhysicsHooks.modifySolverContacts`.
     *
     * The context object it closes over is reused across every manifold and every
     * step: the Rust side points it at the manifold being modified right before the
     * call, so the hook costs no allocation. Both the context and the closure are
     * cached until the hooks object changes.
     */
    private recordHookError(error: unknown) {
        if (!this.hookFailed) {
            this.hookFailed = true;
            this.hookError = error;
        }
    }

    /**
     * Refreshes the wrapped hook functions for `hooks`.
     *
     * Each wrapper catches what the user's function throws, records the first
     * error for {@link rethrowHookError}, and answers the way an absent hook
     * would (keep the pair, compute its impulses) for the rest of the step; the
     * user's function is not called again during that step.
     */
    private guardHooks(hooks: PhysicsHooks) {
        if (this.hooksObject !== hooks) {
            this.hooksObject = hooks;
            this.filterContactPairSource = undefined;
            this.filterIntersectionPairSource = undefined;
            this.modifySolverContactsSource = undefined;
        }

        const filterContactPair = hooks.filterContactPair;
        if (this.filterContactPairSource !== filterContactPair) {
            this.filterContactPairSource = filterContactPair;
            this.filterContactPair = filterContactPair
                ? (collider1, collider2, body1, body2) => {
                      if (this.hookFailed) return SolverFlags.COMPUTE_IMPULSE;
                      try {
                          return hooks.filterContactPair!(collider1, collider2, body1, body2);
                      } catch (e) {
                          this.recordHookError(e);
                          return SolverFlags.COMPUTE_IMPULSE;
                      }
                  }
                : undefined;
        }

        const filterIntersectionPair = hooks.filterIntersectionPair;
        if (this.filterIntersectionPairSource !== filterIntersectionPair) {
            this.filterIntersectionPairSource = filterIntersectionPair;
            this.filterIntersectionPair = filterIntersectionPair
                ? (collider1, collider2, body1, body2) => {
                      if (this.hookFailed) return true;
                      try {
                          return hooks.filterIntersectionPair!(collider1, collider2, body1, body2);
                      } catch (e) {
                          this.recordHookError(e);
                          return true;
                      }
                  }
                : undefined;
        }

        const modifySolverContacts = hooks.modifySolverContacts;
        if (this.modifySolverContactsSource !== modifySolverContacts) {
            this.modifySolverContactsSource = modifySolverContacts;
            if (modifySolverContacts) {
                const context = (this.contactModification ??= new ContactModificationContext());
                this.modifySolverContacts = () => {
                    if (this.hookFailed) return;
                    try {
                        hooks.modifySolverContacts!(context);
                    } catch (e) {
                        this.recordHookError(e);
                    }
                };
            } else {
                this.modifySolverContacts = undefined;
            }
        }
    }

    /**
     * Throws the first exception a hook raised during the last step, if any.
     *
     * @internal
     */
    public rethrowHookError() {
        if (this.hookFailed) {
            const error = this.hookError;
            this.hookFailed = false;
            this.hookError = undefined;
            throw error;
        }
    }

    public step(
        gravity: Vector,
        integrationParameters: IntegrationParameters,
        islands: IslandManager,
        broadPhase: BroadPhase,
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        impulseJoints: ImpulseJointSet,
        multibodyJoints: MultibodyJointSet,
        ccdSolver: CCDSolver,
        eventQueue?: EventQueue,
        hooks?: PhysicsHooks,
    ) {
        if (
            !this.cachedGravity ||
            gravity.x !== this.lastGravityX ||
            gravity.y !== this.lastGravityY ||
            gravity.z !== this.lastGravityZ
        ) {
            this.cachedGravity?.free();
            this.cachedGravity = VectorOps.intoRaw(gravity);
            this.lastGravityX = gravity.x;
            this.lastGravityY = gravity.y;
            this.lastGravityZ = gravity.z;
        }

        // The wrapped hook functions stay cached across steps, but only a step
        // that was given hooks may run them.
        let filterContactPair: FilterContactPair | undefined = undefined;
        let filterIntersectionPair: FilterIntersectionPair | undefined = undefined;
        let modifySolverContacts: (() => void) | undefined = undefined;
        if (hooks) {
            this.guardHooks(hooks);
            filterContactPair = this.filterContactPair;
            filterIntersectionPair = this.filterIntersectionPair;
            modifySolverContacts = this.modifySolverContacts;
        }

        if (!!eventQueue) {
            this.raw.stepWithEvents(
                this.cachedGravity,
                integrationParameters.raw,
                islands.raw,
                broadPhase.raw,
                narrowPhase.raw,
                bodies.raw,
                colliders.raw,
                impulseJoints.raw,
                multibodyJoints.raw,
                ccdSolver.raw,
                eventQueue.raw,
                hooks as object,
                filterContactPair,
                filterIntersectionPair,
                modifySolverContacts,
            );
        } else if (!!hooks) {
            this.raw.stepWithHooks(
                this.cachedGravity,
                integrationParameters.raw,
                islands.raw,
                broadPhase.raw,
                narrowPhase.raw,
                bodies.raw,
                colliders.raw,
                impulseJoints.raw,
                multibodyJoints.raw,
                ccdSolver.raw,
                hooks as object,
                filterContactPair,
                filterIntersectionPair,
                modifySolverContacts,
            );
        } else {
            this.raw.step(
                this.cachedGravity,
                integrationParameters.raw,
                islands.raw,
                broadPhase.raw,
                narrowPhase.raw,
                bodies.raw,
                colliders.raw,
                impulseJoints.raw,
                multibodyJoints.raw,
                ccdSolver.raw,
            );
        }
    }
}
