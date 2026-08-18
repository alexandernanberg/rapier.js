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
import {ContactModificationContext, PhysicsHooks} from "./physics_hooks";

export class PhysicsPipeline {
    raw: RawPhysicsPipeline;
    private contactModification: ContactModificationContext | null = null;
    private modifyHooks: PhysicsHooks | null = null;
    private modifyCallback: (() => void) | null = null;
    private cachedGravity: RawVector | null = null;
    private lastGravityX = 0;
    private lastGravityY = 0;

    public free() {
        if (this.contactModification) {
            this.contactModification.free();
            this.contactModification = null;
            this.modifyHooks = null;
            this.modifyCallback = null;
        }
        if (this.cachedGravity) {
            this.cachedGravity.free();
            this.cachedGravity = null;
        }
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined!;
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
    private modifySolverContactsCallback(hooks?: PhysicsHooks): (() => void) | undefined {
        if (!hooks?.modifySolverContacts) {
            return undefined;
        }

        if (this.modifyHooks !== hooks) {
            const context = (this.contactModification ??= new ContactModificationContext());
            this.modifyHooks = hooks;
            this.modifyCallback = () => hooks.modifySolverContacts!(context);
        }

        return this.modifyCallback!;
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
            gravity.y !== this.lastGravityY
        ) {
            this.cachedGravity?.free();
            this.cachedGravity = VectorOps.intoRaw(gravity);
            this.lastGravityX = gravity.x;
            this.lastGravityY = gravity.y;
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
                hooks?.filterContactPair,
                hooks?.filterIntersectionPair,
                this.modifySolverContactsCallback(hooks),
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
                hooks.filterContactPair,
                hooks.filterIntersectionPair,
                this.modifySolverContactsCallback(hooks),
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
