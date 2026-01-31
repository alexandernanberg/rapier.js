import {
    IntegrationParameters,
    ImpulseJointSet,
    MultibodyJointSet,
    RigidBodyHandle,
    RigidBodySet,
    CCDSolver,
    IslandManager,
} from "../dynamics";
import {BroadPhase, ColliderHandle, ColliderSet, NarrowPhase} from "../geometry";
import {Vector, VectorOps} from "../math";
import {RawPhysicsPipeline, RawVector} from "../raw";
import {EventQueue} from "./event_queue";
import {PhysicsHooks} from "./physics_hooks";

export class PhysicsPipeline {
    raw: RawPhysicsPipeline;
    private cachedGravity: RawVector | null = null;
    private lastGravityX = 0;
    private lastGravityY = 0;
    private lastGravityZ = 0;

    public free() {
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
                hooks?.filterContactPair as Function,
                hooks?.filterIntersectionPair as Function,
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
