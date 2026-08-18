import {Rotation, RotationOps, Vector, VectorOps} from "../math";
import {RawJointAxis, RawJointType, RawMotorModel, RawMultibodyJointSet} from "../raw";
import {JointAxis, JointType, MotorModel} from "./impulse_joint";
import {RigidBody} from "./rigid_body";
import {RigidBodySet} from "./rigid_body_set";

/**
 * The integer identifier of a collider added to a `ColliderSet`.
 */
export type MultibodyJointHandle = number;

export class MultibodyJoint {
    protected rawSet: RawMultibodyJointSet; // The MultibodyJoint won't need to free this.
    protected bodySet: RigidBodySet; // The MultibodyJoint won’t need to free this.
    handle: MultibodyJointHandle;

    constructor(rawSet: RawMultibodyJointSet, bodySet: RigidBodySet, handle: MultibodyJointHandle) {
        this.rawSet = rawSet;
        this.bodySet = bodySet;
        this.handle = handle;
    }

    public static newTyped(
        rawSet: RawMultibodyJointSet,
        bodySet: RigidBodySet,
        handle: MultibodyJointHandle,
    ): MultibodyJoint {
        switch (rawSet.jointType(handle)) {
            case RawJointType.Revolute:
                return new RevoluteMultibodyJoint(rawSet, bodySet, handle);
            case RawJointType.Prismatic:
                return new PrismaticMultibodyJoint(rawSet, bodySet, handle);
            case RawJointType.Fixed:
                return new FixedMultibodyJoint(rawSet, bodySet, handle);
            case RawJointType.Spherical:
                return new SphericalMultibodyJoint(rawSet, bodySet, handle);
            default:
                return new MultibodyJoint(rawSet, bodySet, handle);
        }
    }

    /** @internal */
    public finalizeDeserialization(bodySet: RigidBodySet) {
        this.bodySet = bodySet;
    }

    /**
     * Checks if this joint is still valid (i.e. that it has
     * not been deleted from the joint set yet).
     */
    public isValid(): boolean {
        return this.rawSet.contains(this.handle);
    }

    /**
     * The first rigid-body this joint is attached to.
     *
     * That is the body of the parent link. Returns `null` for a joint attached to
     * the root of its multibody, which has no parent.
     */
    public body1(): RigidBody | null {
        const handle = this.rawSet.jointBodyHandle1(this.handle);
        return handle === undefined ? null : this.bodySet.get(handle);
    }

    /**
     * The second rigid-body this joint is attached to.
     */
    public body2(): RigidBody | null {
        const handle = this.rawSet.jointBodyHandle2(this.handle);
        return handle === undefined ? null : this.bodySet.get(handle);
    }

    /**
     * The type of this joint given as a string.
     */
    public type(): JointType {
        return this.rawSet.jointType(this.handle) as number as JointType;
    }

    /**
     * The rotation quaternion that aligns this joint's first local axis to the `x` axis.
     *
     * @param target - Optional target object to write the result to (avoids allocation).
     */
    public frameX1(target?: Rotation): Rotation {
        return RotationOps.fromRaw(this.rawSet.jointFrameX1(this.handle), target)!;
    }

    /**
     * The rotation matrix that aligns this joint's second local axis to the `x` axis.
     *
     * @param target - Optional target object to write the result to (avoids allocation).
     */
    public frameX2(target?: Rotation): Rotation {
        return RotationOps.fromRaw(this.rawSet.jointFrameX2(this.handle), target)!;
    }

    /**
     * The position of the first anchor of this joint.
     *
     * The first anchor gives the position of the application point on the
     * local frame of the first rigid-body it is attached to.
     *
     * @param target - Optional target object to write the result to (avoids allocation).
     */
    public anchor1(target?: Vector): Vector {
        return VectorOps.fromRaw(this.rawSet.jointAnchor1(this.handle), target)!;
    }

    /**
     * The position of the second anchor of this joint.
     *
     * The second anchor gives the position of the application point on the
     * local frame of the second rigid-body it is attached to.
     *
     * @param target - Optional target object to write the result to (avoids allocation).
     */
    public anchor2(target?: Vector): Vector {
        return VectorOps.fromRaw(this.rawSet.jointAnchor2(this.handle), target)!;
    }

    /**
     * Controls whether contacts are computed between colliders attached
     * to the rigid-bodies linked by this joint.
     */
    public setContactsEnabled(enabled: boolean) {
        this.rawSet.jointSetContactsEnabled(this.handle, enabled);
    }

    /**
     * Indicates if contacts are enabled between colliders attached
     * to the rigid-bodies linked by this joint.
     */
    public contactsEnabled(): boolean {
        return this.rawSet.jointContactsEnabled(this.handle);
    }
}

export class UnitMultibodyJoint extends MultibodyJoint {
    /**
     * The axis left free by this joint.
     */
    protected rawAxis(): RawJointAxis {
        throw new Error("rawAxis must be implemented by subclasses");
    }

    /**
     * Are the limits enabled for this joint?
     */
    public limitsEnabled(): boolean {
        return this.rawSet.jointLimitsEnabled(this.handle, this.rawAxis());
    }

    /**
     * The min limit of this joint.
     */
    public limitsMin(): number {
        return this.rawSet.jointLimitsMin(this.handle, this.rawAxis());
    }

    /**
     * The max limit of this joint.
     */
    public limitsMax(): number {
        return this.rawSet.jointLimitsMax(this.handle, this.rawAxis());
    }

    /**
     * Sets the limits of this joint.
     *
     * @param min - The minimum bound of this joint’s free coordinate.
     * @param max - The maximum bound of this joint’s free coordinate.
     */
    public setLimits(min: number, max: number) {
        this.rawSet.jointSetLimits(this.handle, this.rawAxis(), min, max);
    }

    public configureMotorModel(model: MotorModel) {
        this.rawSet.jointConfigureMotorModel(
            this.handle,
            this.rawAxis(),
            model as number as RawMotorModel,
        );
    }

    /**
     * Sets the maximum force (or torque, for an angular axis) the motor of this
     * joint can deliver.
     *
     * @param maxForce - The maximum force the motor can deliver.
     */
    public setMotorMaxForce(maxForce: number) {
        this.rawSet.jointSetMotorMaxForce(this.handle, this.rawAxis(), maxForce);
    }

    public configureMotorVelocity(targetVel: number, factor: number) {
        this.rawSet.jointConfigureMotorVelocity(this.handle, this.rawAxis(), targetVel, factor);
    }

    public configureMotorPosition(targetPos: number, stiffness: number, damping: number) {
        this.rawSet.jointConfigureMotorPosition(
            this.handle,
            this.rawAxis(),
            targetPos,
            stiffness,
            damping,
        );
    }

    public configureMotor(
        targetPos: number,
        targetVel: number,
        stiffness: number,
        damping: number,
    ) {
        this.rawSet.jointConfigureMotor(
            this.handle,
            this.rawAxis(),
            targetPos,
            targetVel,
            stiffness,
            damping,
        );
    }
}

export class FixedMultibodyJoint extends MultibodyJoint {}

export class PrismaticMultibodyJoint extends UnitMultibodyJoint {
    public rawAxis(): RawJointAxis {
        return RawJointAxis.LinX;
    }
}

export class RevoluteMultibodyJoint extends UnitMultibodyJoint {
    public rawAxis(): RawJointAxis {
        return RawJointAxis.AngX;
    }
}

export class SphericalMultibodyJoint extends MultibodyJoint {
    /**
     * Sets the motor model of one of this joint's angular axes.
     *
     * @param axis - The angular axis (`JointAxis.AngX/AngY/AngZ`) to configure.
     * @param model - The motor model to apply to that axis.
     */
    public configureMotorModel(axis: JointAxis, model: MotorModel) {
        this.rawSet.jointConfigureMotorModel(
            this.handle,
            axis as number as RawJointAxis,
            model as number as RawMotorModel,
        );
    }

    /**
     * Sets the maximum torque the motor of the given angular axis can deliver.
     *
     * @param axis - The angular axis (`JointAxis.AngX/AngY/AngZ`) to configure.
     * @param maxForce - The maximum torque the axis motor can deliver.
     */
    public setMotorMaxForce(axis: JointAxis, maxForce: number) {
        this.rawSet.jointSetMotorMaxForce(this.handle, axis as number as RawJointAxis, maxForce);
    }

    /**
     * Makes the motor of the given angular axis target a specific angular velocity.
     *
     * @param axis - The angular axis (`JointAxis.AngX/AngY/AngZ`) to configure.
     * @param targetVel - The target angular velocity along the axis, in radians per second.
     * @param factor - The strength used to reach the target velocity (a damping coefficient).
     */
    public configureMotorVelocity(axis: JointAxis, targetVel: number, factor: number) {
        this.rawSet.jointConfigureMotorVelocity(
            this.handle,
            axis as number as RawJointAxis,
            targetVel,
            factor,
        );
    }

    /**
     * Makes the motor of the given angular axis target a specific angle.
     *
     * @param axis - The angular axis (`JointAxis.AngX/AngY/AngZ`) to configure.
     * @param targetPos - The target angle along the axis, in radians.
     * @param stiffness - The motor's stiffness (how strongly it pulls towards the target).
     * @param damping - The motor's damping (how strongly it resists velocity).
     */
    public configureMotorPosition(
        axis: JointAxis,
        targetPos: number,
        stiffness: number,
        damping: number,
    ) {
        this.rawSet.jointConfigureMotorPosition(
            this.handle,
            axis as number as RawJointAxis,
            targetPos,
            stiffness,
            damping,
        );
    }

    /**
     * Configures both the target angle and target angular velocity of the motor of
     * the given angular axis.
     *
     * @param axis - The angular axis (`JointAxis.AngX/AngY/AngZ`) to configure.
     * @param targetPos - The target angle along the axis, in radians.
     * @param targetVel - The target angular velocity along the axis, in radians per second.
     * @param stiffness - The motor's stiffness (how strongly it pulls towards the target).
     * @param damping - The motor's damping (how strongly it resists velocity).
     */
    public configureMotor(
        axis: JointAxis,
        targetPos: number,
        targetVel: number,
        stiffness: number,
        damping: number,
    ) {
        this.rawSet.jointConfigureMotor(
            this.handle,
            axis as number as RawJointAxis,
            targetPos,
            targetVel,
            stiffness,
            damping,
        );
    }
}
