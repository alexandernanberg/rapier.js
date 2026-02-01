import {IntegrationParameters, JointAxesMask, RigidBody, RigidBodySet} from "../dynamics";
import {Rotation, RotationOps, Vector, VectorOps} from "../math";
import {RawPidController} from "../raw";

/**
 * @deprecated Use `JointAxesMask` instead. This is an alias for backwards compatibility.
 */
export const PidAxesMask = JointAxesMask;
/**
 * @deprecated Use `JointAxesMask` instead. This is an alias for backwards compatibility.
 */
export type PidAxesMask = JointAxesMask;

/**
 * A controller for controlling dynamic bodies using the
 * Proportional-Integral-Derivative correction model.
 */
export class PidController {
    private raw: RawPidController;

    private params: IntegrationParameters;
    private bodies: RigidBodySet;

    constructor(
        params: IntegrationParameters,
        bodies: RigidBodySet,
        kp: number,
        ki: number,
        kd: number,
        axes: PidAxesMask,
    ) {
        this.params = params;
        this.bodies = bodies;
        this.raw = new RawPidController(kp, ki, kd, axes);
    }

    /** @internal */
    public free() {
        if (!!this.raw) {
            this.raw.free();
        }

        this.raw = undefined!;
    }

    public setKp(kp: number, axes: PidAxesMask) {
        this.raw.set_kp(kp, axes);
    }

    public setKi(ki: number, axes: PidAxesMask) {
        this.raw.set_kp(ki, axes);
    }

    public setKd(kd: number, axes: PidAxesMask) {
        this.raw.set_kp(kd, axes);
    }

    public setAxes(axes: PidAxesMask) {
        this.raw.set_axes_mask(axes);
    }

    public resetIntegrals() {
        this.raw.reset_integrals();
    }

    public applyLinearCorrection(body: RigidBody, targetPosition: Vector, targetLinvel: Vector) {
        let rawPos = VectorOps.intoRaw(targetPosition);
        let rawVel = VectorOps.intoRaw(targetLinvel);
        this.raw.apply_linear_correction(
            this.params.dt,
            this.bodies.raw,
            body.handle,
            rawPos,
            rawVel,
        );
        rawPos.free();
        rawVel.free();
    }

    public applyAngularCorrection(body: RigidBody, targetRotation: Rotation, targetAngVel: Vector) {
        let rawPos = RotationOps.intoRaw(targetRotation);
        let rawVel = VectorOps.intoRaw(targetAngVel);
        this.raw.apply_angular_correction(
            this.params.dt,
            this.bodies.raw,
            body.handle,
            rawPos,
            rawVel,
        );
        rawPos.free();
        rawVel.free();
    }

    public linearCorrection(body: RigidBody, targetPosition: Vector, targetLinvel: Vector): Vector {
        let rawPos = VectorOps.intoRaw(targetPosition);
        let rawVel = VectorOps.intoRaw(targetLinvel);
        let correction = this.raw.linear_correction(
            this.params.dt,
            this.bodies.raw,
            body.handle,
            rawPos,
            rawVel,
        );
        rawPos.free();
        rawVel.free();

        return VectorOps.fromRaw(correction)!;
    }

    public angularCorrection(
        body: RigidBody,
        targetRotation: Rotation,
        targetAngVel: Vector,
    ): Vector {
        let rawPos = RotationOps.intoRaw(targetRotation);
        let rawVel = VectorOps.intoRaw(targetAngVel);
        let correction = this.raw.angular_correction(
            this.params.dt,
            this.bodies.raw,
            body.handle,
            rawPos,
            rawVel,
        );
        rawPos.free();
        rawVel.free();

        return VectorOps.fromRaw(correction)!;
    }
}
