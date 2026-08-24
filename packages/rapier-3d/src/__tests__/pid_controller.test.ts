import RAPIER, {init} from "@alexandernanberg/rapier3d/compat";
import {describe, test, expect, beforeAll} from "vitest";
import {_v} from "./_target";

const NO_GRAVITY = {x: 0, y: 0, z: 0};

beforeAll(async () => {
    await init();
});

/**
 * The PID correction is `kp * positionError + kd * velocityError + ki * integral`,
 * so each gain can be isolated by zeroing the other two. `setKi`/`setKd` used to
 * both write the proportional gain, which these would not have caught by looking
 * at `setKp` alone.
 */
describe("pid controller", () => {
    function bodyAtOrigin() {
        const world = new RAPIER.World(NO_GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        return {world, body};
    }

    test("each gain drives its own term of the correction", () => {
        const {world, body} = bodyAtOrigin();
        const controller = world.createPidController(0, 0, 0, RAPIER.PidAxesMask.All);
        const target = {x: 1, y: 2, z: 3};
        const zero = {x: 0, y: 0, z: 0};

        // No gains: nothing to correct with.
        expect(controller.linearCorrection(body, target, zero, _v())).toEqual(zero);

        // Proportional: the position error is `target - translation`.
        controller.setKp(2, RAPIER.PidAxesMask.All);
        let correction = controller.linearCorrection(body, target, zero, _v());
        expect(correction.x).toBeCloseTo(2, 5);
        expect(correction.y).toBeCloseTo(4, 5);
        expect(correction.z).toBeCloseTo(6, 5);

        // Derivative: the velocity error is `targetLinvel - linvel`, on every axis.
        controller.setKp(0, RAPIER.PidAxesMask.All);
        controller.setKd(3, RAPIER.PidAxesMask.All);
        correction = controller.linearCorrection(body, zero, target, _v());
        expect(correction.x).toBeCloseTo(3, 5);
        expect(correction.y).toBeCloseTo(6, 5);
        expect(correction.z).toBeCloseTo(9, 5);

        world.free();
    });

    test("the integral gain accumulates the position error over time", () => {
        const {world, body} = bodyAtOrigin();
        const controller = world.createPidController(0, 0, 0, RAPIER.PidAxesMask.All);
        const target = {x: 1, y: 0, z: 0};
        const zero = {x: 0, y: 0, z: 0};

        controller.setKi(1, RAPIER.PidAxesMask.All);

        const dt = world.integrationParameters.dt;
        const first = controller.linearCorrection(body, target, zero, _v()).x;
        const second = controller.linearCorrection(body, target, zero, _v()).x;

        expect(first).toBeCloseTo(dt, 5);
        expect(second).toBeCloseTo(2 * dt, 5);

        controller.resetIntegrals();
        expect(controller.linearCorrection(body, target, zero, _v()).x).toBeCloseTo(dt, 5);

        world.free();
    });

    test("a mask restricts the correction to the selected axes", () => {
        const {world, body} = bodyAtOrigin();
        const controller = world.createPidController(0, 0, 0, RAPIER.PidAxesMask.LinX);
        const target = {x: 1, y: 1, z: 1};

        controller.setKp(1, RAPIER.PidAxesMask.All);
        const correction = controller.linearCorrection(body, target, {x: 0, y: 0, z: 0}, _v());

        expect(correction.x).toBeCloseTo(1, 5);
        expect(correction.y).toBeCloseTo(0, 5);
        expect(correction.z).toBeCloseTo(0, 5);

        world.free();
    });
});
