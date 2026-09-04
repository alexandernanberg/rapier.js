import RAPIER, {init} from "@alexandernanberg/rapier3d/compat";
import {describe, test, expect, beforeAll} from "vitest";

const GRAVITY = {x: 0, y: -9.81, z: 0};

beforeAll(async () => {
    await init();
});

/**
 * Between steps, pose and velocity getters read straight out of the transform
 * buffer. Every WASM-side mutation therefore has to be published into the
 * buffer immediately, not just on the next `step()`.
 */
describe("transform buffer coherence", () => {
    function fallingBody() {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 10, 0));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        // A step makes the buffer live, so the getters below take the buffer path.
        world.step();
        return {world, body};
    }

    test("linvel reflects applyImpulse before the next step", () => {
        const {world, body} = fallingBody();
        const before = body.linvel();

        body.applyImpulse({x: 10, y: 0, z: 0}, true);

        const after = body.linvel();
        expect(after.x).toBeCloseTo(before.x + 10 / body.mass(), 5);
        expect(after.y).toBeCloseTo(before.y, 5);
        world.free();
    });

    test("angvel reflects applyTorqueImpulse before the next step", () => {
        const {world, body} = fallingBody();
        expect(body.angvel().y).toBeCloseTo(0, 5);

        body.applyTorqueImpulse({x: 0, y: 1, z: 0}, true);

        expect(body.angvel().y).toBeGreaterThan(0);
        world.free();
    });

    test("velocity reads zero right after sleep()", () => {
        const {world, body} = fallingBody();
        expect(body.linvel().y).toBeLessThan(0);

        body.sleep();

        expect(body.linvel()).toEqual({x: 0, y: 0, z: 0});
        expect(body.angvel()).toEqual({x: 0, y: 0, z: 0});
        world.free();
    });

    test("velocity reads zero right after switching to a fixed body", () => {
        const {world, body} = fallingBody();
        expect(body.linvel().y).toBeLessThan(0);

        body.setBodyType(RAPIER.RigidBodyType.Fixed, true);

        expect(body.linvel()).toEqual({x: 0, y: 0, z: 0});
        world.free();
    });

    test("a rejected (non-normalized) rotation is not written into the buffer", () => {
        const {world, body} = fallingBody();

        body.setRotation({x: 0, y: 0, z: 0, w: 0}, true);

        expect(body.rotation()).toEqual({x: 0, y: 0, z: 0, w: 1});
        // The buffer read has to agree with WASM.
        world.step();
        expect(body.rotation().w).toBeCloseTo(1, 5);
        world.free();
    });

    test("setTransform with a rejected rotation still applies the translation", () => {
        const {world, body} = fallingBody();

        body.setTransform({x: 3, y: 4, z: 5}, {x: 0, y: 0, z: 0, w: 0}, true);

        expect(body.translation()).toEqual({x: 3, y: 4, z: 5});
        expect(body.rotation()).toEqual({x: 0, y: 0, z: 0, w: 1});
        world.free();
    });
});

describe("joints", () => {
    test("a spherical joint reports its own type", () => {
        const world = new RAPIER.World(GRAVITY);
        const a = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        const b = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(1.5, 0, 0));
        const joint = world.createImpulseJoint(
            RAPIER.JointData.spherical({x: 0, y: 0, z: 0}, {x: -1.5, y: 0, z: 0}),
            a,
            b,
            true,
        );

        expect(joint.type()).toBe(RAPIER.JointType.Spherical);
        expect(joint).toBeInstanceOf(RAPIER.SphericalImpulseJoint);
        world.free();
    });

    test("a joint with a zero axis is rejected with a readable error", () => {
        const world = new RAPIER.World(GRAVITY);
        const a = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        const b = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const zero = {x: 0, y: 0, z: 0};

        expect(() =>
            world.createImpulseJoint(RAPIER.JointData.revolute(zero, zero, zero), a, b, true),
        ).toThrow(/axis/);
        expect(() =>
            world.createImpulseJoint(RAPIER.JointData.prismatic(zero, zero, zero), a, b, true),
        ).toThrow(/axis/);
        world.free();
    });

    test("a generic joint can be created repeatedly without leaking", () => {
        const world = new RAPIER.World(GRAVITY);
        const a = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        const b = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const data = RAPIER.JointData.generic(
            {x: 0, y: 0, z: 0},
            {x: 0, y: 0, z: 0},
            {x: 1, y: 0, z: 0},
            RAPIER.JointAxesMask.LinX | RAPIER.JointAxesMask.LinY,
        );

        for (let i = 0; i < 10; i++) {
            const joint = world.createImpulseJoint(data, a, b, true);
            expect(joint.type()).toBe(RAPIER.JointType.Generic);
            world.removeImpulseJoint(joint, true);
        }
        world.free();
    });
});

describe("shapes", () => {
    test("a halfspace with a zero normal is rejected", () => {
        const world = new RAPIER.World(GRAVITY);
        expect(() =>
            world.createCollider(RAPIER.ColliderDesc.halfspace({x: 0, y: 0, z: 0})),
        ).toThrow(/normal/);
        world.free();
    });

    test("round cone colliders expose radius and half height", () => {
        const world = new RAPIER.World(GRAVITY);
        const collider = world.createCollider(RAPIER.ColliderDesc.roundCone(1.5, 0.75, 0.1));

        expect(collider.radius()).toBeCloseTo(0.75, 5);
        expect(collider.halfHeight()).toBeCloseTo(1.5, 5);
        collider.setRadius(1);
        collider.setHalfHeight(2);
        expect(collider.radius()).toBeCloseTo(1, 5);
        expect(collider.halfHeight()).toBeCloseTo(2, 5);
        world.free();
    });

    test("polyline colliders expose their vertices", () => {
        const world = new RAPIER.World(GRAVITY);
        const vertices = new Float32Array([0, 0, 0, 1, 0, 0, 1, 1, 0]);
        const collider = world.createCollider(RAPIER.ColliderDesc.polyline(vertices));

        expect(collider.vertices()).toEqual(vertices);
        expect(collider.indices()).toEqual(new Uint32Array([0, 1, 1, 2]));
        world.free();
    });
});

describe("snapshots", () => {
    test("a restored parentless collider has no parent", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 5, 0));
        const attached = world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        const ground = world.createCollider(RAPIER.ColliderDesc.cuboid(10, 0.1, 10));
        world.step();

        const restored = RAPIER.World.restoreSnapshot(world.takeSnapshot())!;

        // Before the fix the missing parent aliased to arena index 0, i.e. `body`.
        expect(restored.getCollider(ground.handle)!.parent()).toBeNull();
        expect(restored.getCollider(attached.handle)!.parent()!.handle).toBe(body.handle);
        restored.free();
        world.free();
    });
});

describe("vehicle controller", () => {
    test("an airborne wheel has no ground object", () => {
        const world = new RAPIER.World(GRAVITY);
        world.createCollider(RAPIER.ColliderDesc.cuboid(20, 0.1, 20));
        const chassis = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 50, 0),
        );
        world.createCollider(RAPIER.ColliderDesc.cuboid(1, 0.25, 2), chassis);
        const vehicle = world.createVehicleController(chassis);
        vehicle.addWheel({x: 1, y: -0.25, z: 1}, {x: 0, y: -1, z: 0}, {x: 1, y: 0, z: 0}, 0.5, 0.3);

        vehicle.updateVehicle(1 / 60);

        expect(vehicle.wheelIsInContact(0)).toBe(false);
        expect(vehicle.wheelGroundObject(0)).toBeNull();
        expect(vehicle.wheelGroundObject(7)).toBeNull();

        world.removeVehicleController(vehicle);
        world.free();
    });
});
