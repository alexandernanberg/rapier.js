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

/**
 * Getters that used to hand a wasm-bindgen object across the boundary now write
 * into the scratch buffer. Values and `null` cases have to be unchanged.
 */
describe("scratch-buffer getters", () => {
    test("collider ray casts match the world-level query and miss as null", () => {
        const world = new RAPIER.World(GRAVITY);
        const vertices = new Float32Array([-1, 0, -1, 1, 0, -1, 1, 0, 1, -1, 0, 1]);
        const indices = new Uint32Array([0, 1, 2, 0, 2, 3]);
        const collider = world.createCollider(RAPIER.ColliderDesc.trimesh(vertices, indices));
        const ball = world.createCollider(RAPIER.ColliderDesc.ball(1).setTranslation(10, 0, 0));
        world.step();

        // The world-level query reports through a separate (f64) result buffer,
        // so it is an independent reference for the collider-level encoding.
        for (const ray of [
            new RAPIER.Ray({x: 0.5, y: 1, z: -0.2}, {x: 0, y: -1, z: 0}),
            new RAPIER.Ray({x: -0.5, y: 1, z: 0.2}, {x: 0, y: -1, z: 0}),
            new RAPIER.Ray({x: 10, y: 5, z: 0}, {x: 0, y: -1, z: 0}),
        ]) {
            const reference = world.castRayAndGetNormal(ray, 10, true)!;
            expect(reference).not.toBeNull();
            const hit = reference.collider.castRayAndGetNormal(ray, 10, true)!;
            expect(hit).not.toBeNull();
            expect(hit.timeOfImpact).toBeCloseTo(reference.timeOfImpact, 5);
            expect(hit.normal.x).toBeCloseTo(reference.normal.x, 5);
            expect(hit.normal.y).toBeCloseTo(reference.normal.y, 5);
            expect(hit.normal.z).toBeCloseTo(reference.normal.z, 5);
            expect(hit.featureType).toBe(reference.featureType);
            expect(hit.featureId).toBe(reference.featureId);
        }

        const down = new RAPIER.Ray({x: 0, y: 1, z: 0}, {x: 0, y: -1, z: 0});
        const missRay = new RAPIER.Ray({x: 5, y: 1, z: 5}, {x: 0, y: -1, z: 0});
        expect(collider.castRayAndGetNormal(missRay, 10, true)).toBeNull();
        expect(ball.castRayAndGetNormal(down, 10, true)).toBeNull();
        expect(collider.castRay(down, 10, true)).toBeCloseTo(1, 5);
        expect(collider.castRay(missRay, 10, true)).toBeLessThan(0);
        expect(collider.intersectsRay(down, 10)).toBe(true);
        expect(collider.intersectsRay(missRay, 10)).toBe(false);
        world.free();
    });

    test("collider point queries", () => {
        const world = new RAPIER.World(GRAVITY);
        const collider = world.createCollider(RAPIER.ColliderDesc.ball(1));

        expect(collider.containsPoint({x: 0.5, y: 0, z: 0})).toBe(true);
        expect(collider.containsPoint({x: 2, y: 0, z: 0})).toBe(false);

        const outside = collider.projectPoint({x: 3, y: 0, z: 0}, true);
        expect(outside.isInside).toBe(false);
        expect(outside.point.x).toBeCloseTo(1, 5);

        const inside = collider.projectPoint({x: 0.5, y: 0, z: 0}, false);
        expect(inside.isInside).toBe(true);
        expect(inside.point.x).toBeCloseTo(1, 5);
        world.free();
    });

    test("shape-level ray and point queries", () => {
        const ball = new RAPIER.Ball(1);
        const pos = {x: 0, y: 0, z: 0};
        const rot = {x: 0, y: 0, z: 0, w: 1};

        const hit = ball.castRayAndGetNormal(
            new RAPIER.Ray({x: 0, y: 5, z: 0}, {x: 0, y: -1, z: 0}),
            pos,
            rot,
            10,
            true,
        )!;
        expect(hit.timeOfImpact).toBeCloseTo(4, 5);
        expect(hit.normal.y).toBeCloseTo(1, 5);
        expect(
            ball.castRayAndGetNormal(
                new RAPIER.Ray({x: 5, y: 5, z: 0}, {x: 0, y: -1, z: 0}),
                pos,
                rot,
                10,
                true,
            ),
        ).toBeNull();

        const proj = ball.projectPoint(pos, rot, {x: 0, y: 3, z: 0}, true);
        expect(proj.point.y).toBeCloseTo(1, 5);
        expect(proj.isInside).toBe(false);
    });

    test("shape-specific collider getters return null for other shapes", () => {
        const world = new RAPIER.World(GRAVITY);
        const cuboid = world.createCollider(RAPIER.ColliderDesc.cuboid(1, 2, 3));
        const ball = world.createCollider(RAPIER.ColliderDesc.ball(1));
        const heights = new Float32Array([0, 0, 0, 0]);
        const field = world.createCollider(
            RAPIER.ColliderDesc.heightfield(1, 1, heights, {x: 4, y: 1, z: 6}),
        );

        expect(cuboid.halfExtents()).toEqual({x: 1, y: 2, z: 3});
        expect(ball.halfExtents()).toBeNull();
        expect(field.heightfieldScale()).toEqual({x: 4, y: 1, z: 6});
        expect(ball.heightfieldScale()).toBeNull();
        world.free();
    });

    test("inertia matrices read six symmetric elements", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.createCollider(RAPIER.ColliderDesc.cuboid(1, 2, 3).setDensity(1), body);
        world.step();

        const inertia = body.effectiveAngularInertia();
        expect(inertia.elements.length).toBe(6);
        expect(inertia.m11).toBeGreaterThan(0);
        expect(inertia.m22).toBeGreaterThan(0);
        expect(inertia.m33).toBeGreaterThan(0);
        expect(inertia.m32).toBe(inertia.m23);

        const inv = body.effectiveWorldInvInertia();
        expect(inv.m11 * inertia.m11).toBeCloseTo(1, 4);
        expect(inv.m22 * inertia.m22).toBeCloseTo(1, 4);

        // A target must be a distinct copy, not a view into the scratch buffer.
        const target = new RAPIER.SdpMatrix3(new Float32Array(6));
        expect(body.effectiveAngularInertia(target)).toBe(target);
        body.effectiveWorldInvInertia();
        expect(target.m11).toBeCloseTo(inertia.m11, 5);
        world.free();
    });

    test("vehicle wheel vectors", () => {
        const world = new RAPIER.World(GRAVITY);
        const chassis = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const vehicle = world.createVehicleController(chassis);
        vehicle.addWheel({x: 1, y: -0.25, z: 1}, {x: 0, y: -1, z: 0}, {x: 1, y: 0, z: 0}, 0.5, 0.3);

        expect(vehicle.wheelChassisConnectionPointCs(0)).toEqual({x: 1, y: -0.25, z: 1});
        expect(vehicle.wheelDirectionCs(0)).toEqual({x: 0, y: -1, z: 0});
        expect(vehicle.wheelAxleCs(0)).toEqual({x: 1, y: 0, z: 0});
        expect(vehicle.wheelAxleCs(3)).toBeNull();
        const target = {x: 0, y: 0, z: 0};
        expect(vehicle.wheelDirectionCs(0, target)).toBe(target);

        world.removeVehicleController(vehicle);
        world.free();
    });
});
