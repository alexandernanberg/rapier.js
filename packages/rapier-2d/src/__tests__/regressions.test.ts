import RAPIER, {init} from "@alexandernanberg/rapier2d/compat";
import {describe, test, expect, beforeAll} from "vitest";

const GRAVITY = {x: 0, y: -9.81};

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
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 10));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        // A step makes the buffer live, so the getters below take the buffer path.
        world.step();
        return {world, body};
    }

    test("linvel reflects applyImpulse before the next step", () => {
        const {world, body} = fallingBody();
        const before = body.linvel();

        body.applyImpulse({x: 10, y: 0}, true);

        const after = body.linvel();
        expect(after.x).toBeCloseTo(before.x + 10 / body.mass(), 5);
        expect(after.y).toBeCloseTo(before.y, 5);
        world.free();
    });

    test("angvel reflects applyTorqueImpulse before the next step", () => {
        const {world, body} = fallingBody();
        expect(body.angvel()).toBeCloseTo(0, 5);

        body.applyTorqueImpulse(1, true);

        expect(body.angvel()).toBeGreaterThan(0);
        world.free();
    });

    test("velocity reads zero right after sleep()", () => {
        const {world, body} = fallingBody();
        expect(body.linvel().y).toBeLessThan(0);

        body.sleep();

        expect(body.linvel()).toEqual({x: 0, y: 0});
        expect(body.angvel()).toBe(0);
        world.free();
    });

    test("velocity reads zero right after switching to a fixed body", () => {
        const {world, body} = fallingBody();
        expect(body.linvel().y).toBeLessThan(0);

        body.setBodyType(RAPIER.RigidBodyType.Fixed, true);

        expect(body.linvel()).toEqual({x: 0, y: 0});
        world.free();
    });
});

describe("body tuning", () => {
    test("translations can be locked on one axis only", () => {
        const world = new RAPIER.World(GRAVITY);
        const spawn = () => {
            const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 5));
            world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
            return body;
        };

        // X free, Y locked: an X impulse moves it but gravity must not.
        const slider = spawn();
        slider.restrictTranslations(true, false, true);
        slider.applyImpulse({x: 1, y: 0}, true);

        // X locked, Y free: the same impulse does nothing but it still falls.
        const faller = spawn();
        faller.setEnabledTranslations(false, true, true);
        faller.applyImpulse({x: 1, y: 0}, true);

        for (let i = 0; i < 30; i++) world.step();

        expect(slider.translation().y).toBeCloseTo(5, 5);
        expect(slider.translation().x).toBeGreaterThan(0);
        expect(faller.translation().x).toBeCloseTo(0, 5);
        expect(faller.translation().y).toBeLessThan(5);

        world.free();
    });
});

describe("joints", () => {
    test("a joint with a zero axis is rejected with a readable error", () => {
        const world = new RAPIER.World(GRAVITY);
        const a = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        const b = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const zero = {x: 0, y: 0};

        expect(() =>
            world.createImpulseJoint(RAPIER.JointData.prismatic(zero, zero, zero), a, b, true),
        ).toThrow(/axis/);
        world.free();
    });
});

describe("shapes", () => {
    test("a halfspace with a zero normal is rejected", () => {
        const world = new RAPIER.World(GRAVITY);
        expect(() => world.createCollider(RAPIER.ColliderDesc.halfspace({x: 0, y: 0}))).toThrow(
            /normal/,
        );
        world.free();
    });

    test("a trimesh collider can be created without flags", () => {
        const world = new RAPIER.World(GRAVITY);
        const vertices = new Float32Array([0, 0, 1, 0, 0, 1]);
        const indices = new Uint32Array([0, 1, 2]);
        const collider = world.createCollider(RAPIER.ColliderDesc.trimesh(vertices, indices));

        expect(collider.shape.type).toBe(RAPIER.ShapeType.TriMesh);
        expect((collider.shape as InstanceType<typeof RAPIER.TriMesh>).flags).toBeUndefined();
        world.free();
    });

    test("polyline colliders accept a null index buffer", () => {
        const world = new RAPIER.World(GRAVITY);
        const vertices = new Float32Array([0, 0, 1, 0, 1, 1]);
        const collider = world.createCollider(RAPIER.ColliderDesc.polyline(vertices, null));

        expect(collider.vertices()).toEqual(vertices);
        expect(collider.indices()).toEqual(new Uint32Array([0, 1, 1, 2]));
        world.free();
    });
});

describe("snapshots", () => {
    test("a restored parentless collider has no parent", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 5));
        const attached = world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        const ground = world.createCollider(RAPIER.ColliderDesc.cuboid(10, 0.1));
        world.step();

        const restored = RAPIER.World.restoreSnapshot(world.takeSnapshot())!;

        // Before the fix the missing parent aliased to arena index 0, i.e. `body`.
        expect(restored.getCollider(ground.handle)!.parent()).toBeNull();
        expect(restored.getCollider(attached.handle)!.parent()!.handle).toBe(body.handle);
        restored.free();
        world.free();
    });
});
