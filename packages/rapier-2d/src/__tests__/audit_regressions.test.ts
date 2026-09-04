import RAPIER, {init} from "@alexandernanberg/rapier2d/compat";
import {describe, test, expect, beforeAll} from "vitest";

const GRAVITY = {x: 0, y: -9.81};

beforeAll(async () => {
    await init();
});

/**
 * Regressions for the issues found while auditing the bindings: silent
 * truncation of buffer walks, stale JS-side caches, reads through recycled
 * transform slots, and WASM traps on stale handles that should be exceptions.
 */
describe("audit regressions", () => {
    test("forEachActiveRigidBody visits every body even when the callback grows WASM memory", () => {
        const world = new RAPIER.World(GRAVITY);
        const handles = new Set<number>();
        for (let i = 0; i < 40; i++) {
            const body = world.createRigidBody(
                RAPIER.RigidBodyDesc.dynamic().setTranslation(i * 3, 10),
            );
            world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
            handles.add(body.handle);
        }
        world.step();

        const visited = new Set<number>();
        world.forEachActiveRigidBody((body) => {
            if (visited.size === 0) {
                // Force the linear memory to grow mid-walk, which detaches every
                // typed-array view onto it.
                RAPIER.reserveMemory(64 * 1024 * 1024);
            }
            visited.add(body.handle);
        });

        expect(visited).toEqual(handles);
        world.free();
    });

    test("the cached shape follows the shape mutators", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());

        const ball = world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        expect((ball.shape as RAPIER.Ball).radius).toBe(0.5);
        ball.setRadius(2);
        expect((ball.shape as RAPIER.Ball).radius).toBe(2);

        const box = world.createCollider(RAPIER.ColliderDesc.cuboid(1, 1), body);
        box.setHalfExtents({x: 2, y: 3});
        expect((box.shape as RAPIER.Cuboid).halfExtents).toEqual({x: 2, y: 3});

        const capsule = world.createCollider(RAPIER.ColliderDesc.capsule(1, 0.5), body);
        capsule.setHalfHeight(3);
        expect((capsule.shape as RAPIER.Capsule).halfHeight).toBe(3);

        const round = world.createCollider(RAPIER.ColliderDesc.roundCuboid(1, 1, 0.1), body);
        round.setRoundRadius(0.25);
        expect((round.shape as RAPIER.RoundCuboid).borderRadius).toBeCloseTo(0.25);

        world.free();
    });

    test("a removed body does not read another body's transform out of the buffer", () => {
        const world = new RAPIER.World(GRAVITY);
        const removed = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(1, 2));
        const collider = world.createCollider(RAPIER.ColliderDesc.ball(0.5), removed);
        world.step();
        expect(removed.translation()).toEqual({x: 1, y: 2});

        world.removeRigidBody(removed);
        // Recycles the arena index the removed body held.
        const successor = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(7, 8));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), successor);
        world.step();

        expect(removed.isValid()).toBe(false);
        expect(collider.isValid()).toBe(false);
        expect(successor.translation()).toEqual({x: 7, y: 8});
        expect(() => removed.translation()).toThrow();
        expect(() => collider.translation()).toThrow();
        world.free();
    });

    test("creating a collider on a removed body throws instead of trapping", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.removeRigidBody(body);

        expect(() => world.createCollider(RAPIER.ColliderDesc.ball(0.5), body)).toThrow(/removed/);

        // The world is still usable afterwards.
        const other = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const collider = world.createCollider(RAPIER.ColliderDesc.ball(0.5), other);
        world.step();
        expect(collider.isValid()).toBe(true);
        expect(world.colliders.len()).toBe(1);
        world.free();
    });

    test("collider queries against a removed collider miss instead of trapping", () => {
        const world = new RAPIER.World(GRAVITY);
        const a = world.createCollider(RAPIER.ColliderDesc.ball(0.5));
        const b = world.createCollider(RAPIER.ColliderDesc.ball(0.5).setTranslation(3, 0));
        world.removeCollider(b, false);

        expect(a.castCollider({x: 1, y: 0}, b, {x: 0, y: 0}, 0, 10, true)).toBeNull();
        expect(a.contactCollider(b, 1)).toBeNull();
        world.free();
    });

    test("a body pushed by the character controller reports its new velocity right away", () => {
        const world = new RAPIER.World(GRAVITY);
        const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(RAPIER.ColliderDesc.cuboid(20, 0.5), ground);

        const box = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(1.2, 1.0));
        world.createCollider(RAPIER.ColliderDesc.cuboid(0.5, 0.5).setDensity(1), box);

        const character = world.createRigidBody(
            RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(0, 1.0),
        );
        const characterCollider = world.createCollider(
            RAPIER.ColliderDesc.cuboid(0.3, 0.5),
            character,
        );
        // Settle: the buffer is live from here on, which is the path under test.
        world.step();
        expect(box.linvel().x).toBeCloseTo(0, 3);

        const controller = world.createCharacterController(0.01);
        controller.setApplyImpulsesToDynamicBodies(true);
        controller.setCharacterMass(50);
        controller.computeColliderMovement(characterCollider, {x: 1, y: 0});

        expect(controller.numComputedCollisions()).toBeGreaterThan(0);
        expect(box.linvel().x).toBeGreaterThan(0);
        world.free();
    });

    test("setRotation reads back the same angle before and after a step", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.step();

        // 4 radians lies outside (-π, π]: the engine stores it wrapped, and the
        // buffer used to be overwritten with the raw input until the next step.
        body.setRotation(4.0, false);
        const before = body.rotation();
        world.step();
        expect(body.rotation()).toBeCloseTo(before, 5);
        expect(Math.cos(before)).toBeCloseTo(Math.cos(4.0), 5);
        expect(Math.sin(before)).toBeCloseTo(Math.sin(4.0), 5);
        world.free();
    });

    test("bodies created in a restored world read correct transforms before its first step", () => {
        const world = new RAPIER.World(GRAVITY);
        world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(1, 2));
        world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(4, 5));
        world.step();

        const restored = RAPIER.World.restoreSnapshot(world.takeSnapshot())!;
        const spawned = restored.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(7, 8));
        const spawnedCollider = restored.createCollider(
            RAPIER.ColliderDesc.ball(0.5).setTranslation(0.5, 0),
            spawned,
        );

        const positions = () =>
            restored.bodies
                .getAll()
                .map((b) => b.translation())
                .sort((a, b) => a.x - b.x);
        const expected = [
            {x: 1, y: 2},
            {x: 4, y: 5},
            {x: 7, y: 8},
        ];
        expect(positions()).toEqual(expected);
        expect(spawnedCollider.translation()).toEqual({x: 7.5, y: 8});
        restored.step();
        expect(positions()).toEqual(expected);
        expect(spawnedCollider.translation()).toEqual({x: 7.5, y: 8});

        world.free();
        restored.free();
    });

    test("bodies spawned between steps keep reading through the buffer", () => {
        const world = new RAPIER.World(GRAVITY);
        const first = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(1, 1));
        world.step();

        // Growing the set moves the buffer; both the existing and the new body
        // must still read their own transforms.
        const spawned = [];
        for (let i = 0; i < 200; i++) {
            spawned.push(world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(i, 0)));
            world.createCollider(RAPIER.ColliderDesc.ball(0.5).setTranslation(0, 1), spawned[i]);
        }
        expect(first.translation()).toEqual({x: 1, y: 1});
        for (let i = 0; i < 200; i++) {
            expect(spawned[i].translation()).toEqual({x: i, y: 0});
            expect(spawned[i].collider(0)!.translation()).toEqual({x: i, y: 1});
        }
        world.free();
    });

    test("rope and spring joints report their own type", () => {
        const world = new RAPIER.World(GRAVITY);
        const a = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const b = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(2, 0));
        const zero = {x: 0, y: 0};

        const rope = world.createImpulseJoint(RAPIER.JointData.rope(3, zero, zero), a, b, true);
        const spring = world.createImpulseJoint(
            RAPIER.JointData.spring(1, 10, 1, zero, zero),
            a,
            b,
            true,
        );
        expect(rope.type()).toBe(RAPIER.JointType.Rope);
        expect(spring.type()).toBe(RAPIER.JointType.Spring);
        world.free();
    });

    test("the contact_natural_frequency alias reads back", () => {
        const world = new RAPIER.World(GRAVITY);
        world.integrationParameters.contact_natural_frequency = 42;
        expect(world.integrationParameters.contact_natural_frequency).toBe(42);
        expect(world.integrationParameters.contactNaturalFrequency).toBe(42);
        world.free();
    });
});
