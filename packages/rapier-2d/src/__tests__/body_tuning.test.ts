import RAPIER, {init} from "@alexandernanberg/rapier2d/compat";
import {describe, test, expect, beforeAll} from "vitest";

const GRAVITY = {x: 0, y: -9.81};

beforeAll(async () => {
    await init();
});

/**
 * The knobs that change how a body integrates — locked axes, damping, gravity
 * scale, CCD — are all set through `RigidBodyDesc` or setters that are never
 * exercised. Each one is checked by its effect on the simulation rather than by
 * reading the value back.
 */
describe("body tuning", () => {
    test("locked translations pin the body in place", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 5).lockTranslations(),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        for (let i = 0; i < 30; i++) world.step();

        expect(body.translation().y).toBeCloseTo(5, 5);

        // Re-enabling the axes lets it fall again.
        body.setEnabledTranslations(true, true, true);
        for (let i = 0; i < 30; i++) world.step();
        expect(body.translation().y).toBeLessThan(5);

        world.free();
    });

    test("locked rotations keep the body upright", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 5).lockRotations(),
        );
        world.createCollider(RAPIER.ColliderDesc.cuboid(0.5, 0.5), body);
        body.applyTorqueImpulse(50, true);

        for (let i = 0; i < 30; i++) world.step();

        expect(body.rotation()).toBeCloseTo(0, 5);

        world.free();
    });

    test("gravityScale scales how fast a body falls", () => {
        const world = new RAPIER.World(GRAVITY);
        const slow = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 5).setGravityScale(0.5),
        );
        const fast = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(2, 5).setGravityScale(2),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(0.1), slow);
        world.createCollider(RAPIER.ColliderDesc.ball(0.1), fast);

        for (let i = 0; i < 30; i++) world.step();

        expect(slow.translation().y).toBeGreaterThan(fast.translation().y);
        expect(slow.gravityScale()).toBeCloseTo(0.5, 6);

        world.free();
    });

    test("linear damping slows a body down", () => {
        const world = new RAPIER.World({x: 0, y: 0});
        const damped = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setLinvel(10, 0).setLinearDamping(5),
        );
        const free = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 5).setLinvel(10, 0),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(0.1), damped);
        world.createCollider(RAPIER.ColliderDesc.ball(0.1), free);

        for (let i = 0; i < 30; i++) world.step();

        expect(damped.linvel().x).toBeLessThan(free.linvel().x);
        expect(damped.linearDamping()).toBeCloseTo(5, 6);

        world.free();
    });

    test("a body can be put to sleep and woken up", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        world.step();

        body.sleep();
        world.step();
        expect(body.isSleeping()).toBe(true);

        body.wakeUp();
        world.step();
        expect(body.isSleeping()).toBe(false);

        world.free();
    });

    test("setBodyType switches a body between dynamic and fixed", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 5));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        body.setBodyType(RAPIER.RigidBodyType.Fixed, true);
        expect(body.bodyType()).toBe(RAPIER.RigidBodyType.Fixed);

        for (let i = 0; i < 30; i++) world.step();
        expect(body.translation().y).toBeCloseTo(5, 5);

        world.free();
    });
});

describe("continuous collision detection", () => {
    test("a fast body does not tunnel through a thin wall when CCD is on", () => {
        const world = new RAPIER.World({x: 0, y: 0});
        const wall = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(RAPIER.ColliderDesc.cuboid(0.05, 5), wall);

        const bullet = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic()
                .setTranslation(-10, 0)
                .setLinvel(2000, 0)
                .setCcdEnabled(true),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(0.1), bullet);
        expect(bullet.isCcdEnabled()).toBe(true);

        for (let i = 0; i < 30; i++) world.step();

        expect(bullet.translation().x).toBeLessThan(0.5);

        world.free();
    });
});

describe("debug render", () => {
    test("debugRender returns one colour per vertex", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(RAPIER.ColliderDesc.cuboid(1, 1), body);
        world.step();

        const buffers = world.debugRender();
        expect(buffers.vertices.length).toBeGreaterThan(0);
        // 2 floats per vertex in 2D, 4 colour components per vertex.
        expect(buffers.colors.length / 4).toBe(buffers.vertices.length / 2);

        world.free();
    });
});

describe("handle lifetime", () => {
    test("a stale handle resolves to the body that recycled its slot", () => {
        const world = new RAPIER.World(GRAVITY);
        const first = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(1, 1));
        const staleHandle = first.handle;

        world.removeRigidBody(first);
        expect(world.getRigidBody(staleHandle)).toBeNull();

        const second = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(9, 9));
        expect(second.handle).not.toBe(staleHandle);

        // Coarena keys on the arena index alone and ignores the generation bits,
        // so a stale handle silently resolves to whichever body took the slot.
        // This pins the current behaviour rather than endorsing it.
        expect(world.getRigidBody(staleHandle)).toBe(second);

        world.free();
    });
});
