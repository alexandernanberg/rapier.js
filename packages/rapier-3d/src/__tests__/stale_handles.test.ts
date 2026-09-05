import RAPIER, {init} from "@alexandernanberg/rapier3d/compat";
import {describe, test, expect, beforeAll} from "vitest";

beforeAll(async () => {
    await init();
});

// Rapier recycles the arena index of a removed entity for the next one it
// inserts, so a handle kept past its entity's removal names a slot that soon
// belongs to someone else. These cover the JS-side bookkeeping around that.
describe("Stale handles", () => {
    test("a stale body handle does not resolve to the body that recycled its index", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const a = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(1, 0, 0));
        const staleHandle = a.handle;
        world.removeRigidBody(a);
        const b = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(2, 0, 0));

        // Same arena index, bumped generation.
        expect(b.handle).not.toBe(staleHandle);
        expect(world.getRigidBody(staleHandle)).toBeNull();
        expect(world.bodies.contains(staleHandle)).toBe(false);
        expect(world.getRigidBody(b.handle)).toBe(b);
        expect(a.isValid()).toBe(false);
        expect(b.isValid()).toBe(true);
        world.free();
    });

    test("a stale collider handle does not resolve to the collider that recycled its index", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const a = world.createCollider(RAPIER.ColliderDesc.ball(1));
        const staleHandle = a.handle;
        world.removeCollider(a, true);
        const b = world.createCollider(RAPIER.ColliderDesc.ball(1));

        expect(world.getCollider(staleHandle)).toBeNull();
        expect(world.colliders.contains(staleHandle)).toBe(false);
        expect(world.getCollider(b.handle)).toBe(b);
        world.free();
    });

    test("removing an already removed body or collider is a no-op", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const a = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(1, 0, 0));
        const ca = world.createCollider(RAPIER.ColliderDesc.ball(1), a);
        world.removeRigidBody(a);
        const b = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(2, 0, 0));
        const cb = world.createCollider(RAPIER.ColliderDesc.ball(1), b);

        // Both used to hit the recycled slot: the body removal trapped the module
        // on the stale handle, the collider removal detached `cb` instead.
        world.removeRigidBody(a);
        world.removeCollider(ca, true);
        world.removeImpulseJoint(
            {handle: staleJointHandle(world)} as unknown as RAPIER.ImpulseJoint,
            true,
        );

        expect(world.bodies.len()).toBe(1);
        expect(world.colliders.len()).toBe(1);
        expect(b.isValid()).toBe(true);
        expect(cb.isValid()).toBe(true);
        expect(cb.translation().x).toBeCloseTo(2);
        world.step();
        expect(world.getRigidBody(b.handle)).toBe(b);
        world.free();
    });

    test("accessors on a removed body throw a JS error and leave the world usable", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(1, 0, 0));
        const collider = world.createCollider(RAPIER.ColliderDesc.ball(1), body);
        world.removeRigidBody(body);

        // A Rust-side `expect` on the stale handle would be a WASM trap that
        // also leaves the set's borrow flag stuck; these have to be plain errors.
        expect(() => body.mass()).toThrow(/removed/);
        expect(() => body.isSleeping()).toThrow(/removed/);
        expect(() => body.setLinvel({x: 1, y: 0, z: 0}, true)).toThrow(/removed/);
        expect(() => body.translation()).toThrow(/removed/);
        expect(() => collider.isSensor()).toThrow(/removed/);
        expect(() => collider.setFriction(0.5)).toThrow(/removed/);
        expect(body.isValid()).toBe(false);
        expect(collider.isValid()).toBe(false);

        const other = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(2, 0, 0));
        world.createCollider(RAPIER.ColliderDesc.ball(1), other);
        world.step();
        expect(other.translation().y).toBeLessThan(0);
        world.free();
    });

    test("accessors on a removed joint throw a JS error", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const a = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(1, 0, 0));
        const b = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(2, 0, 0));
        const params = RAPIER.JointData.fixed(
            {x: 0, y: 0, z: 0},
            {x: 0, y: 0, z: 0, w: 1},
            {x: 0, y: 0, z: 0},
            {x: 0, y: 0, z: 0, w: 1},
        );
        const joint = world.createImpulseJoint(params, a, b, true);
        world.removeImpulseJoint(joint, true);
        expect(joint.isValid()).toBe(false);
        expect(() => joint.anchor1()).toThrow(/removed/);

        const joint2 = world.createImpulseJoint(params, a, b, true);
        // Removing the body removes its joints with it.
        world.removeRigidBody(a);
        expect(joint2.isValid()).toBe(false);
        expect(() => joint2.body1()).toThrow(/removed/);
        world.step();
        world.free();
    });
});

function staleJointHandle(world: RAPIER.World): number {
    const a = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
    const b = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
    const params = RAPIER.JointData.fixed(
        {x: 0, y: 0, z: 0},
        {x: 0, y: 0, z: 0, w: 1},
        {x: 0, y: 0, z: 0},
        {x: 0, y: 0, z: 0, w: 1},
    );
    const joint = world.createImpulseJoint(params, a, b, true);
    const handle = joint.handle;
    world.removeImpulseJoint(joint, true);
    world.removeRigidBody(a);
    world.removeRigidBody(b);
    return handle;
}
