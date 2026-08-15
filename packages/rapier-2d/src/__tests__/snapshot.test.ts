import RAPIER, {init} from "@alexandernanberg/rapier2d/compat";
import {describe, test, expect, beforeAll} from "vitest";

const GRAVITY = {x: 0, y: -9.81};

beforeAll(async () => {
    await init();
});

/**
 * `restoreSnapshot` rebuilds every JS wrapper from raw sets, which is the one
 * path where bodies and colliders are constructed without their sibling set and
 * patched up afterwards by `finalizeDeserialization`. It is also the only place
 * a `World` starts life with an unpopulated transform buffer.
 */
describe("snapshots", () => {
    function scene() {
        const world = new RAPIER.World(GRAVITY);
        const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(RAPIER.ColliderDesc.cuboid(20, 0.1), ground);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(1, 4));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        for (let i = 0; i < 10; i++) world.step();
        return {world, body};
    }

    test("a restored world reproduces the snapshotted transforms", () => {
        const {world, body} = scene();
        const before = body.translation();

        const snapshot = world.takeSnapshot();
        expect(snapshot.byteLength).toBeGreaterThan(0);

        const restored = RAPIER.World.restoreSnapshot(snapshot)!;
        expect(restored).not.toBeNull();

        const restoredBody = restored.getRigidBody(body.handle)!;
        expect(restoredBody).not.toBeNull();
        expect(restoredBody.translation().x).toBeCloseTo(before.x, 5);
        expect(restoredBody.translation().y).toBeCloseTo(before.y, 5);

        restored.free();
        world.free();
    });

    test("a restored body can still reach its colliders", () => {
        const {world, body} = scene();
        const restored = RAPIER.World.restoreSnapshot(world.takeSnapshot())!;

        // `RigidBody.collider()` reads through the collider set that only
        // `finalizeDeserialization` can supply on this path.
        const restoredBody = restored.getRigidBody(body.handle)!;
        expect(restoredBody.numColliders()).toBe(1);
        expect(restoredBody.collider(0)).not.toBeNull();

        restored.free();
        world.free();
    });

    test("a restored world keeps simulating in step with the original", () => {
        const {world, body} = scene();
        const restored = RAPIER.World.restoreSnapshot(world.takeSnapshot())!;

        for (let i = 0; i < 30; i++) {
            world.step();
            restored.step();
        }

        const restoredBody = restored.getRigidBody(body.handle)!;
        expect(restoredBody.translation().x).toBeCloseTo(body.translation().x, 4);
        expect(restoredBody.translation().y).toBeCloseTo(body.translation().y, 4);

        restored.free();
        world.free();
    });
});
