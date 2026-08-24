import RAPIER, {init} from "@alexandernanberg/rapier3d/compat";
import {describe, test, expect, beforeAll} from "vitest";
import {_v, _q} from "./_target";

beforeAll(async () => {
    await init();
});

/**
 * Exercises the collider world-space transform buffer, which is synced only for
 * colliders attached to awake bodies (plus a full rewrite on insertion), and is
 * invalidated on the JS side when a collider is mutated directly.
 */
describe("collider transform buffer sync", () => {
    test("attached collider tracks its parent body after a step", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 10, 0));
        const collider = world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        for (let i = 0; i < 10; i++) world.step();

        const b = body.translation(_v());
        const c = collider.translation(_v());
        expect(c.x).toBeCloseTo(b.x, 5);
        expect(c.y).toBeCloseTo(b.y, 5);
        expect(c.z).toBeCloseTo(b.z, 5);

        // Rotation should match the parent too.
        const br = body.rotation(_q());
        const cr = collider.rotation(_q());
        expect(cr.x).toBeCloseTo(br.x, 5);
        expect(cr.w).toBeCloseTo(br.w, 5);

        world.free();
    });

    test("collider on a sleeping body keeps its resting transform", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const floor = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(RAPIER.ColliderDesc.cuboid(10, 0.1, 10), floor);

        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 1, 0));
        const collider = world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        for (let i = 0; i < 600; i++) {
            world.step();
            if (body.isSleeping()) break;
        }
        expect(body.isSleeping()).toBe(true);

        const resting = collider.translation(_v()).y;
        for (let i = 0; i < 30; i++) {
            world.step();
            expect(collider.translation(_v()).y).toBeCloseTo(resting, 5);
        }

        world.free();
    });

    test("parentless collider keeps its position across steps", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const collider = world.createCollider(
            RAPIER.ColliderDesc.cuboid(0.5, 0.5, 0.5).setTranslation(4, 2, -3),
        );

        for (let i = 0; i < 10; i++) world.step();

        const t = collider.translation(_v());
        expect(t.x).toBeCloseTo(4, 5);
        expect(t.y).toBeCloseTo(2, 5);
        expect(t.z).toBeCloseTo(-3, 5);

        world.free();
    });

    test("collider.setTranslation is reflected immediately (buffer invalidated)", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const collider = world.createCollider(
            RAPIER.ColliderDesc.ball(0.5).setTranslation(0, 0, 0),
        );
        world.step();

        collider.setTranslation({x: 7, y: 8, z: 9});
        const t = collider.translation(_v());
        expect(t.x).toBeCloseTo(7, 5);
        expect(t.y).toBeCloseTo(8, 5);
        expect(t.z).toBeCloseTo(9, 5);

        world.free();
    });

    test("detached buffer view is re-attached", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(1, 2, 3));
        const collider = world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        world.step();

        // Reference value read via the (valid) buffer path.
        const expected = collider.translation(_v());

        // A WASM `memory.grow()` detaches the view's underlying ArrayBuffer,
        // leaving a 0-length view (reads would otherwise yield undefined). The
        // WASM-backed buffer can't be detached on demand, so install a detached
        // view to reproduce the exact state `liveBuffer()` must handle.
        const detached = new Float32Array(16);
        (detached.buffer as ArrayBuffer & {transfer(): ArrayBuffer}).transfer();
        expect(detached.byteLength).toBe(0);
        world.colliders._bufferRef.buffer = detached;

        // The buffer contents are still valid, so `liveBuffer()` must re-create
        // the view rather than fall back to the WASM path, and yield the same
        // values as before (not undefined/garbage).
        const t = collider.translation(_v());
        expect(t.x).toBeCloseTo(expected.x, 4);
        expect(t.y).toBeCloseTo(expected.y, 4);
        expect(t.z).toBeCloseTo(expected.z, 4);
        expect(world.colliders._bufferRef.buffer!.byteLength).toBeGreaterThan(0);

        world.free();
    });

    test("invalidated buffer falls back to the WASM path", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(1, 2, 3));
        const collider = world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        world.step();

        const expected = collider.translation(_v());

        // `ptr === 0` marks the contents as stale: reads must go through WASM
        // and must not resurrect the view.
        world.colliders._bufferRef.buffer = null;
        world.colliders._bufferRef.ptr = 0;

        const t = collider.translation(_v());
        expect(t.x).toBeCloseTo(expected.x, 4);
        expect(t.y).toBeCloseTo(expected.y, 4);
        expect(t.z).toBeCloseTo(expected.z, 4);
        expect(world.colliders._bufferRef.buffer).toBeNull();

        world.free();
    });

    test("buffer reads match the WASM path right after creating colliders", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const colliders: any[] = [];
        for (let i = 0; i < 50; i++) {
            const body = world.createRigidBody(
                RAPIER.RigidBodyDesc.dynamic().setTranslation(i, 5, 0),
            );
            colliders.push(world.createCollider(RAPIER.ColliderDesc.ball(0.5), body));
        }

        for (let i = 0; i < 5; i++) world.step();

        // Compare the buffer value (default path) with a forced WASM read.
        const scratch = {x: 0, y: 0, z: 0};
        for (const c of colliders) {
            const fromBuffer = c.translation(_v());
            // Force the WASM fallback.
            world.colliders._bufferRef.buffer = null;
            world.colliders._bufferRef.ptr = 0;
            c.translation(scratch);
            world.colliders.syncTransformBuffer(); // restore
            expect(fromBuffer.x).toBeCloseTo(scratch.x, 4);
            expect(fromBuffer.y).toBeCloseTo(scratch.y, 4);
            expect(fromBuffer.z).toBeCloseTo(scratch.z, 4);
        }

        world.free();
    });
});
