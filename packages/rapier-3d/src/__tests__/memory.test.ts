import RAPIER, {init} from "@alexandernanberg/rapier3d/compat";
import {describe, test, expect, beforeAll} from "vitest";

const GRAVITY = {x: 0, y: -9.81, z: 0};
const PAGE = 65536;

let memory: WebAssembly.Memory;

beforeAll(async () => {
    await init();
    // The module's linear memory, reached through the transform buffer's view of
    // it. The `WebAssembly.Memory` object is stable for the instance; only its
    // `buffer` is swapped out when the memory grows.
    const world = new RAPIER.World(GRAVITY);
    world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    world.step();
    memory = (world.bodies as unknown as {_bufferRef: {memory: WebAssembly.Memory}})._bufferRef
        .memory;
    world.free();
});

const pages = () => memory.buffer.byteLength / PAGE;

/**
 * Runs `body` a few times to reach a steady state, then asserts that running it
 * many more times does not grow the WASM heap.
 *
 * WASM memory only ever grows, so a missing `free()` on the raw objects that
 * cross the boundary shows up here as unbounded growth. Each workload is sized
 * so that leaking a single raw object per inner call would add hundreds of
 * kilobytes over the measured phase — comfortably past `maxPageGrowth`, while
 * a correct workload measures at exactly zero.
 */
function expectSteadyMemory(
    run: (i: number) => void,
    {warmup, measure, maxPageGrowth = 2}: {warmup: number; measure: number; maxPageGrowth?: number},
) {
    for (let i = 0; i < warmup; i++) run(i);

    const before = pages();
    for (let i = 0; i < measure; i++) run(warmup + i);
    const grown = pages() - before;

    expect(
        grown,
        `WASM heap grew by ${grown} pages (${grown * PAGE} bytes) over ${measure} iterations`,
    ).toBeLessThanOrEqual(maxPageGrowth);
}

function populate(world: RAPIER.World, count: number) {
    const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    world.createCollider(RAPIER.ColliderDesc.cuboid(50, 0.1, 50), ground);
    for (let i = 0; i < count; i++) {
        const body = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation((i % 10) * 2 - 10, 1 + i * 0.1, 0),
        );
        world.createCollider(RAPIER.ColliderDesc.ball(0.4), body);
    }
    world.step();
}

describe("memory", () => {
    test("spawning and despawning bodies reaches a steady heap", () => {
        const world = new RAPIER.World(GRAVITY);
        const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(RAPIER.ColliderDesc.cuboid(50, 0.1, 50), ground);

        expectSteadyMemory(
            () => {
                const spawned: RAPIER.RigidBody[] = [];
                for (let i = 0; i < 50; i++) {
                    const body = world.createRigidBody(
                        RAPIER.RigidBodyDesc.dynamic().setTranslation(i * 0.5 - 12, 3, 0),
                    );
                    world.createCollider(RAPIER.ColliderDesc.ball(0.4), body);
                    spawned.push(body);
                }
                world.step();
                for (const body of spawned) world.removeRigidBody(body);
                world.step();
            },
            {warmup: 20, measure: 120},
        );

        world.free();
    });

    test("scene queries do not leak the raw objects they hand across", () => {
        const world = new RAPIER.World(GRAVITY);
        populate(world, 50);

        const ray = new RAPIER.Ray({x: 0, y: 30, z: 0}, {x: 0, y: -1, z: 0});
        const shape = new RAPIER.Ball(0.5);

        expectSteadyMemory(
            () => {
                for (let i = 0; i < 100; i++) {
                    world.castRay(ray, 100, true);
                    world.castRayAndGetNormal(ray, 100, true);
                    world.projectPoint({x: 0, y: 2, z: 0}, true);
                    world.intersectionsWithRay(ray, 100, true, () => true);
                    world.intersectionsWithPoint({x: 0, y: 2, z: 0}, () => true);
                    world.castShape(
                        {x: 0, y: 30, z: 0},
                        {x: 0, y: 0, z: 0, w: 1},
                        {x: 0, y: -1, z: 0},
                        shape,
                        0,
                        100,
                        true,
                    );
                }
            },
            // Sized from measurement: a single missing `free()` in any of these
            // paths grows the heap by ~20 pages over the measured phase, while a
            // correct one measures at zero. Smaller runs are swallowed by the
            // slack the allocator already holds.
            {warmup: 40, measure: 400},
        );

        world.free();
    });

    test("mass-property getters do not leak the raw objects they consume", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        world.step();

        expectSteadyMemory(
            () => {
                for (let i = 0; i < 500; i++) {
                    body.principalInertia();
                    body.invPrincipalInertia();
                    body.principalInertiaLocalFrame();
                    body.effectiveWorldInvInertia();
                    body.effectiveAngularInertia();
                    body.localCom();
                    body.worldCom();
                }
            },
            {warmup: 10, measure: 100},
        );

        world.free();
    });

    test("creating and freeing worlds reaches a steady heap", () => {
        expectSteadyMemory(
            () => {
                const world = new RAPIER.World(GRAVITY);
                for (let i = 0; i < 80; i++) {
                    const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
                    world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
                }
                world.step();
                world.free();
            },
            {warmup: 20, measure: 150},
        );
    });

    test("character and PID controllers are released by free()", () => {
        const world = new RAPIER.World(GRAVITY);
        populate(world, 10);

        // Check the controller actually works once, outside the measured loop —
        // driving one is expensive enough that including it would cap the
        // iteration count well below what it takes to move the heap.
        const probe = world.createCharacterController(0.01);
        probe.computeColliderMovement(world.colliders.getAll()[0], {x: 0, y: -0.1, z: 0});
        expect(probe.computedMovement().y).toBeLessThan(0);
        world.removeCharacterController(probe);

        expectSteadyMemory(
            () => {
                world.removeCharacterController(world.createCharacterController(0.01));
                world.removePidController(
                    world.createPidController(1, 1, 1, RAPIER.PidAxesMask.All),
                );
            },
            // Controllers are small, so it takes tens of thousands of them to
            // outgrow the free memory the earlier tests in this file left behind
            // — a leak grows without bound, slack does not. Churning them is
            // cheap enough that that is affordable.
            {warmup: 2000, measure: 40_000},
        );

        world.free();
    });

    test("the transform buffer stays bounded while arena indices are recycled", () => {
        const world = new RAPIER.World(GRAVITY);
        const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(RAPIER.ColliderDesc.cuboid(50, 0.1, 50), ground);

        const bufferLen = () =>
            (world.bodies as unknown as {_bufferRef: {len: number}})._bufferRef.len;

        const churn = () => {
            const spawned: RAPIER.RigidBody[] = [];
            for (let i = 0; i < 40; i++) {
                const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
                world.createCollider(RAPIER.ColliderDesc.ball(0.4), body);
                spawned.push(body);
            }
            world.step();
            for (const body of spawned) world.removeRigidBody(body);
            world.step();
        };

        for (let i = 0; i < 20; i++) churn();
        const settled = bufferLen();
        for (let i = 0; i < 200; i++) churn();

        // Freed slots are handed back out, so ten times the churn must not make
        // the buffer any bigger than the high-water mark of live bodies.
        expect(bufferLen()).toBe(settled);

        world.free();
    });

    test("buffered transforms survive WASM memory growth", () => {
        const world = new RAPIER.World(GRAVITY);
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(3, 7, -2));
        const collider = world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        world.step();

        const beforeBody = body.translation();
        const beforeCollider = collider.translation();
        const beforePages = pages();

        // Growing the linear memory detaches every Float32Array over it, which
        // is what the buffer views are. Reads have to notice and re-attach.
        RAPIER.reserveMemory(64 * 1024 * 1024);
        expect(pages()).toBeGreaterThan(beforePages);

        expect(body.translation()).toEqual(beforeBody);
        expect(collider.translation()).toEqual(beforeCollider);

        // And the views have to be rebuilt rather than reused on the next step.
        world.step();
        expect(body.translation().y).toBeLessThan(beforeBody.y);
        expect(collider.translation()).toEqual(body.translation());

        world.free();
    });
});
