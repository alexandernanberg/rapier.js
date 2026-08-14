import RAPIER, {init} from "@alexandernanberg/rapier3d/compat";
import {describe, test, expect, beforeAll} from "vitest";

beforeAll(async () => {
    await init();
});

describe("Collider", () => {
    test("creates ball collider", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const collider = world.createCollider(RAPIER.ColliderDesc.ball(1.5), body);

        const shape = collider.shape;
        expect(shape.type).toBe(RAPIER.ShapeType.Ball);
        expect((shape as RAPIER.Ball).radius).toBe(1.5);

        world.free();
    });

    test("creates cuboid collider", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const collider = world.createCollider(RAPIER.ColliderDesc.cuboid(1, 2, 3), body);

        const shape = collider.shape;
        expect(shape.type).toBe(RAPIER.ShapeType.Cuboid);
        const cuboid = shape as RAPIER.Cuboid;
        expect(cuboid.halfExtents.x).toBe(1);
        expect(cuboid.halfExtents.y).toBe(2);
        expect(cuboid.halfExtents.z).toBe(3);

        world.free();
    });

    test("creates capsule collider", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const collider = world.createCollider(RAPIER.ColliderDesc.capsule(2, 0.5), body);

        const shape = collider.shape;
        expect(shape.type).toBe(RAPIER.ShapeType.Capsule);
        const capsule = shape as RAPIER.Capsule;
        expect(capsule.halfHeight).toBe(2);
        expect(capsule.radius).toBe(0.5);

        world.free();
    });

    test("collider attached to body moves with it", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 10, 0));
        const collider = world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

        const initialColliderY = collider.translation().y;
        expect(initialColliderY).toBeCloseTo(10, 1);

        world.step();

        // Collider should have moved with the body
        const newColliderY = collider.translation().y;
        expect(newColliderY).toBeLessThan(initialColliderY);

        // Collider and body positions should match
        expect(newColliderY).toBeCloseTo(body.translation().y, 4);

        world.free();
    });

    test("collider translation and rotation", () => {
        const world = new RAPIER.World({x: 0, y: 0, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(5, 10, 15));
        const collider = world.createCollider(RAPIER.ColliderDesc.ball(1), body);

        const pos = collider.translation();
        expect(pos.x).toBeCloseTo(5);
        expect(pos.y).toBeCloseTo(10);
        expect(pos.z).toBeCloseTo(15);

        const rot = collider.rotation();
        expect(rot.x).toBeCloseTo(0);
        expect(rot.y).toBeCloseTo(0);
        expect(rot.z).toBeCloseTo(0);
        expect(rot.w).toBeCloseTo(1);

        world.free();
    });

    test("collider with density affects body mass", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        world.createCollider(RAPIER.ColliderDesc.ball(1).setDensity(5.0), body);

        expect(body.mass()).toBeGreaterThan(0);

        world.free();
    });

    test("collider with sensor does not generate contacts", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});

        // Floor
        const floor = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        world.createCollider(RAPIER.ColliderDesc.cuboid(10, 0.1, 10), floor);

        // Falling sensor ball
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 1, 0));
        world.createCollider(RAPIER.ColliderDesc.ball(0.5).setSensor(true), body);

        // Step multiple times - sensor should fall through the floor
        for (let i = 0; i < 120; i++) {
            world.step();
        }

        // Sensor body should have fallen below the floor (y < -0.1)
        expect(body.translation().y).toBeLessThan(-0.1);

        world.free();
    });
});

describe("Compound shapes", () => {
    test("round-trips a compound collider through the shape accessor", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const collider = world.createCollider(
            RAPIER.ColliderDesc.compound(
                [new RAPIER.Ball(0.5), new RAPIER.Cuboid(1, 2, 3)],
                [
                    {x: 0, y: 0, z: 0},
                    {x: 4, y: 0, z: 0},
                ],
                [
                    {x: 0, y: 0, z: 0, w: 1},
                    {x: 0, y: 0, z: 0, w: 1},
                ],
            ),
            body,
        );

        // Read the shape back from WASM instead of the cached descriptor shape.
        const shape = RAPIER.Shape.fromRaw(world.colliders.raw, collider.handle);
        expect(shape.type).toBe(RAPIER.ShapeType.Compound);

        const compound = shape as RAPIER.Compound;
        expect(compound.shapes.length).toBe(2);
        expect(compound.shapes[0].type).toBe(RAPIER.ShapeType.Ball);
        expect((compound.shapes[0] as RAPIER.Ball).radius).toBe(0.5);
        expect(compound.shapes[1].type).toBe(RAPIER.ShapeType.Cuboid);
        expect((compound.shapes[1] as RAPIER.Cuboid).halfExtents.y).toBe(2);
        expect(compound.positions[1].x).toBe(4);
        expect(compound.rotations[1].w).toBe(1);

        world.free();
    });

    test("rejects invalid compound shapes", () => {
        expect(() => new RAPIER.Compound([], [], [])).toThrow();
        expect(() => new RAPIER.Compound([new RAPIER.Ball(1)], [], [])).toThrow();
        expect(
            () =>
                new RAPIER.Compound(
                    [
                        new RAPIER.Compound(
                            [new RAPIER.Ball(1)],
                            [{x: 0, y: 0, z: 0}],
                            [{x: 0, y: 0, z: 0, w: 1}],
                        ),
                    ],
                    [{x: 0, y: 0, z: 0}],
                    [{x: 0, y: 0, z: 0, w: 1}],
                ),
        ).toThrow();
    });

    test("decomposes a mesh into convex parts", () => {
        // A unit cube, which decomposes into a single convex part.
        const vertices = new Float32Array([
            0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1,
        ]);
        const indices = new Uint32Array([
            0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7,
            6, 3, 0, 4, 3, 4, 7,
        ]);

        const desc = RAPIER.ColliderDesc.convexDecomposition(vertices, indices);
        expect(desc).not.toBeNull();
        expect(desc!.shape.type).toBe(RAPIER.ShapeType.Compound);
        expect((desc!.shape as RAPIER.Compound).shapes.length).toBeGreaterThan(0);

        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const collider = world.createCollider(desc!, body);
        expect(RAPIER.Shape.fromRaw(world.colliders.raw, collider.handle).type).toBe(
            RAPIER.ShapeType.Compound,
        );
        world.free();
    });

    test("returns null when the decomposition yields no convex part", () => {
        // A single degenerate (zero-area) triangle.
        const vertices = new Float32Array([0, 0, 0, 1, 0, 0, 2, 0, 0]);
        const indices = new Uint32Array([0, 1, 2]);
        expect(RAPIER.ColliderDesc.convexDecomposition(vertices, indices)).toBeNull();
    });

    test("round-trips a convex polyhedron collider", () => {
        const world = new RAPIER.World({x: 0, y: -9.81, z: 0});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const points = new Float32Array([
            0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1,
        ]);
        const collider = world.createCollider(RAPIER.ColliderDesc.convexHull(points)!, body);

        const shape = RAPIER.Shape.fromRaw(
            world.colliders.raw,
            collider.handle,
        ) as RAPIER.ConvexPolyhedron;
        expect(shape.type).toBe(RAPIER.ShapeType.ConvexPolyhedron);

        // The exported vertices/indices must be consistent with each other, so that the
        // shape can be rebuilt from them.
        const maxIndex = shape.indices!.reduce((a, b) => Math.max(a, b), 0);
        expect(maxIndex).toBeLessThan(shape.vertices.length / 3);
        expect(RAPIER.ColliderDesc.convexMesh(shape.vertices, shape.indices)).not.toBeNull();

        world.free();
    });
});
