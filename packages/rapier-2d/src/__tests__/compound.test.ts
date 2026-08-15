import RAPIER, {init} from "@alexandernanberg/rapier2d/compat";
import {describe, test, expect, beforeAll} from "vitest";

beforeAll(async () => {
    await init();
});

describe("Compound shapes", () => {
    test("round-trips a compound collider through the shape accessor", () => {
        const world = new RAPIER.World({x: 0, y: -9.81});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const collider = world.createCollider(
            RAPIER.ColliderDesc.compound(
                [new RAPIER.Ball(0.5), new RAPIER.Cuboid(1, 2)],
                [
                    {x: 0, y: 0},
                    {x: 4, y: 0},
                ],
                [0, Math.PI / 2],
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
        expect((compound.shapes[1] as RAPIER.Cuboid).halfExtents.y).toBe(2);
        expect(compound.positions[1].x).toBe(4);
        expect(compound.rotations[1]).toBeCloseTo(Math.PI / 2, 5);

        world.free();
    });

    test("rejects invalid compound shapes", () => {
        expect(() => new RAPIER.Compound([], [], [])).toThrow();
        expect(() => new RAPIER.Compound([new RAPIER.Ball(1)], [], [])).toThrow();
        expect(
            () =>
                new RAPIER.Compound(
                    [new RAPIER.Compound([new RAPIER.Ball(1)], [{x: 0, y: 0}], [0])],
                    [{x: 0, y: 0}],
                    [0],
                ),
        ).toThrow();
    });

    test("decomposes a polyline into convex parts", () => {
        // An L-shaped polyline, which needs more than one convex part.
        const vertices = new Float32Array([0, 0, 2, 0, 2, 1, 1, 1, 1, 2, 0, 2]);
        const indices = new Uint32Array([0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 0]);

        const desc = RAPIER.ColliderDesc.convexDecomposition(vertices, indices);
        expect(desc).not.toBeNull();
        expect(desc!.shape.type).toBe(RAPIER.ShapeType.Compound);
        expect((desc!.shape as RAPIER.Compound).shapes.length).toBeGreaterThan(0);

        const world = new RAPIER.World({x: 0, y: -9.81});
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const collider = world.createCollider(desc!, body);
        expect(RAPIER.Shape.fromRaw(world.colliders.raw, collider.handle).type).toBe(
            RAPIER.ShapeType.Compound,
        );
        world.free();
    });
});
