import type * as RAPIER_NS from "@alexandernanberg/rapier3d";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof RAPIER_NS;

/**
 * A cross/plus-shaped extruded mesh. It is strongly non-convex, so a convex
 * decomposition gives a noticeably better fit than a single convex hull.
 */
function crossMesh(halfArm: number, halfThickness: number, halfDepth: number) {
    // The 12 corners of the cross, in the xy plane, in counter-clockwise order.
    const outline = [
        [-halfThickness, -halfArm],
        [halfThickness, -halfArm],
        [halfThickness, -halfThickness],
        [halfArm, -halfThickness],
        [halfArm, halfThickness],
        [halfThickness, halfThickness],
        [halfThickness, halfArm],
        [-halfThickness, halfArm],
        [-halfThickness, halfThickness],
        [-halfArm, halfThickness],
        [-halfArm, -halfThickness],
        [-halfThickness, -halfThickness],
    ];

    const n = outline.length;
    const vertices: number[] = [];

    // Back face, then front face.
    for (const z of [-halfDepth, halfDepth]) {
        for (const [x, y] of outline) {
            vertices.push(x, y, z);
        }
    }

    const indices: number[] = [];

    // The side quads.
    for (let i = 0; i < n; ++i) {
        const next = (i + 1) % n;
        indices.push(i, next, n + i);
        indices.push(next, n + next, n + i);
    }

    // The two caps, triangulated as fans around the first corner. The outline is
    // non-convex, but the caps are only there to close the mesh: the decomposition
    // works from the voxelized volume.
    for (let i = 1; i < n - 1; ++i) {
        indices.push(0, i + 1, i);
        indices.push(n, n + i, n + i + 1);
    }

    return {
        vertices: new Float32Array(vertices),
        indices: new Uint32Array(indices),
    };
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    const gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    const world = new RAPIER.World(gravity);

    // Create the ground.
    const groundDesc = RAPIER.RigidBodyDesc.fixed();
    const ground = world.createRigidBody(groundDesc);
    world.createCollider(RAPIER.ColliderDesc.cuboid(30.0, 0.1, 30.0), ground);

    const mesh = crossMesh(1.5, 0.4, 0.4);
    const colliderDesc = RAPIER.ColliderDesc.convexDecomposition(mesh.vertices, mesh.indices, {
        maxConvexHulls: 8,
        resolution: 32,
    });

    if (!colliderDesc) {
        throw new Error("the convex decomposition produced no convex part");
    }

    const numParts = (colliderDesc.shape as InstanceType<RAPIER_API["Compound"]>).shapes.length;
    console.log(`Convex decomposition produced ${numParts} convex parts.`);

    for (let j = 0; j < 10; ++j) {
        for (let i = 0; i < 3; ++i) {
            for (let k = 0; k < 3; ++k) {
                const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
                    i * 4.0 - 4.0,
                    j * 2.0 + 2.0,
                    k * 4.0 - 4.0,
                );
                const body = world.createRigidBody(bodyDesc);
                // The same descriptor can be reused: it is converted to a raw shape on
                // each `createCollider` call.
                world.createCollider(colliderDesc, body);
            }
        }
    }

    testbed.setWorld(world);

    testbed.lookAt({
        eye: {x: -25.0, y: 20.0, z: 25.0},
        target: {x: 0.0, y: 5.0, z: 0.0},
    });
}
