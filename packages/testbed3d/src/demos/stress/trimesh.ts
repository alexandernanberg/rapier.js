import type {Testbed} from "../../Testbed";
import {createFallingShapes, generateHeightfield, GROUND_SIZE, NSUBDIVS} from "./heightfield";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

/**
 * The same ground as the `heightfield` stress test, but converted to a triangle
 * mesh: same geometry, a different narrow-phase path.
 */
function heightfieldToTrimesh(heights: Float32Array) {
    let vertices = [];
    let indices = [];
    let i, j;

    for (j = 0; j <= NSUBDIVS; ++j) {
        for (i = 0; i <= NSUBDIVS; ++i) {
            let x = (j / NSUBDIVS - 0.5) * GROUND_SIZE.x;
            let y = heights[j * (NSUBDIVS + 1) + i] * GROUND_SIZE.y;
            let z = (i / NSUBDIVS - 0.5) * GROUND_SIZE.z;

            vertices.push(x, y, z);
        }
    }

    for (j = 0; j < NSUBDIVS; ++j) {
        for (i = 0; i < NSUBDIVS; ++i) {
            let i1 = (i + 0) * (NSUBDIVS + 1) + (j + 0);
            let i2 = (i + 0) * (NSUBDIVS + 1) + (j + 1);
            let i3 = (i + 1) * (NSUBDIVS + 1) + (j + 0);
            let i4 = (i + 1) * (NSUBDIVS + 1) + (j + 1);

            indices.push(i1, i3, i2);
            indices.push(i3, i4, i2);
        }
    }

    return {
        vertices: new Float32Array(vertices),
        indices: new Uint32Array(indices),
    };
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    let ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    let mesh = heightfieldToTrimesh(generateHeightfield());
    world.createCollider(RAPIER.ColliderDesc.trimesh(mesh.vertices, mesh.indices), ground);

    createFallingShapes(RAPIER, world);

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: 100.0, y: 100.0, z: 100.0},
        target: {x: 0.0, y: 0.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
