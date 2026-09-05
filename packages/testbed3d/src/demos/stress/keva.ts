import type RAPIER from "@alexandernanberg/rapier3d";
import type {Testbed} from "../../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

/**
 * One tier of the tower: alternating layers of planks laid at right angles to
 * each other, then a solid cap closing the top.
 */
function buildBlock(
    RAPIER: RAPIER_API,
    world: RAPIER.World,
    halfExtents: RAPIER.Vector3,
    shift: RAPIER.Vector3,
    numx: number,
    numy: number,
    numz: number,
) {
    // The two plank orientations, swapped every layer.
    let dimensions = [
        new RAPIER.Vector3(halfExtents.x, halfExtents.y, halfExtents.z),
        new RAPIER.Vector3(halfExtents.z, halfExtents.y, halfExtents.x),
    ];
    let blockWidth = 2.0 * halfExtents.z * numx;
    let blockHeight = 2.0 * halfExtents.y * numy;
    let spacing = (halfExtents.z * numx - halfExtents.x) / (numz - 1.0);
    let i, j, k;

    for (i = 0; i < numy; ++i) {
        let swap = numx;
        numx = numz;
        numz = swap;

        let dim = dimensions[i % 2];
        let y = dim.y * i * 2.0;

        for (j = 0; j < numx; ++j) {
            let x = i % 2 == 0 ? spacing * j * 2.0 : dim.x * j * 2.0;

            for (k = 0; k < numz; ++k) {
                let z = i % 2 == 0 ? dim.z * k * 2.0 : spacing * k * 2.0;

                let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
                    x + dim.x + shift.x,
                    y + dim.y + shift.y,
                    z + dim.z + shift.z,
                );
                let body = world.createRigidBody(bodyDesc);
                world.createCollider(RAPIER.ColliderDesc.cuboid(dim.x, dim.y, dim.z), body);
            }
        }
    }

    // Close the top.
    let dim = new RAPIER.Vector3(halfExtents.z, halfExtents.x, halfExtents.y);

    for (i = 0; i < Math.floor(blockWidth / (dim.x * 2.0)); ++i) {
        for (j = 0; j < Math.floor(blockWidth / (dim.z * 2.0)); ++j) {
            let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
                i * dim.x * 2.0 + dim.x + shift.x,
                dim.y + shift.y + blockHeight,
                j * dim.z * 2.0 + dim.z + shift.z,
            );
            let body = world.createRigidBody(bodyDesc);
            world.createCollider(RAPIER.ColliderDesc.cuboid(dim.x, dim.y, dim.z), body);
        }
    }
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Create Ground.
    let groundSize = 50.0;
    let groundHeight = 0.1;
    let groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0.0, -groundHeight, 0.0);
    let ground = world.createRigidBody(groundDesc);
    world.createCollider(RAPIER.ColliderDesc.cuboid(groundSize, groundHeight, groundSize), ground);

    // A tapering stack of plank tiers. The layer counts should stay odd,
    // otherwise the planks don't align in the nicest way. Upstream uses
    // [_, 13, 17, 21, 41, 83] layers (~40k planks); these keep it around 4k.
    let halfExtents = new RAPIER.Vector3(0.1, 0.5, 2.0);
    let numy = [0, 5, 7, 9, 13, 21];
    let blockHeight = 0.0;
    let i;

    for (i = 5; i >= 1; --i) {
        let numx = i * 2;
        let numz = numx * 3 + 1;
        let blockWidth = numx * halfExtents.z * 2.0;

        buildBlock(
            RAPIER,
            world,
            halfExtents,
            new RAPIER.Vector3(-blockWidth / 2.0, blockHeight, -blockWidth / 2.0),
            numx,
            numy[i],
            numz,
        );

        blockHeight += numy[i] * halfExtents.y * 2.0 + halfExtents.x * 2.0;
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: 60.0, y: 60.0, z: 60.0},
        target: {x: 0.0, y: 10.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
