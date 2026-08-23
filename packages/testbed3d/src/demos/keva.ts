import type * as RAPIER_NS from "@alexandernanberg/rapier3d";
import type RAPIER from "@alexandernanberg/rapier3d";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof RAPIER_NS;

function buildBlock(
    RAPIER: RAPIER_API,
    world: RAPIER.World,
    halfExtents: RAPIER.Vector,
    shift: RAPIER.Vector,
    numx: number,
    numy: number,
    numz: number,
) {
    const half_extents_zyx = {
        x: halfExtents.z,
        y: halfExtents.y,
        z: halfExtents.x,
    };
    const dimensions = [halfExtents, half_extents_zyx];
    const blockWidth = 2.0 * halfExtents.z * numx;
    const blockHeight = 2.0 * halfExtents.y * numy;
    const spacing = (halfExtents.z * numx - halfExtents.x) / (numz - 1.0);

    let i;
    let j;
    let k;

    for (i = 0; i < numy; ++i) {
        [numx, numz] = [numz, numx];
        const dim = dimensions[i % 2];
        const y = dim.y * i * 2.0;

        for (j = 0; j < numx; ++j) {
            const x = i % 2 == 0 ? spacing * j * 2.0 : dim.x * j * 2.0;

            for (k = 0; k < numz; ++k) {
                const z = i % 2 == 0 ? dim.z * k * 2.0 : spacing * k * 2.0;
                // Build the rigid body.
                const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
                    x + dim.x + shift.x,
                    y + dim.y + shift.y,
                    z + dim.z + shift.z,
                );
                const body = world.createRigidBody(bodyDesc);
                const colliderDesc = RAPIER.ColliderDesc.cuboid(dim.x, dim.y, dim.z);
                world.createCollider(colliderDesc, body);
            }
        }
    }

    // Close the top.
    const dim = {x: halfExtents.z, y: halfExtents.x, z: halfExtents.y};

    for (i = 0; i < blockWidth / (dim.x * 2.0); ++i) {
        for (j = 0; j < blockWidth / (dim.z * 2.0); ++j) {
            // Build the rigid body.
            const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
                i * dim.x * 2.0 + dim.x + shift.x,
                dim.y + shift.y + blockHeight,
                j * dim.z * 2.0 + dim.z + shift.z,
            );
            const body = world.createRigidBody(bodyDesc);
            const colliderDesc = RAPIER.ColliderDesc.cuboid(dim.x, dim.y, dim.z);
            world.createCollider(colliderDesc, body);
        }
    }
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    const gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    const world = new RAPIER.World(gravity);

    // Create Ground.
    const groundSize = 50.0;
    const groundHeight = 0.1;
    const bodyDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0.0, -groundHeight, 0.0);
    const body = world.createRigidBody(bodyDesc);
    const colliderDesc = RAPIER.ColliderDesc.cuboid(groundSize, groundHeight, groundSize);
    world.createCollider(colliderDesc, body);

    // Keva tower.
    const halfExtents = new RAPIER.Vector3(0.1, 0.5, 2.0);
    let blockHeight = 0.0;
    // These should only be set to odd values otherwise
    // the blocks won't align in the nicest way.
    const numyArr = [0, 3, 5, 5, 7, 9];
    let i;

    for (i = 5; i >= 1; --i) {
        const numx = i;
        const numy = numyArr[i];
        const numz = numx * 3 + 1;
        const blockWidth = numx * halfExtents.z * 2.0;
        buildBlock(
            RAPIER,
            world,
            halfExtents,
            new RAPIER.Vector3(-blockWidth / 2.0, blockHeight, -blockWidth / 2.0),
            numx,
            numy,
            numz,
        );
        blockHeight += numy * halfExtents.y * 2.0 + halfExtents.x * 2.0;
    }

    testbed.setWorld(world);
    const cameraPosition = {
        eye: {
            x: -70.38553832116718,
            y: 17.893810295517365,
            z: 29.34767842147597,
        },
        target: {
            x: 0.5890869353464383,
            y: 3.132044603021203,
            z: -0.2899937806661885,
        },
    };
    testbed.lookAt(cameraPosition);
}
