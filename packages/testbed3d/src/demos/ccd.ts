import type * as RAPIER_NS from "@alexandernanberg/rapier3d";
import type RAPIER from "@alexandernanberg/rapier3d";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof RAPIER_NS;

function createWall(
    RAPIER: RAPIER_API,
    testbed: Testbed,
    world: RAPIER.World,
    offset: {x: number; y: number; z: number},
    stackHeight: number,
) {
    let i, j;

    const shiftY = 1.0;
    const shiftZ = 2.0;

    for (i = 0; i < stackHeight; ++i) {
        for (j = i; j < stackHeight; ++j) {
            const x = offset.x;
            const y = i * shiftY + offset.y;
            const z = (i * shiftZ) / 2.0 + (j - i) * shiftZ + offset.z - stackHeight;

            // Create dynamic cube.
            const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
            const body = world.createRigidBody(bodyDesc);
            const colliderDesc = RAPIER.ColliderDesc.cuboid(0.5, 0.5, 1.0);
            world.createCollider(colliderDesc, body);
        }
    }
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    const gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    const world = new RAPIER.World(gravity);

    // Create Ground.
    const groundHeight = 0.1;
    let bodyDesc = RAPIER.RigidBodyDesc.fixed();
    let body = world.createRigidBody(bodyDesc);
    let colliderDesc = RAPIER.ColliderDesc.cuboid(30.0, 0.1, 30.0);
    world.createCollider(colliderDesc, body);

    const numX = 5;
    const numZ = 8;
    const shiftY = groundHeight + 0.5;

    let i;
    for (i = 0; i < numX; ++i) {
        const x = i * 6.0;
        createWall(RAPIER, testbed, world, {x, y: shiftY, z: 0.0}, numZ);
    }

    // A very fast rigid-body with CCD enabled.
    // Create dynamic cube.
    bodyDesc = RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(-20.0, shiftY + 2.0, 0.0)
        .setLinvel(1000.0, 0.0, 0.0)
        .setCcdEnabled(true);
    body = world.createRigidBody(bodyDesc);
    colliderDesc = RAPIER.ColliderDesc.ball(1.0).setDensity(10.0);
    world.createCollider(colliderDesc, body);

    testbed.setWorld(world);
    const cameraPosition = {
        eye: {x: -31.96000000000001, y: 19.730000000000008, z: -27.86},
        target: {x: -0.0505, y: -0.4126, z: -0.0229},
    };
    testbed.lookAt(cameraPosition);
}
