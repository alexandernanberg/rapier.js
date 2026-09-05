import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Create Ground. It is perfectly elastic, so each ball only loses the energy
    // its own restitution gives up.
    let groundSize = 20.0;
    let groundHeight = 1.0;
    let groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0.0, -groundHeight, 0.0);
    let ground = world.createRigidBody(groundDesc);
    let groundColliderDesc = RAPIER.ColliderDesc.cuboid(
        groundSize,
        groundHeight,
        2.0,
    ).setRestitution(1.0);
    world.createCollider(groundColliderDesc, ground);

    // Two rows of balls, with the restitution going from 0 (no bounce at all) on
    // the left to 1 (bounces back to its initial height) on the right.
    let num = 10;
    let rad = 0.5;
    let i, j;

    for (j = 0; j < 2; ++j) {
        for (i = 0; i <= num; ++i) {
            let x = i - num / 2.0;

            let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
                x * 2.0,
                10.0 * (j + 1.0),
                0.0,
            );
            let body = world.createRigidBody(bodyDesc);
            let colliderDesc = RAPIER.ColliderDesc.ball(rad).setRestitution(i / num);
            world.createCollider(colliderDesc, body);
        }
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: 0.0, y: 3.0, z: 30.0},
        target: {x: 0.0, y: 3.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
