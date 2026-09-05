import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Create Ground.
    let groundSize = 100.1;
    let groundHeight = 2.1;
    let groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0.0, -groundHeight, 0.0);
    let ground = world.createRigidBody(groundDesc);
    let groundColliderDesc = RAPIER.ColliderDesc.cuboid(groundSize, groundHeight, groundSize);
    world.createCollider(groundColliderDesc, ground);

    // Create the primitives.
    let num = 8;
    let rad = 1.0;

    let shift = rad * 2.0 + rad;
    let centerx = shift * (num / 2);
    let centery = shift / 2.0;
    let centerz = shift * (num / 2);

    let offset = -num * (rad * 2.0 + rad) * 0.5;
    let i, j, k;

    for (j = 0; j < 20; ++j) {
        for (i = 0; i < num; ++i) {
            for (k = 0; k < num; ++k) {
                let x = i * shift - centerx + offset;
                let y = j * shift + centery + 3.0;
                let z = k * shift - centerz + offset;

                let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
                let body = world.createRigidBody(bodyDesc);
                let colliderDesc;

                switch (j % 5) {
                    case 1:
                        colliderDesc = RAPIER.ColliderDesc.ball(rad);
                        break;
                    // Rounded cylinders are much more efficient than cylinders, even
                    // if the rounding margin is small.
                    case 2:
                        colliderDesc = RAPIER.ColliderDesc.roundCylinder(rad, rad, rad / 10.0);
                        break;
                    case 3:
                        colliderDesc = RAPIER.ColliderDesc.cone(rad, rad);
                        break;
                    default:
                        colliderDesc = RAPIER.ColliderDesc.capsule(rad, rad);
                        break;
                }

                world.createCollider(colliderDesc, body);
            }
        }

        offset -= 0.05 * rad * (num - 1.0);
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: 100.0, y: 100.0, z: 100.0},
        target: {x: 0.0, y: 0.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
