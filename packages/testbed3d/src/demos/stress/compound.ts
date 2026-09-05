import type {Testbed} from "../../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Create Ground.
    let groundSize = 200.1;
    let groundHeight = 0.1;
    let groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0.0, -groundHeight, 0.0);
    let ground = world.createRigidBody(groundDesc);
    world.createCollider(RAPIER.ColliderDesc.cuboid(groundSize, groundHeight, groundSize), ground);

    // U-shaped bodies, each made of three box colliders on one rigid-body: three
    // times the colliders of the `boxes` test for the same body count.
    let num = 8;
    let rad = 0.2;

    let shift = rad * 4.0 + rad;
    let centerx = shift * (num / 2);
    let centery = shift / 2.0;
    let centerz = shift * (num / 2);

    let offset = -num * (rad * 2.0 + rad) * 0.5;
    let i, j, k;

    for (j = 0; j < 25; ++j) {
        for (i = 0; i < num; ++i) {
            for (k = 0; k < num; ++k) {
                let x = i * shift * 5.0 - centerx + offset;
                let y = j * (shift * 5.0) + centery + 3.0;
                let z = k * shift * 2.0 - centerz + offset;

                let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
                let body = world.createRigidBody(bodyDesc);

                world.createCollider(RAPIER.ColliderDesc.cuboid(rad * 10.0, rad, rad), body);
                world.createCollider(
                    RAPIER.ColliderDesc.cuboid(rad, rad * 10.0, rad).setTranslation(
                        rad * 10.0,
                        rad * 10.0,
                        0.0,
                    ),
                    body,
                );
                world.createCollider(
                    RAPIER.ColliderDesc.cuboid(rad, rad * 10.0, rad).setTranslation(
                        -rad * 10.0,
                        rad * 10.0,
                        0.0,
                    ),
                    body,
                );
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
