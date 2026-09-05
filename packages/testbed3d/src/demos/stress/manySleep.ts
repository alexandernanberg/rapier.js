import type {Testbed} from "../../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // The same lattice as the `balls` stress test, except every body starts
    // asleep: what is measured is how cheap a large sleeping island is to keep
    // around. Upstream uses 50 per side (125k balls).
    let num = 15;
    let rad = 1.0;

    let shift = rad * 2.0 + 1.0;
    let centerx = (shift * num) / 2.0;
    let centery = shift / 2.0;
    let centerz = (shift * num) / 2.0;
    let i, j, k;

    for (i = 0; i < num; ++i) {
        for (j = 0; j < num; ++j) {
            for (k = 0; k < num; ++k) {
                let x = i * shift - centerx;
                let y = j * shift + centery;
                let z = k * shift - centerz;

                let bodyDesc =
                    j == 0 ? RAPIER.RigidBodyDesc.fixed() : RAPIER.RigidBodyDesc.dynamic();
                let body = world.createRigidBody(
                    bodyDesc.setTranslation(x, y, z).setSleeping(true),
                );
                world.createCollider(RAPIER.ColliderDesc.ball(rad).setDensity(0.477), body);
            }
        }
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: 100.0, y: 100.0, z: 100.0},
        target: {x: 0.0, y: 0.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
