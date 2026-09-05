import type {Testbed} from "../../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

/**
 * Port of box3d's `large_pyramid` benchmark (`CreateLargePyramid`,
 * `box3d/shared/benchmarks.c`). Release settings: a wide pyramid of unit cubes
 * (density 100) on a large ground box, sleeping disabled.
 */
export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    // box3d's `b3DefaultWorldDef`: gravity (0, -10, 0). box3d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let gravity = new RAPIER.Vector3(0.0, -10.0, 0.0);
    let world = new RAPIER.World(gravity);

    // Upstream's base is 200 boxes wide (20,100 boxes); 80 keeps it around 3.2k.
    let baseCount = 80;

    // Ground: b3MakeBoxHull(400, 1, 400) at y = -1.
    let groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0.0, -1.0, 0.0);
    let ground = world.createRigidBody(groundDesc);
    world.createCollider(RAPIER.ColliderDesc.cuboid(400.0, 1.0, 400.0), ground);

    let h = 0.5;
    let shift = 1.0 * h;
    let i, j;

    for (i = 0; i < baseCount; ++i) {
        let y = (2.0 * i + 1.0) * shift;

        for (j = i; j < baseCount; ++j) {
            let x = (i + 1.0) * shift + 2.0 * (j - i) * shift - h * baseCount;

            let bodyDesc = RAPIER.RigidBodyDesc.dynamic()
                .setTranslation(x, y, 0.0)
                .setCanSleep(false);
            let body = world.createRigidBody(bodyDesc);
            world.createCollider(RAPIER.ColliderDesc.cuboid(h, h, h).setDensity(100.0), body);
        }
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: 0.0, y: 25.0, z: 70.0},
        target: {x: 0.0, y: 12.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
