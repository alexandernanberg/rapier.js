import type {Testbed} from "../../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

/**
 * A wide brick-laid box pyramid: slightly shrunken cubes on a 2.25 pitch with a
 * 1.0 brick offset — which is *not* half of 2.25 — so the boxes rest on four
 * unequal corner patches. The asymmetry is deliberate: a perfectly symmetric
 * brick is a degenerate, marginally-stable configuration.
 */
export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Ground: a 100x1x100 half-extents box at y = -1, so its top face is y = 0.
    let groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0.0, -1.0, 0.0);
    let ground = world.createRigidBody(groundDesc);
    world.createCollider(RAPIER.ColliderDesc.cuboid(100.0, 1.0, 100.0), ground);

    // Upstream stacks 50 layers (~43k boxes); 22 keeps it around 3.8k.
    let pyramidHeight = 22;
    let boxSize = 2.0;
    let boxSeparation = 0.5;
    let halfBoxSize = 0.5 * boxSize;
    // Shrunken cube: the boxes never quite fill their lattice cell.
    let h = halfBoxSize - 0.025;
    let i, j, k;

    for (i = 0; i < pyramidHeight; ++i) {
        // Odd layers are brick-offset by a half box, which is NOT half of the
        // lateral pitch — that is what makes the four corner supports unequal.
        let brick = i % 2 != 0 ? halfBoxSize : 0.0;
        let y = 1.0 + (boxSize + boxSeparation) * i;
        let from = Math.floor(i / 2);
        let to = pyramidHeight - Math.floor((i + 1) / 2);

        for (j = from; j < to; ++j) {
            for (k = from; k < to; ++k) {
                let x = -pyramidHeight + (boxSize + 0.25) * j + brick;
                let z = -pyramidHeight + (boxSize + 0.25) * k + brick;

                let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
                let body = world.createRigidBody(bodyDesc);
                // Water density. For a uniform-density pile this changes nothing
                // dynamically: the soft-contact coefficients are mass-normalized.
                world.createCollider(RAPIER.ColliderDesc.cuboid(h, h, h).setDensity(1000.0), body);
            }
        }
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: 100.0, y: 70.0, z: 100.0},
        target: {x: 5.0, y: 25.0, z: 5.0},
    };
    testbed.lookAt(cameraPosition);
}
