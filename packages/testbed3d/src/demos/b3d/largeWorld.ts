import type {Testbed} from "../../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

const CELL_SIZE = 10.0;
// Upstream's floor is 1000x1000 (a million static shapes); 60x60 keeps the
// build time and memory inside what a browser tab will put up with.
const GRID = 60;
const SPHERES = 100;
const DROP_INTERVAL = 5;

/**
 * Port of box3d's `large_world` benchmark (`CreateLargeWorld` +
 * `StepLargeWorld`, `box3d/shared/benchmarks.c`): a large static box floor onto
 * which dynamic spheres are dropped, one every few steps.
 *
 * box3d creates one static *body* per floor box; rapier's idiomatic (and
 * perf-equivalent) static geometry is a parentless collider, so the floor is
 * built from standalone fixed colliders. The point of the scene is the
 * broad-phase against a huge static set.
 */
export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    // box3d's `b3DefaultWorldDef`: gravity (0, -10, 0). box3d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let gravity = new RAPIER.Vector3(0.0, -10.0, 0.0);
    let world = new RAPIER.World(gravity);

    let halfSpan = 0.5 * CELL_SIZE * GRID;
    let i, j;

    for (i = 0; i < GRID; ++i) {
        let x = -halfSpan + (i + 0.5) * CELL_SIZE;

        for (j = 0; j < GRID; ++j) {
            let z = -halfSpan + (j + 0.5) * CELL_SIZE;
            world.createCollider(
                RAPIER.ColliderDesc.cuboid(0.5 * CELL_SIZE, 0.25, 0.5 * CELL_SIZE).setTranslation(
                    x,
                    0.0,
                    z,
                ),
            );
        }
    }

    testbed.setWorld(world);

    // `StepLargeWorld`: drop one sphere every `DROP_INTERVAL` steps, spread on a
    // coarse grid over the inner 80% of the floor, up to `SPHERES` total.
    let side = 1;
    while (side * side < SPHERES) {
        side += 1;
    }

    let stepCount = 0;
    let dropped = 0;

    testbed.setpreTimestepAction((graphics) => {
        if (dropped < SPHERES && stepCount > 0 && stepCount % DROP_INTERVAL == 0) {
            let gi = dropped % side;
            let gj = Math.floor(dropped / side);
            let inset = 0.1 * 2.0 * halfSpan;
            let usable = 2.0 * halfSpan - 2.0 * inset;
            let x = -halfSpan + inset + (gi + 0.5) * (usable / side);
            let z = -halfSpan + inset + (gj + 0.5) * (usable / side);

            let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, 1.5, z);
            let body = testbed.world.createRigidBody(bodyDesc);
            let collider = testbed.world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
            // Spawned after `setWorld`, so the renderer has to be told about it.
            graphics.addCollider(RAPIER, testbed.world, collider);
            dropped += 1;
        }

        stepCount += 1;
    });

    // Pulled back from upstream's framing, which is set up for a floor 16x wider.
    let cameraPosition = {
        eye: {x: 0.0, y: 160.0, z: 330.0},
        target: {x: 0.0, y: 0.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
