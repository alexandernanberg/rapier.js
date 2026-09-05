import type RAPIER from "@alexandernanberg/rapier3d";
import type {Testbed} from "../../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

function createSmallPyramid(
    RAPIER: RAPIER_API,
    world: RAPIER.World,
    baseCount: number,
    extent: number,
    centerX: number,
    baseZ: number,
) {
    let i, j;

    for (i = 0; i < baseCount; ++i) {
        let y = (2.0 * i + 1.0) * extent;

        for (j = i; j < baseCount; ++j) {
            let x = (i + 1.0) * extent + 2.0 * (j - i) * extent + centerX - 0.5;

            let bodyDesc = RAPIER.RigidBodyDesc.dynamic()
                .setTranslation(x, y, baseZ)
                .setCanSleep(false);
            let body = world.createRigidBody(bodyDesc);
            world.createCollider(
                RAPIER.ColliderDesc.cuboid(extent, extent, extent).setDensity(100.0),
                body,
            );
        }
    }
}

/**
 * Port of box3d's `many_pyramids` benchmark (`CreateManyPyramids`,
 * `box3d/shared/benchmarks.c`). Release settings: a grid of 10-base pyramids of
 * small cubes (density 100) on a ground box, sleeping disabled.
 */
export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    // box3d's `b3DefaultWorldDef`: gravity (0, -10, 0). box3d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let gravity = new RAPIER.Vector3(0.0, -10.0, 0.0);
    let world = new RAPIER.World(gravity);

    let baseCount = 10;
    let extent = 0.5;
    // Upstream lays out 14x14 pyramids (10,780 boxes); 8x8 keeps it around 3.5k.
    let rowCount = 8;
    let columnCount = 8;
    let groundExtent = extent * columnCount * (baseCount + 1.0);

    // Create Ground.
    let groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0.0, -1.0, 0.0);
    let ground = world.createRigidBody(groundDesc);
    world.createCollider(RAPIER.ColliderDesc.cuboid(groundExtent, 1.0, groundExtent), ground);

    let baseWidth = 2.0 * extent * baseCount;
    let baseZ = -groundExtent + 2.0 * extent;
    let deltaZ = (2.0 * (groundExtent - 2.0 * extent)) / (rowCount - 1.0);
    let i, j;

    for (i = 0; i < rowCount; ++i) {
        for (j = 0; j < columnCount; ++j) {
            let centerX = -groundExtent + j * (baseWidth + 2.0 * extent) + 2.0 * extent;
            createSmallPyramid(RAPIER, world, baseCount, extent, centerX, baseZ);
        }

        baseZ += deltaZ;
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: 0.0, y: 30.0, z: 120.0},
        target: {x: 0.0, y: 5.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
