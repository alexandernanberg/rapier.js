import type RAPIER from "@alexandernanberg/rapier3d";
import type {Testbed} from "../../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

function createPyramid(
    RAPIER: RAPIER_API,
    world: RAPIER.World,
    offset: RAPIER.Vector3,
    stackHeight: number,
    rad: number,
) {
    let shift = rad * 2.0;
    let i, j;

    for (i = 0; i < stackHeight; ++i) {
        for (j = i; j < stackHeight; ++j) {
            let x = (i * shift) / 2.0 + (j - i) * shift;
            let y = i * shift;

            let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
                x + offset.x,
                y + offset.y,
                offset.z,
            );
            let body = world.createRigidBody(bodyDesc);
            world.createCollider(RAPIER.ColliderDesc.cuboid(rad, rad, rad), body);
        }
    }
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    let rad = 0.5;
    // Upstream lines up 40 pyramids (8400 boxes); 16 keeps it around 3.4k.
    let pyramidCount = 16;
    let spacing = 4.0;

    // Create Ground.
    let groundSize = 50.0;
    let groundHeight = 0.1;
    let groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0.0, -groundHeight, 0.0);
    let ground = world.createRigidBody(groundDesc);
    world.createCollider(
        RAPIER.ColliderDesc.cuboid(
            groundSize,
            groundHeight,
            (pyramidCount * spacing) / 2.0 + groundSize,
        ),
        ground,
    );

    let i;

    for (i = 0; i < pyramidCount; ++i) {
        createPyramid(
            RAPIER,
            world,
            new RAPIER.Vector3(0.0, rad, (i - pyramidCount / 2.0) * spacing),
            20,
            rad,
        );
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: 100.0, y: 100.0, z: 100.0},
        target: {x: 0.0, y: 0.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
