import type * as RAPIER_NS from "@alexandernanberg/rapier3d";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof RAPIER_NS;

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    const gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    const world = new RAPIER.World(gravity);

    // Create Ground.
    const groundBodyDesc = RAPIER.RigidBodyDesc.fixed();
    const groundBody = world.createRigidBody(groundBodyDesc);
    const groundColliderDesc = RAPIER.ColliderDesc.cuboid(30.0, 0.1, 30.0);
    world.createCollider(groundColliderDesc, groundBody);

    // Dynamic cubes.
    const rad = 0.5;
    const num = 10;
    let i, j, k;
    const shift = rad * 2.5;
    const center = num * rad;
    const height = 10.0;

    for (i = 0; i < num; ++i) {
        for (j = i; j < num; ++j) {
            for (k = i; k < num; ++k) {
                const x = (i * shift) / 2.0 + (k - i) * shift - height * rad - center;
                const y = i * shift + height;
                const z = (i * shift) / 2.0 + (j - i) * shift - height * rad - center;

                // Create dynamic cube.
                const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
                const body = world.createRigidBody(bodyDesc);
                const colliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad, rad);
                world.createCollider(colliderDesc, body);
            }
        }
    }

    testbed.setWorld(world);
    const cameraPosition = {
        eye: {x: -31.96000000000001, y: 19.730000000000008, z: -27.86},
        target: {x: -0.0505, y: -0.4126, z: -0.0229},
    };
    testbed.lookAt(cameraPosition);
}
