import type * as RAPIER_NS from "@alexandernanberg/rapier3d";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof RAPIER_NS;

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    const gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    const world = new RAPIER.World(gravity);

    // Create Ground.
    const groundBodyDesc = RAPIER.RigidBodyDesc.fixed();
    const groundBody = world.createRigidBody(groundBodyDesc);
    let colliderDesc = RAPIER.ColliderDesc.cuboid(5.0, 0.1, 5.0);
    world.createCollider(colliderDesc, groundBody);

    // Setup groups.
    const group1 = 0x00010001;
    const group2 = 0x00020002;

    // Add one floor that collides with the first group only.
    colliderDesc = RAPIER.ColliderDesc.cuboid(1.0, 0.1, 1.0)
        .setTranslation(0.0, 1.0, 0.0)
        .setCollisionGroups(group1);
    world.createCollider(colliderDesc, groundBody);

    // Add one floor that collides with the second group only.
    colliderDesc = RAPIER.ColliderDesc.cuboid(1.0, 0.1, 1.0)
        .setTranslation(0.0, 2.0, 0.0)
        .setCollisionGroups(group2);
    world.createCollider(colliderDesc, groundBody);

    // Dynamic cubes.
    const num = 8;
    const rad = 0.1;

    const shift = rad * 2.0;
    const centerx = shift * (num / 2);
    const centery = 2.5;
    const centerz = shift * (num / 2);
    let i, j, k;

    for (j = 0; j < 4; j++) {
        for (i = 0; i < num; i++) {
            for (k = 0; k < num; k++) {
                const x = i * shift - centerx;
                const y = j * shift + centery;
                const z = k * shift - centerz;

                // Alternate between the green and blue groups.
                const group = k % 2 == 0 ? group1 : group2;
                const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
                const body = world.createRigidBody(bodyDesc);

                colliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad, rad).setCollisionGroups(group);
                world.createCollider(colliderDesc, body);
            }
        }
    }

    testbed.setWorld(world);
    const cameraPosition = {
        eye: {x: 10.0, y: 5.0, z: 10.0},
        target: {x: 0.0, y: 0.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
