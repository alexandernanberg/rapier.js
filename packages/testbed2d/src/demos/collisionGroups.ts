import type * as RAPIER_NS from "@alexandernanberg/rapier2d";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof RAPIER_NS;

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    const gravity = new RAPIER.Vector2(0.0, -9.81);
    const world = new RAPIER.World(gravity);

    /*
     * Ground
     */
    const ground_size = 5.0;
    const ground_height = 0.1;

    const groundBodyDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0.0, -ground_height);
    const groundBody = world.createRigidBody(groundBodyDesc);
    let groundColliderDesc = RAPIER.ColliderDesc.cuboid(ground_size, ground_height);
    world.createCollider(groundColliderDesc, groundBody);

    /*
     * Setup groups
     */
    const group1 = 0x00010001;
    const group2 = 0x00020002;

    /*
     * A green floor that will collide with the first group only.
     */
    groundColliderDesc = RAPIER.ColliderDesc.cuboid(1.0, 0.1)
        .setTranslation(0.0, 1.0)
        .setCollisionGroups(group1);
    world.createCollider(groundColliderDesc, groundBody);

    /*
     * A blue floor that will collide with the second group only.
     */
    groundColliderDesc = RAPIER.ColliderDesc.cuboid(1.0, 0.1)
        .setTranslation(0.0, 2.0)
        .setCollisionGroups(group2);
    world.createCollider(groundColliderDesc, groundBody);

    /*
     * Create the cubes
     */
    const num = 8;
    const rad = 0.1;

    const shift = rad * 2.0;
    const centerx = shift * (num / 2);
    const centery = 2.5;
    let i, j;

    for (j = 0; j < 4; ++j) {
        for (i = 0; i < num; ++i) {
            const x = i * shift - centerx;
            const y = j * shift + centery;

            // Alternate between the green and blue groups.
            const group = i % 2 == 0 ? group1 : group2;

            const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y);
            const body = world.createRigidBody(bodyDesc);
            const colliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad).setCollisionGroups(group);
            world.createCollider(colliderDesc, body);
        }
    }

    testbed.setWorld(world);
    testbed.lookAt({
        target: {x: 0.0, y: -1.0},
        zoom: 100.0,
    });
}
