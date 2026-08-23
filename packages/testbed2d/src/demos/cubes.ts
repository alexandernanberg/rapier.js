import type * as RAPIER_NS from "@alexandernanberg/rapier2d";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof RAPIER_NS;

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    const gravity = new RAPIER.Vector2(0.0, -9.81);
    const world = new RAPIER.World(gravity);

    // Create Ground.
    const groundSize = 40.0;
    const grounds = [
        {x: 0.0, y: 0.0, hx: groundSize, hy: 0.1},
        {x: -groundSize, y: groundSize, hx: 0.1, hy: groundSize},
        {x: groundSize, y: groundSize, hx: 0.1, hy: groundSize},
    ];

    grounds.forEach((ground) => {
        const bodyDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(ground.x, ground.y);
        const body = world.createRigidBody(bodyDesc);
        const colliderDesc = RAPIER.ColliderDesc.cuboid(ground.hx, ground.hy);
        world.createCollider(colliderDesc, body);
    });

    // Dynamic cubes.
    const num = 20;
    const numy = 50;
    const rad = 1.0;

    const shift = rad * 2.0 + rad;
    const centerx = shift * (num / 2);
    const centery = shift / 2.0;

    let i, j;

    for (j = 0; j < numy; ++j) {
        for (i = 0; i < num; ++i) {
            const x = i * shift - centerx;
            const y = j * shift + centery + 3.0;

            // Create dynamic cube.
            const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y);
            const body = world.createRigidBody(bodyDesc);
            const colliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad);
            world.createCollider(colliderDesc, body);
        }
    }

    testbed.setWorld(world);
    testbed.lookAt({
        target: {x: -10.0, y: -30.0},
        zoom: 7.0,
    });
}
