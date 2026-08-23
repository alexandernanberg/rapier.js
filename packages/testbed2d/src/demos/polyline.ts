import type * as RAPIER_NS from "@alexandernanberg/rapier2d";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof RAPIER_NS;

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    const gravity = new RAPIER.Vector2(0.0, -9.81);
    const world = new RAPIER.World(gravity);
    let i, j;

    /*
     * Ground
     */
    const nsubdivs = 100;
    const points = [];
    const groundSize = 30.0;
    const stepSize = groundSize / nsubdivs;

    points.push(-groundSize / 2.0, 25.0);
    for (i = 1; i < nsubdivs; ++i) {
        const x = -groundSize / 2.0 + i * stepSize;
        const y = Math.cos(i * stepSize) * 2.0;
        points.push(x, y);
    }
    points.push(groundSize / 2.0, 25.0);
    const groundBodyDesc = RAPIER.RigidBodyDesc.fixed();
    const groundBody = world.createRigidBody(groundBodyDesc);
    const groundColliderDesc = RAPIER.ColliderDesc.polyline(new Float32Array(points));
    world.createCollider(groundColliderDesc, groundBody);

    /*
     * Create the cubes
     */
    const num = 10;
    const rad = 0.5;

    const shift = rad * 2.0;
    const centerx = shift * (num / 2);
    const centery = shift / 2.0;

    for (i = 0; i < num; ++i) {
        for (j = 0; j < num * 5; ++j) {
            const x = i * shift - centerx;
            const y = j * shift + centery + 3.0;

            // Build the rigid groundBody.
            const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y);
            const body = world.createRigidBody(bodyDesc);

            if (j % 2 == 0) {
                const colliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad);
                world.createCollider(colliderDesc, body);
            } else {
                const colliderDesc = RAPIER.ColliderDesc.ball(rad);
                world.createCollider(colliderDesc, body);
            }
        }
    }

    testbed.setWorld(world);
    testbed.lookAt({
        target: {x: -10.0, y: -15.0},
        zoom: 10.0,
    });
}
