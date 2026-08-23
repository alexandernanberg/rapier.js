import type * as RAPIER_NS from "@alexandernanberg/rapier2d";
import seedrandom from "seedrandom";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof RAPIER_NS;

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    const gravity = new RAPIER.Vector2(0.0, -9.81);
    const world = new RAPIER.World(gravity);

    /*
     * Ground
     */
    // Create Ground.
    const groundSize = 30.0;
    const grounds = [
        {x: 0.0, y: 0.0, hx: groundSize, hy: 1.2},
        {x: -groundSize, y: groundSize, hx: 1.2, hy: groundSize},
        {x: groundSize, y: groundSize, hx: 1.2, hy: groundSize},
    ];

    grounds.forEach((ground) => {
        const bodyDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(ground.x, ground.y);
        const body = world.createRigidBody(bodyDesc);
        const colliderDesc = RAPIER.ColliderDesc.cuboid(ground.hx, ground.hy);
        world.createCollider(colliderDesc, body);
    });

    /*
     * Create the convex polygons
     */
    const num = 14;
    const scale = 4.0;

    const shift = scale;
    const centerx = (shift * num) / 2.0;
    const centery = shift / 2.0;

    let i, j, k;
    const rng = seedrandom("convexPolygon");

    for (i = 0; i < num; ++i) {
        for (j = 0; j < num * 2; ++j) {
            const x = i * shift - centerx;
            const y = j * shift * 2.0 + centery + 2.0;

            const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y);
            const body = world.createRigidBody(bodyDesc);

            const points = [];
            for (k = 0; k < 10; ++k) {
                points.push(rng() * scale, rng() * scale);
            }
            // `convexHull` returns null if the random point set is degenerate.
            const colliderDesc = RAPIER.ColliderDesc.convexHull(new Float32Array(points));
            if (colliderDesc !== null) {
                world.createCollider(colliderDesc, body);
            }
        }
    }

    testbed.setWorld(world);
    testbed.lookAt({
        target: {x: -10.0, y: -30.0},
        zoom: 7.0,
    });
}
