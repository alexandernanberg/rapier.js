import type * as RAPIER_NS from "@alexandernanberg/rapier2d";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof RAPIER_NS;

function generateVoxels(n: number) {
    const points = [];

    let i;
    for (i = 0; i <= n; ++i) {
        const y = Math.max(-0.8, Math.min(Math.sin((i / n) * 10.0), 0.8)) * 8.0;
        points.push(i - n / 2.0, y);
    }
    return {
        points: new Float32Array(points),
        voxelSize: {x: 1.0, y: 1.2},
    };
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    const gravity = new RAPIER.Vector2(0.0, -9.81);
    const world = new RAPIER.World(gravity);

    // Create Ground.
    const groundBodyDesc = RAPIER.RigidBodyDesc.fixed();
    const groundBody = world.createRigidBody(groundBodyDesc);
    const voxels = generateVoxels(100);
    const groundColliderDesc = RAPIER.ColliderDesc.voxels(voxels.points, voxels.voxelSize);
    world.createCollider(groundColliderDesc, groundBody);

    // Dynamic cubes.
    const num = 10;
    const numy = 4;
    const rad = 1.0;

    const shift = rad * 2.0 + rad;
    const centery = shift / 2.0;

    let offset = -num * (rad * 2.0 + rad) * 0.5;
    let i, j;

    for (j = 0; j < numy; ++j) {
        for (i = 0; i < num; ++i) {
            const x = i * shift + offset;
            const y = j * shift + centery + 10.0;

            // Create dynamic cube.
            const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y);
            const body = world.createRigidBody(bodyDesc);
            let colliderDesc;

            switch (j % 3) {
                case 0:
                    colliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad);
                    break;
                case 1:
                    colliderDesc = RAPIER.ColliderDesc.ball(rad);
                    break;
                // `j % 3` is always 0, 1 or 2 — using `default` for the last case
                // lets the compiler see `colliderDesc` is always assigned.
                default:
                    colliderDesc = RAPIER.ColliderDesc.cuboid(rad / 2.0, rad / 2.0);
                    world.createCollider(colliderDesc, body);
                    colliderDesc = RAPIER.ColliderDesc.cuboid(rad / 2.0, rad).setTranslation(
                        rad,
                        0.0,
                    );
                    world.createCollider(colliderDesc, body);
                    colliderDesc = RAPIER.ColliderDesc.cuboid(rad / 2.0, rad).setTranslation(
                        -rad,
                        0.0,
                    );
                    break;
            }

            world.createCollider(colliderDesc, body);
        }

        offset -= 0.05 * rad * (num - 1.0);
    }

    testbed.setWorld(world);
    testbed.lookAt({
        target: {x: 0.0, y: 0.0},
        zoom: 20.0,
    });
}
