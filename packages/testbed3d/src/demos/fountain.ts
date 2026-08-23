import type * as RAPIER_NS from "@alexandernanberg/rapier3d";
import type {RigidBodyHandle} from "@alexandernanberg/rapier3d";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof RAPIER_NS;

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    const gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    const world = new RAPIER.World(gravity);
    const removableBodies: RigidBodyHandle[] = [];

    // Create Ground.
    const groundBodyDesc = RAPIER.RigidBodyDesc.fixed();
    const groundBody = world.createRigidBody(groundBodyDesc);
    const groundColliderDesc = RAPIER.ColliderDesc.cuboid(40.0, 0.1, 40.0);
    world.createCollider(groundColliderDesc, groundBody);

    // Dynamic cubes.
    const rad = 1.0;
    let j = 0;
    const spawn_interval = 5;

    const spawnBodies = (graphics: Testbed["graphics"]) => {
        j += 1;
        if (j % spawn_interval != 0) {
            return;
        }

        const bodyDesc = RAPIER.RigidBodyDesc.dynamic()
            .setLinvel(0.0, 15.0, 0.0)
            .setTranslation(0.0, 10.0, 0.0);
        let colliderDesc;

        switch ((j / spawn_interval) % 4) {
            case 0:
                colliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad, rad);
                break;
            case 1:
                colliderDesc = RAPIER.ColliderDesc.ball(rad);
                break;
            case 2:
                colliderDesc = RAPIER.ColliderDesc.roundCylinder(rad, rad, rad / 10.0);
                break;
            // `(j / spawn_interval) % 4` is always 0..3 — using `default` for the last
            // case lets the compiler see `colliderDesc` is always assigned.
            default:
                colliderDesc = RAPIER.ColliderDesc.cone(rad, rad);
                break;
        }

        // Use testbed.world instead of captured world so it works after snapshot restore.
        const body = testbed.world.createRigidBody(bodyDesc);
        const collider = testbed.world.createCollider(colliderDesc, body);
        graphics.addCollider(RAPIER, testbed.world, collider);

        removableBodies.push(body.handle);

        // We reached the max number, delete the oldest rigid-body.
        if (removableBodies.length > 400) {
            const rbHandle = removableBodies[0];
            const rb = testbed.world.getRigidBody(rbHandle);

            if (rb !== null) {
                testbed.world.removeRigidBody(rb);
                graphics.removeRigidBody(rb);
            }

            removableBodies.shift();
        }
    };

    testbed.setWorld(world);
    testbed.setpreTimestepAction(spawnBodies);

    const cameraPosition = {
        eye: {
            x: -88.48024008669711,
            y: 46.911325612198354,
            z: 83.56055570254844,
        },
        target: {x: 0.0, y: 10.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
