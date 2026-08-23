import type * as RAPIER_NS from "@alexandernanberg/rapier3d";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof RAPIER_NS;

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    const gravity = new RAPIER.Vector3(0.0, 0.0, 0.0);
    const world = new RAPIER.World(gravity);

    /*
     * Create the cubes
     */
    const num = 10;
    const rad = 0.2;

    const subdiv = 1.0 / num;

    let i;
    for (i = 0; i < num; ++i) {
        const x = Math.sin(i * subdiv * Math.PI * 2.0);
        const y = Math.cos(i * subdiv * Math.PI * 2.0);

        // Build the rigid body.
        const bodyDesc = RAPIER.RigidBodyDesc.dynamic()
            .setTranslation(x, y, 0.0)
            .setLinvel(x * 10.0, y * 10.0, 0.0)
            .setAngvel(new RAPIER.Vector3(0.0, 0.0, 100.0))
            .setLinearDamping((i + 1) * subdiv * 10.0)
            .setAngularDamping((num - i) * subdiv * 10.0);
        const body = world.createRigidBody(bodyDesc);

        // Build the collider.
        const colliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad, rad);
        world.createCollider(colliderDesc, body);
    }

    testbed.setWorld(world);
    const cameraPosition = {
        eye: {x: 0, y: 2.0, z: 20},
        target: {x: 0, y: 2.0, z: 0},
    };
    testbed.lookAt(cameraPosition);
}
