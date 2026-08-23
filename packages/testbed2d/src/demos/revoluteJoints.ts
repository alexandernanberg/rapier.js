import type * as RAPIER_NS from "@alexandernanberg/rapier2d";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof RAPIER_NS;

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    const gravity = new RAPIER.Vector2(0.0, -9.81);
    const world = new RAPIER.World(gravity);
    const bodies = [];

    const rad = 0.4;
    const numi = 30; // Num vertical nodes.
    const numk = 30; // Num horizontal nodes.
    const shift = 1.0;
    let i, k;

    for (k = 0; k < numk; ++k) {
        for (i = 0; i < numi; ++i) {
            const status =
                k >= numk / 2 - 3 && k <= numk / 2 + 3 && i == 0
                    ? RAPIER.RigidBodyType.Fixed
                    : RAPIER.RigidBodyType.Dynamic;

            const bodyDesc = new RAPIER.RigidBodyDesc(status).setTranslation(k * shift, -i * shift);
            const child = world.createRigidBody(bodyDesc);
            const colliderDesc = RAPIER.ColliderDesc.ball(rad);
            world.createCollider(colliderDesc, child);

            // Vertical joint.
            if (i > 0) {
                const parent = bodies[bodies.length - 1];
                const anchor1 = new RAPIER.Vector2(0.0, 0.0);
                const anchor2 = new RAPIER.Vector2(0.0, shift);
                const JointData = RAPIER.JointData.revolute(anchor1, anchor2);
                world.createImpulseJoint(JointData, parent, child, true);
            }

            // Horizontal joint.
            if (k > 0) {
                const parentIndex = bodies.length - numi;
                const parent = bodies[parentIndex];
                const anchor1 = new RAPIER.Vector2(0.0, 0.0);
                const anchor2 = new RAPIER.Vector2(-shift, 0.0);
                const JointData = RAPIER.JointData.revolute(anchor1, anchor2);
                world.createImpulseJoint(JointData, parent, child, true);
            }

            bodies.push(child);
        }
    }

    testbed.setWorld(world);
    testbed.lookAt({
        target: {x: 30.0, y: 30.0},
        zoom: 10.0,
    });
}
