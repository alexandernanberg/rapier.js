import type * as RAPIER_NS from "@alexandernanberg/rapier3d";
import type RAPIER from "@alexandernanberg/rapier3d";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof RAPIER_NS;

function createPrismaticJoints(
    RAPIER: RAPIER_API,
    world: RAPIER.World,
    origin: RAPIER.Vector,
    num: number,
) {
    const rad = 0.4;
    const shift = 1.0;

    const groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(origin.x, origin.y, origin.z);
    let currParent = world.createRigidBody(groundDesc);
    const groundColliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad, rad);
    world.createCollider(groundColliderDesc, currParent);

    let i;
    let z;

    for (i = 0; i < num; ++i) {
        z = origin.z + (i + 1) * shift;
        const rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(origin.x, origin.y, z);
        const currChild = world.createRigidBody(rigidBodyDesc);
        const colliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad, rad);
        world.createCollider(colliderDesc, currChild);

        let axis;

        if (i % 2 == 0) {
            axis = new RAPIER.Vector3(1.0, 1.0, 0.0);
        } else {
            axis = new RAPIER.Vector3(-1.0, 1.0, 0.0);
        }

        z = new RAPIER.Vector3(0.0, 0.0, 1.0);
        const prism = RAPIER.JointData.prismatic(
            new RAPIER.Vector3(0.0, 0.0, 0.0),
            new RAPIER.Vector3(0.0, 0.0, -shift),
            axis,
        );
        prism.limitsEnabled = true;
        prism.limits = [-2.0, 2.0];
        world.createImpulseJoint(prism, currParent, currChild, true);

        currParent = currChild;
    }
}

function createRevoluteJoints(
    RAPIER: RAPIER_API,
    world: RAPIER.World,
    origin: RAPIER.Vector3,
    num: number,
) {
    const rad = 0.4;
    const shift = 2.0;

    const groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(origin.x, origin.y, 0.0);
    let currParent = world.createRigidBody(groundDesc);
    const groundColliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad, rad);
    world.createCollider(groundColliderDesc, currParent);

    let i, k;
    let z;

    for (i = 0; i < num; ++i) {
        // Create four bodies.
        z = origin.z + i * shift * 2.0 + shift;

        const positions = [
            new RAPIER.Vector3(origin.x, origin.y, z),
            new RAPIER.Vector3(origin.x + shift, origin.y, z),
            new RAPIER.Vector3(origin.x + shift, origin.y, z + shift),
            new RAPIER.Vector3(origin.x, origin.y, z + shift),
        ];

        const parents = [currParent, currParent, currParent, currParent];

        for (k = 0; k < 4; ++k) {
            const rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
                positions[k].x,
                positions[k].y,
                positions[k].z,
            );
            const rigidBody = world.createRigidBody(rigidBodyDesc);
            const colliderDesc = RAPIER.ColliderDesc.cuboid(rad, rad, rad);
            world.createCollider(colliderDesc, rigidBody);

            parents[k] = rigidBody;
        }

        // Setup four joints.
        const o = new RAPIER.Vector3(0.0, 0.0, 0.0);
        const x = new RAPIER.Vector3(1.0, 0.0, 0.0);
        z = new RAPIER.Vector3(0.0, 0.0, 1.0);

        const revs = [
            RAPIER.JointData.revolute(o, new RAPIER.Vector3(0.0, 0.0, -shift), z),
            RAPIER.JointData.revolute(o, new RAPIER.Vector3(-shift, 0.0, 0.0), x),
            RAPIER.JointData.revolute(o, new RAPIER.Vector3(0.0, 0.0, -shift), z),
            RAPIER.JointData.revolute(o, new RAPIER.Vector3(shift, 0.0, 0.0), x),
        ];

        world.createImpulseJoint(revs[0], currParent, parents[0], true);
        world.createImpulseJoint(revs[1], parents[0], parents[1], true);
        world.createImpulseJoint(revs[2], parents[1], parents[2], true);
        world.createImpulseJoint(revs[3], parents[2], parents[3], true);

        currParent = parents[3];
    }
}

function createFixedJoints(
    RAPIER: RAPIER_API,
    world: RAPIER.World,
    origin: RAPIER.Vector3,
    num: number,
) {
    const rad = 0.4;
    const shift = 1.0;
    let i, k;
    const parents = [];

    for (k = 0; k < num; ++k) {
        for (i = 0; i < num; ++i) {
            const fk = k;
            const fi = i;

            // NOTE: the num - 2 test is to avoid two consecutive
            // fixed bodies. Because physx will crash if we add
            // a joint between these.
            let bodyType;

            if (i == 0 && ((k % 4 == 0 && k != num - 2) || k == num - 1)) {
                bodyType = RAPIER.RigidBodyType.Fixed;
            } else {
                bodyType = RAPIER.RigidBodyType.Dynamic;
            }

            const rigidBody = new RAPIER.RigidBodyDesc(bodyType).setTranslation(
                origin.x + fk * shift,
                origin.y,
                origin.z + fi * shift,
            );
            const child = world.createRigidBody(rigidBody);
            const colliderDesc = RAPIER.ColliderDesc.ball(rad);
            world.createCollider(colliderDesc, child);

            // Vertical joint.
            if (i > 0) {
                const parent = parents[parents.length - 1];
                const params = RAPIER.JointData.fixed(
                    new RAPIER.Vector3(0.0, 0.0, 0.0),
                    new RAPIER.Quaternion(0.0, 0.0, 0.0, 1.0),
                    new RAPIER.Vector3(0.0, 0.0, -shift),
                    new RAPIER.Quaternion(0.0, 0.0, 0.0, 1.0),
                );

                world.createImpulseJoint(params, parent, child, true);
            }

            // Horizontal joint.
            if (k > 0) {
                const parent_index = parents.length - num;
                const parent = parents[parent_index];
                const params = RAPIER.JointData.fixed(
                    new RAPIER.Vector3(0.0, 0.0, 0.0),
                    new RAPIER.Quaternion(0.0, 0.0, 0.0, 1.0),
                    new RAPIER.Vector3(-shift, 0.0, 0.0),
                    new RAPIER.Quaternion(0.0, 0.0, 0.0, 1.0),
                );

                world.createImpulseJoint(params, parent, child, true);
            }

            parents.push(child);
        }
    }
}

function createBallJoints(RAPIER: RAPIER_API, world: RAPIER.World, num: number) {
    const rad = 0.4;
    const shift = 1.0;
    let i, k;
    const parents = [];

    for (k = 0; k < num; ++k) {
        for (i = 0; i < num; ++i) {
            const fk = k;
            const fi = i;

            let bodyType;

            if (i == 0 && (k % 4 == 0 || k == num - 1)) {
                bodyType = RAPIER.RigidBodyType.Fixed;
            } else {
                bodyType = RAPIER.RigidBodyType.Dynamic;
            }

            const bodyDesc = new RAPIER.RigidBodyDesc(bodyType).setTranslation(
                fk * shift,
                0.0,
                fi * shift,
            );
            const child = world.createRigidBody(bodyDesc);
            const colliderDesc = RAPIER.ColliderDesc.ball(rad);
            world.createCollider(colliderDesc, child);

            // Vertical joint.
            const o = new RAPIER.Vector3(0.0, 0.0, 0.0);

            if (i > 0) {
                const parent = parents[parents.length - 1];
                const params = RAPIER.JointData.spherical(o, new RAPIER.Vector3(0.0, 0.0, -shift));
                world.createImpulseJoint(params, parent, child, true);
            }

            // Horizontal joint.
            if (k > 0) {
                const parent_index = parents.length - num;
                const parent = parents[parent_index];
                const params = RAPIER.JointData.spherical(o, new RAPIER.Vector3(-shift, 0.0, 0.0));
                world.createImpulseJoint(params, parent, child, true);
            }

            parents.push(child);
        }
    }
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    const gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    const world = new RAPIER.World(gravity);

    createPrismaticJoints(RAPIER, world, new RAPIER.Vector3(20.0, 10.0, 0.0), 5);
    createFixedJoints(RAPIER, world, new RAPIER.Vector3(0.0, 10.0, 0.0), 5);
    createRevoluteJoints(RAPIER, world, new RAPIER.Vector3(20.0, 0.0, 0.0), 3);
    createBallJoints(RAPIER, world, 15);

    testbed.setWorld(world);
    const cameraPosition = {
        eye: {x: 15.0, y: 5.0, z: 42.0},
        target: {x: 13.0, y: 1.0, z: 1.0},
    };
    testbed.lookAt(cameraPosition);
}
