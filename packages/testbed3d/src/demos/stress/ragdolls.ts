import type RAPIER from "@alexandernanberg/rapier3d";
import type {Testbed} from "../../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

/** A quarter turn about `z`, which lays a capsule down along the `x` axis. */
const CAPSULE_X = {x: 0.0, y: 0.0, z: Math.SQRT1_2, w: Math.SQRT1_2};

interface Part {
    body: RAPIER.RigidBody;
    offset: RAPIER.Vector3;
}

/**
 * A 10-body articulated ragdoll (torso, head, 2x2 arm links, 2x2 leg links)
 * whose torso center is at `origin`. 9 joints, all with limits: spherical neck,
 * shoulders and hips; revolute elbows and knees.
 */
function ragdoll(RAPIER: RAPIER_API, world: RAPIER.World, origin: RAPIER.Vector3) {
    let part = (offset: RAPIER.Vector3, colliderDesc: RAPIER.ColliderDesc): Part => {
        let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
            origin.x + offset.x,
            origin.y + offset.y,
            origin.z + offset.z,
        );
        let body = world.createRigidBody(bodyDesc);
        world.createCollider(colliderDesc, body);
        return {body, offset};
    };

    // Anchors are given in the ragdoll's frame, so each side of a joint gets the
    // anchor minus the body's own offset.
    let anchorFor = (p: Part, anchor: RAPIER.Vector3) =>
        new RAPIER.Vector3(anchor.x - p.offset.x, anchor.y - p.offset.y, anchor.z - p.offset.z);

    let spherical = (parent: Part, child: Part, anchor: RAPIER.Vector3, limit: number) => {
        let joint = world.createImpulseJoint(
            RAPIER.JointData.spherical(anchorFor(parent, anchor), anchorFor(child, anchor)),
            parent.body,
            child.body,
            true,
        ) as RAPIER.SphericalImpulseJoint;
        joint.setLimits(RAPIER.JointAxis.AngX, -limit, limit);
        joint.setLimits(RAPIER.JointAxis.AngY, -limit, limit);
        joint.setLimits(RAPIER.JointAxis.AngZ, -limit, limit);
        joint.setContactsEnabled(false);
    };

    let revolute = (
        parent: Part,
        child: Part,
        anchor: RAPIER.Vector3,
        limits: [number, number],
    ) => {
        let joint = world.createImpulseJoint(
            RAPIER.JointData.revolute(
                anchorFor(parent, anchor),
                anchorFor(child, anchor),
                new RAPIER.Vector3(0.0, 0.0, 1.0),
            ),
            parent.body,
            child.body,
            true,
        ) as RAPIER.RevoluteImpulseJoint;
        joint.setLimits(limits[0], limits[1]);
        joint.setContactsEnabled(false);
    };

    let torso = part(new RAPIER.Vector3(0.0, 0.0, 0.0), RAPIER.ColliderDesc.capsule(0.3, 0.15));
    let head = part(new RAPIER.Vector3(0.0, 0.55, 0.0), RAPIER.ColliderDesc.ball(0.15));
    spherical(torso, head, new RAPIER.Vector3(0.0, 0.42, 0.0), 0.5);

    [-1.0, 1.0].forEach((side) => {
        let upperArm = part(
            new RAPIER.Vector3(side * 0.36, 0.25, 0.0),
            RAPIER.ColliderDesc.capsule(0.14, 0.06).setRotation(CAPSULE_X),
        );
        let forearm = part(
            new RAPIER.Vector3(side * 0.7, 0.25, 0.0),
            RAPIER.ColliderDesc.capsule(0.14, 0.06).setRotation(CAPSULE_X),
        );
        let thigh = part(
            new RAPIER.Vector3(side * 0.09, -0.52, 0.0),
            RAPIER.ColliderDesc.capsule(0.16, 0.07),
        );
        let shin = part(
            new RAPIER.Vector3(side * 0.09, -0.92, 0.0),
            RAPIER.ColliderDesc.capsule(0.16, 0.07),
        );

        spherical(torso, upperArm, new RAPIER.Vector3(side * 0.19, 0.25, 0.0), 1.2);
        revolute(upperArm, forearm, new RAPIER.Vector3(side * 0.53, 0.25, 0.0), [0.0, 2.5]);
        spherical(torso, thigh, new RAPIER.Vector3(side * 0.09, -0.33, 0.0), 1.0);
        revolute(thigh, shin, new RAPIER.Vector3(side * 0.09, -0.72, 0.0), [0.0, 2.3]);
    });
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Create Ground.
    let groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0.0, -1.0, 0.0);
    let ground = world.createRigidBody(groundDesc);
    world.createCollider(RAPIER.ColliderDesc.cuboid(100.0, 1.0, 100.0), ground);

    // Ragdolls dropped into a pile: a 5x5 grid, 5 layers (125 ragdolls, 1250
    // dynamic bodies, 1125 limit joints).
    let layer, row, col;

    for (layer = 0; layer < 5; ++layer) {
        for (row = 0; row < 5; ++row) {
            for (col = 0; col < 5; ++col) {
                ragdoll(RAPIER, world, new RAPIER.Vector3(col * 2.2, 1.5 + layer * 2.6, row * 2.2));
            }
        }
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: -12.0, y: 10.0, z: -12.0},
        target: {x: 4.5, y: 1.0, z: 4.5},
    };
    testbed.lookAt(cameraPosition);
}
