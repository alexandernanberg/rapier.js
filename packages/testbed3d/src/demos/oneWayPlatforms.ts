import type RAPIER from "@alexandernanberg/rapier3d";
import type {Testbed} from "../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

const _normal = {x: 0.0, y: 0.0, z: 0.0};
const _tangentVelocity = {x: 0.0, y: 0.0, z: 0.0};
const _translation = {x: 0.0, y: 0.0, z: 0.0};

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Two platforms on the same fixed body. Both let a cube through from one side
    // only, and both act as conveyor belts once the cube lands on them.
    let ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    let platform1 = world.createCollider(
        RAPIER.ColliderDesc.cuboid(9.0, 0.5, 25.0)
            .setTranslation(0.0, 2.0, 30.0)
            .setActiveHooks(RAPIER.ActiveHooks.MODIFY_SOLVER_CONTACTS),
        ground,
    ).handle;
    let platform2 = world.createCollider(
        RAPIER.ColliderDesc.cuboid(9.0, 0.5, 25.0)
            .setTranslation(0.0, -2.0, -30.0)
            .setActiveHooks(RAPIER.ActiveHooks.MODIFY_SOLVER_CONTACTS),
        ground,
    ).handle;

    // The contact normal points from the first collider towards the second one, so
    // the direction a platform accepts contacts from flips depending on which side
    // of the pair the platform ended up on. The platforms never rotate, so this
    // world-space normal is also their local one.
    const hooks: RAPIER.PhysicsHooks = {
        modifySolverContacts(context) {
            let collider1 = context.collider1();
            let collider2 = context.collider2();
            let allowedNormalY = 0.0;

            if (collider1 == platform1) {
                allowedNormalY = 1.0;
            } else if (collider2 == platform1) {
                allowedNormalY = -1.0;
            }

            if (collider1 == platform2) {
                allowedNormalY = -1.0;
            } else if (collider2 == platform2) {
                allowedNormalY = 1.0;
            }

            // Anything hitting the platform from the other side goes through: drop
            // every contact of this manifold so the solver never sees it.
            let normal = context.normal(_normal);

            if (normal === null || normal.y * allowedNormalY < Math.cos(0.1)) {
                context.clearSolverContacts();
                return;
            }

            // Set the surface velocity of the contacts that were kept, which is what
            // makes the platform act as a conveyor belt.
            _tangentVelocity.z = collider1 == platform1 || collider2 == platform2 ? -12.0 : 12.0;

            for (let i = 0; i < context.numSolverContacts(); ++i) {
                context.setSolverContactTangentVelocity(i, _tangentVelocity);
            }
        },
    };

    testbed.setWorld(world);
    testbed.setPhysicsHooks(hooks);

    let stepId = 0;
    let numCubes = 0;

    testbed.setpreTimestepAction((graphics) => {
        stepId += 1;

        // Spawn cubes at regular intervals…
        if (stepId % 200 == 0 && numCubes < 7) {
            let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(0.0, 6.0, 20.0);
            let body = testbed.world.createRigidBody(bodyDesc);
            let collider = testbed.world.createCollider(
                RAPIER.ColliderDesc.cuboid(1.0, 2.0, 1.5),
                body,
            );
            graphics.addCollider(RAPIER, testbed.world, collider);
            numCubes += 1;
        }

        // …and flip their gravity depending on which platform they are riding, so
        // they keep looping between the two.
        testbed.world.forEachActiveRigidBody((body) => {
            let y = body.translation(_translation).y;

            if (y > 1.0) {
                body.setGravityScale(1.0, false);
            } else if (y < -1.0) {
                body.setGravityScale(-1.0, false);
            }
        });
    });

    let cameraPosition = {
        eye: {x: 100.0, y: 0.0, z: 0.0},
        target: {x: 0.0, y: 0.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
