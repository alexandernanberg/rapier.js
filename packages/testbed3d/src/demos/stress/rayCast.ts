import type {Testbed} from "../../Testbed";

type RAPIER_API = typeof import("@alexandernanberg/rapier3d");

const _translation = {x: 0.0, y: 0.0, z: 0.0};

/**
 * Points spread over a sphere of the given radius, used as the ray origins. A
 * lat/long grid, so the poles are denser than the equator — which is fine here,
 * the point is to have a few thousand rays pointing inwards from every side.
 */
function sphereOrigins(radius: number, nsubdivs: number) {
    let origins = [];
    let i, j;

    for (i = 0; i <= nsubdivs; ++i) {
        let theta = (i / nsubdivs) * Math.PI;

        for (j = 0; j < nsubdivs; ++j) {
            let phi = (j / nsubdivs) * Math.PI * 2.0;
            origins.push({
                x: radius * Math.sin(theta) * Math.cos(phi),
                y: radius * Math.cos(theta),
                z: radius * Math.sin(theta) * Math.sin(phi),
            });
        }
    }

    return origins;
}

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    // Create Ground.
    let groundSize = 200.1;
    let groundHeight = 0.1;
    let groundDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(0.0, -groundHeight, 0.0);
    let ground = world.createRigidBody(groundDesc);
    world.createCollider(RAPIER.ColliderDesc.cuboid(groundSize, groundHeight, groundSize), ground);

    // Create the cubes.
    let num = 10;
    let rad = 1.0;

    let shift = rad * 2.0;
    let centerx = shift * (num / 2);
    let centery = shift / 2.0;
    let centerz = shift * (num / 2);

    let offset = -num * (rad * 2.0) * 0.5;
    let i, j, k;

    for (j = 0; j < num; ++j) {
        for (i = 0; i < num; ++i) {
            for (k = 0; k < num; ++k) {
                let x = i * shift - centerx + offset;
                let y = j * shift + centery;
                let z = k * shift - centerz + offset;

                let bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
                let body = world.createRigidBody(bodyDesc);
                world.createCollider(RAPIER.ColliderDesc.cuboid(rad, rad, rad), body);
            }
        }

        offset -= 0.05 * rad * (num - 1.0);
    }

    testbed.setWorld(world);

    // Rays are cast inwards from a sphere around the scene, every step.
    let rayBallRadius = 100.0;
    let maxToi = rayBallRadius - 1.0;
    let origins = sphereOrigins(rayBallRadius, 60);
    let ray = new RAPIER.Ray({x: 0.0, y: 0.0, z: 0.0}, {x: 0.0, y: 0.0, z: 0.0});

    testbed.setpostTimestepAction(() => {
        // Re-center the rays on the current position of all the bodies, so a demo
        // whose objects have fallen away doesn't end up casting into the void.
        let centerx = 0.0;
        let centery = 0.0;
        let centerz = 0.0;
        let count = 0;

        testbed.world.forEachRigidBody((body) => {
            body.translation(_translation);
            centerx += _translation.x;
            centery += _translation.y;
            centerz += _translation.z;
            count += 1;
        });

        if (count > 0) {
            centerx /= count;
            centery /= count;
            centerz /= count;
        }

        let hits = 0;
        let t0 = performance.now();

        origins.forEach((origin) => {
            ray.origin.x = centerx + origin.x;
            ray.origin.y = centery + origin.y;
            ray.origin.z = centerz + origin.z;
            // Each ray points back at the center of the sphere it sits on.
            ray.dir.x = -origin.x / rayBallRadius;
            ray.dir.y = -origin.y / rayBallRadius;
            ray.dir.z = -origin.z / rayBallRadius;

            if (testbed.world.castRay(ray, maxToi, true) !== null) {
                hits += 1;
            }
        });

        let elapsed = performance.now() - t0;

        testbed.setDemoText(
            `Ray count: ${origins.length}\nRay hits: ${hits}\nRay-cast time: ${elapsed.toFixed(2)}ms`,
        );
    });

    let cameraPosition = {
        eye: {x: 100.0, y: 100.0, z: 100.0},
        target: {x: 0.0, y: 0.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}
