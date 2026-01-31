import type {BenchResult} from "../runner.js";
import {bench} from "../runner.js";

export async function benchSetters(
    RAPIER: any,
    is3D: boolean,
    quick: boolean,
): Promise<BenchResult[]> {
    const results: BenchResult[] = [];
    const bodyCount = 1000;
    const opts = quick ? {warmup: 50, iterations: 200} : {warmup: 100, iterations: 1000};

    const gravity = is3D ? {x: 0, y: -9.81, z: 0} : {x: 0, y: -9.81};
    const world = new RAPIER.World(gravity);

    // Create dynamic bodies for transform tests
    const dynamicBodies: any[] = [];
    for (let i = 0; i < bodyCount; i++) {
        const bodyDesc = RAPIER.RigidBodyDesc.dynamic();
        if (is3D) {
            bodyDesc.setTranslation(
                Math.random() * 100 - 50,
                Math.random() * 100,
                Math.random() * 100 - 50,
            );
        } else {
            bodyDesc.setTranslation(Math.random() * 100 - 50, Math.random() * 100);
        }
        const body = world.createRigidBody(bodyDesc);
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        dynamicBodies.push(body);
    }

    // Create kinematic bodies for kinematic transform tests
    const kinematicBodies: any[] = [];
    for (let i = 0; i < bodyCount; i++) {
        const bodyDesc = RAPIER.RigidBodyDesc.kinematicPositionBased();
        if (is3D) {
            bodyDesc.setTranslation(
                Math.random() * 100 - 50,
                Math.random() * 100,
                Math.random() * 100 - 50,
            );
        } else {
            bodyDesc.setTranslation(Math.random() * 100 - 50, Math.random() * 100);
        }
        const body = world.createRigidBody(bodyDesc);
        world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        kinematicBodies.push(body);
    }

    // Prepare test data
    const translations3D = dynamicBodies.map(() => ({
        x: Math.random() * 100 - 50,
        y: Math.random() * 100,
        z: Math.random() * 100 - 50,
    }));
    const translations2D = dynamicBodies.map(() => ({
        x: Math.random() * 100 - 50,
        y: Math.random() * 100,
    }));
    const rotations3D = dynamicBodies.map(() => {
        // Generate random quaternion
        const u1 = Math.random();
        const u2 = Math.random() * Math.PI * 2;
        const u3 = Math.random() * Math.PI * 2;
        const a = Math.sqrt(1 - u1);
        const b = Math.sqrt(u1);
        return {
            x: a * Math.sin(u2),
            y: a * Math.cos(u2),
            z: b * Math.sin(u3),
            w: b * Math.cos(u3),
        };
    });
    const rotations2D = dynamicBodies.map(() => Math.random() * Math.PI * 2);

    // Check if batch setTransform is supported (our fork only)
    const supportsBatchTransform = typeof dynamicBodies[0].setTransform === "function";

    if (is3D) {
        // Use batch setTransform when available (fork), otherwise separate calls (official)
        if (supportsBatchTransform) {
            results.push(
                bench(
                    `body.setTransform()`,
                    () => {
                        for (let i = 0; i < dynamicBodies.length; i++) {
                            dynamicBodies[i].setTransform(translations3D[i], rotations3D[i], true);
                        }
                    },
                    opts,
                ),
            );
        } else {
            results.push(
                bench(
                    `body.setTransform()`,
                    () => {
                        for (let i = 0; i < dynamicBodies.length; i++) {
                            dynamicBodies[i].setTranslation(translations3D[i], false);
                            dynamicBodies[i].setRotation(rotations3D[i], true);
                        }
                    },
                    opts,
                ),
            );
        }

        // Use batch setNextKinematicTransform when available (fork), otherwise separate calls
        if (supportsBatchTransform) {
            results.push(
                bench(
                    `body.setNextKinematicTransform()`,
                    () => {
                        for (let i = 0; i < kinematicBodies.length; i++) {
                            kinematicBodies[i].setNextKinematicTransform(
                                translations3D[i],
                                rotations3D[i],
                            );
                        }
                    },
                    opts,
                ),
            );
        } else {
            results.push(
                bench(
                    `body.setNextKinematicTransform()`,
                    () => {
                        for (let i = 0; i < kinematicBodies.length; i++) {
                            kinematicBodies[i].setNextKinematicTranslation(translations3D[i]);
                            kinematicBodies[i].setNextKinematicRotation(rotations3D[i]);
                        }
                    },
                    opts,
                ),
            );
        }
    } else {
        // 2D variants
        if (supportsBatchTransform) {
            results.push(
                bench(
                    `body.setTransform()`,
                    () => {
                        for (let i = 0; i < dynamicBodies.length; i++) {
                            dynamicBodies[i].setTransform(translations2D[i], rotations2D[i], true);
                        }
                    },
                    opts,
                ),
            );
        } else {
            results.push(
                bench(
                    `body.setTransform()`,
                    () => {
                        for (let i = 0; i < dynamicBodies.length; i++) {
                            dynamicBodies[i].setTranslation(translations2D[i], false);
                            dynamicBodies[i].setRotation(rotations2D[i], true);
                        }
                    },
                    opts,
                ),
            );
        }

        if (supportsBatchTransform) {
            results.push(
                bench(
                    `body.setNextKinematicTransform()`,
                    () => {
                        for (let i = 0; i < kinematicBodies.length; i++) {
                            kinematicBodies[i].setNextKinematicTransform(
                                translations2D[i],
                                rotations2D[i],
                            );
                        }
                    },
                    opts,
                ),
            );
        } else {
            results.push(
                bench(
                    `body.setNextKinematicTransform()`,
                    () => {
                        for (let i = 0; i < kinematicBodies.length; i++) {
                            kinematicBodies[i].setNextKinematicTranslation(translations2D[i]);
                            kinematicBodies[i].setNextKinematicRotation(rotations2D[i]);
                        }
                    },
                    opts,
                ),
            );
        }
    }

    world.free();

    return results;
}
