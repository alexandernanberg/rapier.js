import {bench, summary} from "mitata";

export function benchSetters(RAPIER: any, is3D: boolean, quick: boolean): void {
    const bodyCount = quick ? 1000 : 5000;

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

    const translations = is3D ? translations3D : translations2D;
    const rotations = is3D ? rotations3D : rotations2D;

    summary(() => {
        if (supportsBatchTransform) {
            bench(`body.setTransform() x${bodyCount}`, () => {
                for (let i = 0; i < dynamicBodies.length; i++) {
                    dynamicBodies[i].setTransform(translations[i], rotations[i], true);
                }
            });
        } else {
            bench(`body.setTransform() x${bodyCount}`, () => {
                for (let i = 0; i < dynamicBodies.length; i++) {
                    dynamicBodies[i].setTranslation(translations[i], false);
                    dynamicBodies[i].setRotation(rotations[i], true);
                }
            });
        }

        if (supportsBatchTransform) {
            bench(`body.setNextKinematicTransform() x${bodyCount}`, () => {
                for (let i = 0; i < kinematicBodies.length; i++) {
                    kinematicBodies[i].setNextKinematicTransform(translations[i], rotations[i]);
                }
            });
        } else {
            bench(`body.setNextKinematicTransform() x${bodyCount}`, () => {
                for (let i = 0; i < kinematicBodies.length; i++) {
                    kinematicBodies[i].setNextKinematicTranslation(translations[i]);
                    kinematicBodies[i].setNextKinematicRotation(rotations[i]);
                }
            });
        }
    });

    // Per-frame force/velocity setters (hot path for character controllers,
    // vehicles, thrusters, projectiles, custom force fields).
    const vec = is3D ? {x: 1, y: 2, z: 3} : {x: 1, y: 2};
    const pt = is3D ? {x: 0.5, y: 0.5, z: 0.5} : {x: 0.5, y: 0.5};

    summary(() => {
        bench(`body.setLinvel() x${bodyCount}`, () => {
            for (const b of dynamicBodies) b.setLinvel(vec, true);
        });
        bench(`body.applyImpulse() x${bodyCount}`, () => {
            for (const b of dynamicBodies) b.applyImpulse(vec, true);
        });
        bench(`body.addForce() x${bodyCount}`, () => {
            for (const b of dynamicBodies) b.addForce(vec, true);
        });
        bench(`body.applyImpulseAtPoint() x${bodyCount}`, () => {
            for (const b of dynamicBodies) b.applyImpulseAtPoint(vec, pt, true);
        });
    });
}
