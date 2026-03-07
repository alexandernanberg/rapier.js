import {bench, summary} from "mitata";

export function benchSetters(RAPIER: any, is3D: boolean, _quick: boolean): void {
    const bodyCount = 1000;

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
            bench(`body.setTransform()`, () => {
                for (let i = 0; i < dynamicBodies.length; i++) {
                    dynamicBodies[i].setTransform(translations[i], rotations[i], true);
                }
            });
        } else {
            bench(`body.setTransform()`, () => {
                for (let i = 0; i < dynamicBodies.length; i++) {
                    dynamicBodies[i].setTranslation(translations[i], false);
                    dynamicBodies[i].setRotation(rotations[i], true);
                }
            });
        }

        if (supportsBatchTransform) {
            bench(`body.setNextKinematicTransform()`, () => {
                for (let i = 0; i < kinematicBodies.length; i++) {
                    kinematicBodies[i].setNextKinematicTransform(translations[i], rotations[i]);
                }
            });
        } else {
            bench(`body.setNextKinematicTransform()`, () => {
                for (let i = 0; i < kinematicBodies.length; i++) {
                    kinematicBodies[i].setNextKinematicTranslation(translations[i]);
                    kinematicBodies[i].setNextKinematicRotation(rotations[i]);
                }
            });
        }
    });
}
