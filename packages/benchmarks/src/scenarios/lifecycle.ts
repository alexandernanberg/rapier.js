import {bench, summary} from "mitata";

export function benchLifecycle(RAPIER: any, is3D: boolean, quick: boolean): void {
    const gravity = is3D ? {x: 0, y: -9.81, z: 0} : {x: 0, y: -9.81};
    const createCount = quick ? 1000 : 5000;
    const churnCount = quick ? 200 : 500;
    // Spawning into an empty world is not the case that stresses the sets; the
    // churn benchmark runs against a world that already holds bodies.
    const residentCount = 1000;

    summary(() => {
        bench(`create ${createCount} bodies+colliders`, () => {
            const world = new RAPIER.World(gravity);
            for (let i = 0; i < createCount; i++) {
                const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
                world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
            }
            world.free();
        });
    });

    // Create world for spawn/despawn test
    const world = new RAPIER.World(gravity);

    // Pre-create some bodies to have a realistic scenario
    for (let i = 0; i < residentCount; i++) {
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        if (is3D) {
            world.createCollider(RAPIER.ColliderDesc.cuboid(1, 1, 1), body);
        } else {
            world.createCollider(RAPIER.ColliderDesc.cuboid(1, 1), body);
        }
    }

    summary(() => {
        bench(`spawn+despawn ${churnCount} bodies (${residentCount} resident)`, () => {
            const bodies = [];

            for (let i = 0; i < churnCount; i++) {
                const bodyDesc = RAPIER.RigidBodyDesc.dynamic();
                if (is3D) {
                    bodyDesc.setTranslation(
                        Math.random() * 10 - 5,
                        Math.random() * 10,
                        Math.random() * 10 - 5,
                    );
                } else {
                    bodyDesc.setTranslation(Math.random() * 10 - 5, Math.random() * 10);
                }
                const body = world.createRigidBody(bodyDesc);
                world.createCollider(RAPIER.ColliderDesc.ball(0.3), body);
                bodies.push(body);
            }

            for (const body of bodies) {
                world.removeRigidBody(body);
            }
        });
    });
}
