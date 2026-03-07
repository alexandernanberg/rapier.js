import {bench, summary} from "mitata";

export function benchLifecycle(RAPIER: any, is3D: boolean, _quick: boolean): void {
    const gravity = is3D ? {x: 0, y: -9.81, z: 0} : {x: 0, y: -9.81};

    summary(() => {
        bench("create 1000 bodies+colliders", () => {
            const world = new RAPIER.World(gravity);
            for (let i = 0; i < 1000; i++) {
                const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
                world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
            }
            world.free();
        });
    });

    // Create world for spawn/despawn test
    const world = new RAPIER.World(gravity);

    // Pre-create some bodies to have a realistic scenario
    for (let i = 0; i < 100; i++) {
        const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        if (is3D) {
            world.createCollider(RAPIER.ColliderDesc.cuboid(1, 1, 1), body);
        } else {
            world.createCollider(RAPIER.ColliderDesc.cuboid(1, 1), body);
        }
    }

    summary(() => {
        bench("spawn+despawn 100 bodies", () => {
            const bodies = [];

            for (let i = 0; i < 100; i++) {
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
