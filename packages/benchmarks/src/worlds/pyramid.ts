import type {World} from "@alexandernanberg/rapier3d";

export function createPyramidWorld(RAPIER: any, is3D: boolean, bodyCount: number): World {
    const gravity = is3D ? {x: 0, y: -9.81, z: 0} : {x: 0, y: -9.81};
    const world = new RAPIER.World(gravity);

    // Create ground
    const groundBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    if (is3D) {
        world.createCollider(RAPIER.ColliderDesc.cuboid(50, 0.1, 50), groundBody);
    } else {
        world.createCollider(RAPIER.ColliderDesc.cuboid(50, 0.1), groundBody);
    }

    // Create pyramid of boxes
    const size = 0.5;
    const spacing = size * 2.1;
    let created = 0;
    let row = 0;

    while (created < bodyCount) {
        const rowWidth = Math.ceil(Math.sqrt(bodyCount - created));
        const y = row * spacing + size + 0.2;

        for (let i = 0; i < rowWidth && created < bodyCount; i++) {
            const x = (i - rowWidth / 2) * spacing;

            if (is3D) {
                for (let j = 0; j < rowWidth && created < bodyCount; j++) {
                    const z = (j - rowWidth / 2) * spacing;
                    const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
                    const body = world.createRigidBody(bodyDesc);
                    world.createCollider(RAPIER.ColliderDesc.cuboid(size, size, size), body);
                    created++;
                }
            } else {
                const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y);
                const body = world.createRigidBody(bodyDesc);
                world.createCollider(RAPIER.ColliderDesc.cuboid(size, size), body);
                created++;
            }
        }
        row++;
    }

    return world;
}
