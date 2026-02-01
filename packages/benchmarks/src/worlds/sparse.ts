import type {World} from "@alexandernanberg/rapier3d";

export function createSparseWorld(RAPIER: any, is3D: boolean, bodyCount: number): World {
    const gravity = is3D ? {x: 0, y: -9.81, z: 0} : {x: 0, y: -9.81};
    const world = new RAPIER.World(gravity);

    // Create ground
    const groundBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    if (is3D) {
        world.createCollider(RAPIER.ColliderDesc.cuboid(100, 0.1, 100), groundBody);
    } else {
        world.createCollider(RAPIER.ColliderDesc.cuboid(100, 0.1), groundBody);
    }

    // Spread bodies evenly across a large area
    const spread = 80;
    const gridSize = Math.ceil(Math.sqrt(bodyCount));

    for (let i = 0; i < bodyCount; i++) {
        const gridX = i % gridSize;
        const gridY = Math.floor(i / gridSize);

        const x = (gridX / gridSize - 0.5) * spread;
        const y = 1 + Math.random() * 5;

        if (is3D) {
            const z = (gridY / gridSize - 0.5) * spread;
            const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
            const body = world.createRigidBody(bodyDesc);
            world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        } else {
            const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x + gridY * 0.1, y);
            const body = world.createRigidBody(bodyDesc);
            world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        }
    }

    return world;
}
