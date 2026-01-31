import type { BenchResult } from "../runner.js";
import { bench } from "../runner.js";

export async function benchLifecycle(
  RAPIER: any,
  is3D: boolean,
  quick: boolean
): Promise<BenchResult[]> {
  const results: BenchResult[] = [];
  const opts = quick
    ? { warmup: 5, iterations: 20 }
    : { warmup: 10, iterations: 100 };

  const gravity = is3D ? { x: 0, y: -9.81, z: 0 } : { x: 0, y: -9.81 };

  // Batch creation of bodies with colliders
  results.push(
    bench(
      "create 1000 bodies+colliders",
      () => {
        const world = new RAPIER.World(gravity);
        for (let i = 0; i < 1000; i++) {
          const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
          world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
        }
        world.free();
      },
      opts
    )
  );

  // Create world for spawn/despawn test
  const world = new RAPIER.World(gravity);

  // Pre-create some bodies to have a realistic scenario
  const staticBodies = [];
  for (let i = 0; i < 100; i++) {
    const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    if (is3D) {
      world.createCollider(
        RAPIER.ColliderDesc.cuboid(1, 1, 1),
        body
      );
    } else {
      world.createCollider(RAPIER.ColliderDesc.cuboid(1, 1), body);
    }
    staticBodies.push(body);
  }

  // Spawn/despawn churn (fountain pattern)
  results.push(
    bench(
      "spawn+despawn 100 bodies",
      () => {
        const bodies = [];

        // Spawn 100 bodies
        for (let i = 0; i < 100; i++) {
          const bodyDesc = RAPIER.RigidBodyDesc.dynamic();
          if (is3D) {
            bodyDesc.setTranslation(
              Math.random() * 10 - 5,
              Math.random() * 10,
              Math.random() * 10 - 5
            );
          } else {
            bodyDesc.setTranslation(
              Math.random() * 10 - 5,
              Math.random() * 10
            );
          }
          const body = world.createRigidBody(bodyDesc);
          world.createCollider(RAPIER.ColliderDesc.ball(0.3), body);
          bodies.push(body);
        }

        // Despawn all
        for (const body of bodies) {
          world.removeRigidBody(body);
        }
      },
      opts
    )
  );

  world.free();

  return results;
}
