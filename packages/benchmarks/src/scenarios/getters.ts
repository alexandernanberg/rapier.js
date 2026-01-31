import type { BenchResult } from "../runner.js";
import { bench } from "../runner.js";

export async function benchGetters(
  RAPIER: any,
  is3D: boolean,
  quick: boolean
): Promise<BenchResult[]> {
  const results: BenchResult[] = [];
  const bodyCount = 1000;
  const opts = quick
    ? { warmup: 50, iterations: 200 }
    : { warmup: 100, iterations: 1000 };

  const gravity = is3D ? { x: 0, y: -9.81, z: 0 } : { x: 0, y: -9.81 };
  const world = new RAPIER.World(gravity);

  const bodies: any[] = [];
  const colliders: any[] = [];

  for (let i = 0; i < bodyCount; i++) {
    const bodyDesc = RAPIER.RigidBodyDesc.dynamic();
    if (is3D) {
      bodyDesc.setTranslation(
        Math.random() * 100 - 50,
        Math.random() * 100,
        Math.random() * 100 - 50
      );
    } else {
      bodyDesc.setTranslation(Math.random() * 100 - 50, Math.random() * 100);
    }
    const body = world.createRigidBody(bodyDesc);
    const collider = world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);
    bodies.push(body);
    colliders.push(collider);
  }

  // Step once to initialize velocities
  world.step();

  // RigidBody getters - allocating pattern
  results.push(
    bench(
      `body.translation() [alloc]`,
      () => {
        for (const b of bodies) b.translation();
      },
      opts
    )
  );

  // Check if zero-alloc pattern is supported (our fork only)
  const sampleBody = bodies[0];
  const supportsTargetParam = (() => {
    try {
      const target = is3D ? { x: 0, y: 0, z: 0 } : { x: 0, y: 0 };
      sampleBody.translation(target);
      // If it returns the target object mutated, it's supported
      return target.x !== undefined;
    } catch {
      return false;
    }
  })();

  // RigidBody getters - zero-alloc pattern with target (fork only)
  if (supportsTargetParam) {
    const translationTarget = is3D ? { x: 0, y: 0, z: 0 } : { x: 0, y: 0 };
    results.push(
      bench(
        `body.translation() [reuse]`,
        () => {
          for (const b of bodies) b.translation(translationTarget);
        },
        opts
      )
    );
  }

  // Rotation - allocating
  results.push(
    bench(
      `body.rotation() [alloc]`,
      () => {
        for (const b of bodies) b.rotation();
      },
      opts
    )
  );

  // Rotation - zero-alloc (fork only, 3D only)
  if (supportsTargetParam && is3D) {
    const rotationTarget = { x: 0, y: 0, z: 0, w: 1 };
    results.push(
      bench(
        `body.rotation() [reuse]`,
        () => {
          for (const b of bodies) b.rotation(rotationTarget);
        },
        opts
      )
    );
  }

  // Linear velocity - allocating
  results.push(
    bench(
      `body.linvel() [alloc]`,
      () => {
        for (const b of bodies) b.linvel();
      },
      opts
    )
  );

  // Linear velocity - zero-alloc (fork only)
  if (supportsTargetParam) {
    const linvelTarget = is3D ? { x: 0, y: 0, z: 0 } : { x: 0, y: 0 };
    results.push(
      bench(
        `body.linvel() [reuse]`,
        () => {
          for (const b of bodies) b.linvel(linvelTarget);
        },
        opts
      )
    );
  }

  // Collider getters
  results.push(
    bench(
      `collider.translation() [alloc]`,
      () => {
        for (const c of colliders) c.translation();
      },
      opts
    )
  );

  // Collider translation - zero-alloc (fork only)
  if (supportsTargetParam) {
    const colliderTransTarget = is3D ? { x: 0, y: 0, z: 0 } : { x: 0, y: 0 };
    results.push(
      bench(
        `collider.translation() [reuse]`,
        () => {
          for (const c of colliders) c.translation(colliderTransTarget);
        },
        opts
      )
    );
  }

  world.free();

  return results;
}
