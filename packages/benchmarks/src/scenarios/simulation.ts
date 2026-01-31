import type { BenchResult } from "../runner.js";
import { bench, statsFrom } from "../runner.js";
import { createPyramidWorld } from "../worlds/pyramid.js";

export async function benchSimulation(
  RAPIER: any,
  is3D: boolean,
  quick: boolean
): Promise<BenchResult[]> {
  const results: BenchResult[] = [];
  const bodyCount = quick ? 500 : 3000;

  // Create pyramid world
  const world = createPyramidWorld(RAPIER, is3D, bodyCount);

  // Warmup - let simulation settle
  for (let i = 0; i < 60; i++) world.step();

  // Measure step time using performance.now()
  const steps = quick ? 100 : 300;
  const stepTimes: number[] = [];

  for (let i = 0; i < steps; i++) {
    const start = performance.now();
    world.step();
    stepTimes.push(performance.now() - start);
  }

  results.push(statsFrom(`world.step() [${bodyCount} bodies]`, stepTimes));

  // Try built-in profiler if available (our fork only)
  const hasProfiler = typeof world.timingCollisionDetection === "function";

  if (hasProfiler) {
    world.profilerEnabled = true;

    // Collect built-in timing data if profiler works
    const timings = {
      collisionDetection: [] as number[],
      broad: [] as number[],
      narrow: [] as number[],
      solver: [] as number[],
    };

    for (let i = 0; i < steps; i++) {
      world.step();
      timings.collisionDetection.push(world.timingCollisionDetection());
      timings.broad.push(world.timingBroadPhase());
      timings.narrow.push(world.timingNarrowPhase());
      timings.solver.push(world.timingSolver());
    }

    // Only add sub-timings if the profiler is actually working
    const hasProfilerData = timings.collisionDetection.some((t) => t > 0);
    if (hasProfilerData) {
      results.push(
        statsFrom("├─ collisionDetection", timings.collisionDetection)
      );
      results.push(statsFrom("│  ├─ broadPhase", timings.broad));
      results.push(statsFrom("│  └─ narrowPhase", timings.narrow));
      results.push(statsFrom("└─ solver", timings.solver));
    }
  }

  world.free();

  return results;
}
