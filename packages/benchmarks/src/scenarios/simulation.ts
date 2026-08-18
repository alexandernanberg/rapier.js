import {bench, summary} from "mitata";
import {createPyramidWorld} from "../worlds/pyramid.js";

export function benchSimulation(RAPIER: any, is3D: boolean, quick: boolean): void {
    const bodyCount = quick ? 1000 : 3000;

    // A settled pyramid puts essentially every island to sleep — 1 of 3001 bodies
    // stays awake — and stepping it costs ~1000x less than stepping the same scene
    // while it is active. Both are worth tracking, but only as separate numbers:
    // the active one is the simulation cost, the sleeping one is the fast path
    // that skips it.
    const activeWorld = createPyramidWorld(RAPIER, is3D, bodyCount, {canSleep: false});
    for (let i = 0; i < 60; i++) activeWorld.step();

    const sleepingWorld = createPyramidWorld(RAPIER, is3D, bodyCount);
    for (let i = 0; i < 60; i++) sleepingWorld.step();

    summary(() => {
        bench(`world.step() [${bodyCount} bodies, active]`, () => {
            activeWorld.step();
        });

        bench(`world.step() [${bodyCount} bodies, sleeping]`, () => {
            sleepingWorld.step();
        });
    });
}
