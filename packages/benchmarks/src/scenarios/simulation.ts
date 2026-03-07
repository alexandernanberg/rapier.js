import {bench, summary} from "mitata";
import {createPyramidWorld} from "../worlds/pyramid.js";

export function benchSimulation(RAPIER: any, is3D: boolean, quick: boolean): void {
    const bodyCount = quick ? 500 : 3000;

    const world = createPyramidWorld(RAPIER, is3D, bodyCount);

    // Let simulation settle
    for (let i = 0; i < 60; i++) world.step();

    summary(() => {
        bench(`world.step() [${bodyCount} bodies]`, () => {
            world.step();
        });
    });
}
