import {getAllEntities} from "bitecs";
import {expect, it} from "vitest";
import {createSimWorld, hashWorld, idOf, OrderType, Recorder, run} from "../index";

/**
 * The README's usage example, executed.
 *
 * Documentation that does not run rots, and this one doubles as a check that
 * the public surface in `index.ts` is actually usable from outside — the export
 * list is easy to leave incomplete.
 */
it("runs the README example", () => {
    const world = createSimWorld({
        seed: 0xc0ffee,
        mapWidth: 64,
        mapHeight: 64,
        obstacles: [{x: 30, y: 0, w: 2, h: 50, weight: 0}],
    });
    const recorder = new Recorder();

    recorder.issue(world, 0, OrderType.Spawn, 1, 10, 10);
    run(world, world.config.orderDelay + 1);

    const [militia] = getAllEntities(world);
    expect(militia).toBeDefined();

    recorder.issue(world, 0, OrderType.Move, militia, 50, 12);
    run(world, 800);

    const {Position} = world.stores;
    expect(Position.x[idOf(world, militia)]).toBeCloseTo(50, 2);
    expect(Position.y[idOf(world, militia)]).toBeCloseTo(12, 2);
    expect(hashWorld(world)).toMatch(/^[0-9a-f]{16}$/);
    expect(recorder.orders).toHaveLength(2);
});
