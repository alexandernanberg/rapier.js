import {getAllEntities, hasComponent} from "bitecs";
import {describe, expect, it} from "vitest";
import {Recorder} from "../replay";
import {CommandBuffer, CommandKind} from "../sim/command_buffer";
import {UnitKindId} from "../sim/components";
import {OrderType} from "../sim/orders";
import {movementSystem} from "../sim/systems";
import {step} from "../sim/tick";
import {createSimWorld, flushCommands, idOf, isAlive, spawnUnit} from "../sim/world";

describe("CommandBuffer", () => {
    it("hands over a batch and starts a fresh one", () => {
        const buffer = new CommandBuffer();
        buffer.despawn(1);
        buffer.despawn(2);
        expect(buffer.size).toBe(2);

        const drained = buffer.drain();
        expect(drained).toHaveLength(2);
        expect(drained[0]).toMatchObject({kind: CommandKind.Despawn, a: 1});
        expect(buffer.size).toBe(0);

        buffer.despawn(3);
        expect(buffer.drain()).toHaveLength(1);
    });

    it("preserves the order commands were queued in", () => {
        const buffer = new CommandBuffer();
        buffer.spawn(0, 0, 1, 1);
        buffer.damage(7, 5);
        buffer.clearMoveTarget(7);

        expect(buffer.drain().map((c) => c.kind)).toEqual([
            CommandKind.Spawn,
            CommandKind.Damage,
            CommandKind.ClearMoveTarget,
        ]);
    });
});

describe("deferred structural changes", () => {
    it("does not remove a component while the query that saw it is live", () => {
        const world = createSimWorld({seed: 1});
        const eid = spawnUnit(world, UnitKindId.Militia, 0, 0, 0);
        const id = idOf(world, eid);

        world.cmd.setMoveTarget(eid, world.stores.Position.x[id], world.stores.Position.y[id]);
        flushCommands(world);
        expect(hasComponent(world, eid, world.stores.MoveTarget)).toBe(true);

        // The unit is already at its target, so the system queues an arrival.
        movementSystem(world);
        expect(world.cmd.size).toBe(1);
        expect(hasComponent(world, eid, world.stores.MoveTarget)).toBe(true);

        flushCommands(world);
        expect(hasComponent(world, eid, world.stores.MoveTarget)).toBe(false);
    });

    it("delays an order by exactly orderDelay ticks", () => {
        const world = createSimWorld({seed: 1, orderDelay: 4});
        const recorder = new Recorder();

        recorder.issue(world, 0, OrderType.Spawn, UnitKindId.Archer, 5, 5);
        expect(recorder.orders[0].tick).toBe(4);

        // Ticks 0-3 must not see it: the order executes during the step that
        // advances tick 4 to 5.
        for (let i = 0; i < 4; i++) {
            step(world);
            expect(getAllEntities(world)).toHaveLength(0);
        }

        step(world);
        const entities = getAllEntities(world);
        expect(entities).toHaveLength(1);
        expect(world.stores.UnitKind.kind[idOf(world, entities[0])]).toBe(UnitKindId.Archer);
    });

    it("ignores commands aimed at an entity that has already died", () => {
        const world = createSimWorld({seed: 1});
        const eid = spawnUnit(world, UnitKindId.Villager, 0, 0, 0);

        world.cmd.despawn(eid);
        world.cmd.setMoveTarget(eid, 10, 10);
        world.cmd.damage(eid, 5);

        expect(() => flushCommands(world)).not.toThrow();
        expect(isAlive(world, eid)).toBe(false);
    });
});
