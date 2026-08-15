/**
 * A small platformer, written the way the design says gameplay should be written:
 * systems are functions over queries, they read a latched input frame rather than
 * the browser, and structural changes go through the command buffer.
 *
 * It exists so the harness can assert gameplay behaviour ("the player lands on the
 * platform") rather than engine internals.
 */

import type {Stage, System, SimContext} from "./sim.ts";
import {Actions} from "./input.ts";
import {codegenCursor, eachEntity, Query} from "./query.ts";
import {component, tag} from "./schema.ts";
import {World} from "./world.ts";

export const Position = component("Position", {px: "f32", py: "f32", pz: "f32"});
export const Velocity = component("Velocity", {vx: "f32", vy: "f32", vz: "f32"});
export const Character = component("Character", {grounded: "u32", jumps: "u32"});
export const Pickup = component("Pickup", {value: "f32"});
export const Player = tag("Player");

export const GRAVITY = -20;
export const MOVE_SPEED = 6;
export const JUMP_SPEED = 8;
export const GROUND_Y = 0;
export const PICKUP_RADIUS = 0.5;

export interface Scene {
    world: World;
    stages: Stage[];
    player: number;
    pickups: number[];
    /** Written by the pickup system so tests can assert on it. */
    collected: {count: number};
}

export function buildScene(): Scene {
    const world = new World({capacity: 1024, pages: 32, shared: true});

    const player = world.spawn([Position, Velocity, Character, Player]);
    const pickups: number[] = [];
    for (let i = 0; i < 3; i++) pickups.push(world.spawn([Position, Pickup]));

    const collected = {count: 0};

    // place them
    const put = (e: number, x: number, y: number, z: number) => {
        const loc = world.locate(e)!;
        loc.arch.columns.get(`${Position.id}:px`)![loc.row] = x;
        loc.arch.columns.get(`${Position.id}:py`)![loc.row] = y;
        loc.arch.columns.get(`${Position.id}:pz`)![loc.row] = z;
    };
    put(player, 0, 5, 0);
    put(pickups[0], 2, 0, 0);
    put(pickups[1], 4, 0, 0);
    put(pickups[2], 60, 0, 0);

    /* ---- queries, resolved once ---- */

    const qMove = new Query(world, [Position, Velocity, Character, Player]);
    const qBody = new Query(world, [Position, Velocity]);
    const qPickup = new Query(world, [Position, Pickup]);

    const moveCursor = new (codegenCursor(qMove.fields))();
    const bodyCursor = new (codegenCursor(qBody.fields))();

    /* ---- systems ---- */

    const applyInput: System = {
        name: "applyInput",
        run(ctx: SimContext) {
            const inp = ctx.input.frame(0, ctx.tick);
            let ax = 0;
            if (inp.held(Actions.moveRight)) ax += 1;
            if (inp.held(Actions.moveLeft)) ax -= 1;
            ax += inp.moveX;
            const jumped = inp.pressed(Actions.jump);

            eachEntity(qMove, moveCursor, (e) => {
                e.vx = ax * MOVE_SPEED;
                if (jumped && e.grounded === 1) {
                    e.vy = JUMP_SPEED;
                    e.grounded = 0;
                    e.jumps = e.jumps + 1;
                }
            });
        },
    };

    const gravity: System = {
        name: "gravity",
        run(ctx: SimContext) {
            eachEntity(qMove, moveCursor, (e) => {
                if (e.grounded === 0) e.vy = e.vy + GRAVITY * ctx.dt;
            });
        },
    };

    const integrate: System = {
        name: "integrate",
        run(ctx: SimContext) {
            eachEntity(qBody, bodyCursor, (e) => {
                e.px = e.px + e.vx * ctx.dt;
                e.py = e.py + e.vy * ctx.dt;
                e.pz = e.pz + e.vz * ctx.dt;
            });
        },
    };

    const collideGround: System = {
        name: "collideGround",
        run() {
            eachEntity(qMove, moveCursor, (e) => {
                if (e.py <= GROUND_Y) {
                    e.py = GROUND_Y;
                    e.vy = 0;
                    e.grounded = 1;
                }
            });
        },
    };

    // Reads the player position once, then scans pickups. Despawns are deferred
    // to the command buffer rather than mutating during iteration.
    const collectPickups: System = {
        name: "collectPickups",
        run(ctx: SimContext) {
            const loc = world.locate(player);
            if (!loc) return;
            const px = loc.arch.columns.get(`${Position.id}:px`)![loc.row];
            const py = loc.arch.columns.get(`${Position.id}:py`)![loc.row];

            for (let a = 0; a < qPickup.archetypes.length; a++) {
                const arch = qPickup.archetypes[a];
                const cx = arch.columns.get(`${Position.id}:px`)!;
                const cy = arch.columns.get(`${Position.id}:py`)!;
                for (let i = arch.count - 1; i >= 0; i--) {
                    const dx = cx[i] - px;
                    const dy = cy[i] - py;
                    if (dx * dx + dy * dy <= PICKUP_RADIUS * PICKUP_RADIUS) {
                        const entity = arch.entities[i];
                        ctx.world.defer(() => {
                            ctx.world.despawn(entity);
                            collected.count++;
                            qPickup.refresh();
                        });
                    }
                }
            }
        },
    };

    const stages: Stage[] = [
        {name: "input", systems: [applyInput]},
        {name: "fixedUpdate", systems: [gravity, integrate, collideGround]},
        {name: "postUpdate", systems: [collectPickups]},
    ];

    return {world, stages, player, pickups, collected};
}
