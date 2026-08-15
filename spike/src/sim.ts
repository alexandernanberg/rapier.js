/**
 * The headless simulation facade — the agent's handle on a running game.
 *
 * Deterministic, steppable, snapshottable, and driven entirely by input frames.
 * Everything a test needs to assert gameplay behaviour without a canvas, a
 * browser, or a human looking at pixels.
 */

import type {ComponentDef} from "./schema.ts";
import {InputBuffer} from "./input.ts";
import {World} from "./world.ts";

export interface Stage {
    name: string;
    systems: System[];
}

export interface System {
    name: string;
    run(ctx: SimContext): void;
}

export interface SimContext {
    world: World;
    input: InputBuffer;
    tick: number;
    dt: number;
    /** Seeded, replayable. Systems must never touch Math.random. */
    random(): number;
}

export interface Snapshot {
    bytes: Uint8Array;
    tick: number;
    rng: number;
}

export const FIXED_DT = 1 / 60;

export class Sim {
    readonly world: World;
    readonly input: InputBuffer;
    readonly stages: Stage[];
    tick = 0;
    private rng: number;
    private readonly ctx: SimContext;

    constructor(opts: {world: World; stages: Stage[]; players?: number; seed?: number}) {
        this.world = opts.world;
        this.stages = opts.stages;
        this.input = new InputBuffer(opts.players ?? 1, 256);
        this.rng = (opts.seed ?? 0x9e3779b9) >>> 0;

        const self = this;
        this.ctx = {
            world: this.world,
            input: this.input,
            tick: 0,
            dt: FIXED_DT,
            random() {
                // xorshift32 — deterministic and cheap
                let x = self.rng;
                x ^= x << 13;
                x >>>= 0;
                x ^= x >>> 17;
                x ^= x << 5;
                x >>>= 0;
                self.rng = x;
                return x / 4294967296;
            },
        };
    }

    private runTick(latch: boolean): void {
        if (latch) this.input.latch(this.tick);
        this.ctx.tick = this.tick;
        for (const stage of this.stages) {
            for (const sys of stage.systems) sys.run(this.ctx);
        }
        this.world.flush();
        this.tick++;
    }

    /** Advance `n` fixed ticks. Input is latched once per tick, never per frame. */
    step(n = 1): this {
        for (let k = 0; k < n; k++) this.runTick(true);
        return this;
    }

    /** Advance one tick using an already-injected frame — used by replay. */
    stepReplay(): this {
        this.runTick(false);
        return this;
    }

    /* ---- state access, for assertions ---- */

    get(entity: number, comp: ComponentDef, field: string): number {
        const loc = this.world.locate(entity);
        if (!loc) throw new Error(`entity ${entity} does not exist`);
        const col = loc.arch.columns.get(`${comp.id}:${field}`);
        if (!col) {
            throw new Error(
                `entity ${entity} has no ${comp.name}.${field} ` +
                    `(archetype has: ${loc.arch.comps.map((c) => c.name).join(", ")})`,
            );
        }
        return col[loc.row];
    }

    set(entity: number, comp: ComponentDef, field: string, value: number): void {
        const loc = this.world.locate(entity);
        if (!loc) throw new Error(`entity ${entity} does not exist`);
        const col = loc.arch.columns.get(`${comp.id}:${field}`);
        if (!col) throw new Error(`entity ${entity} has no ${comp.name}.${field}`);
        col[loc.row] = value;
    }

    /* ---- snapshot: the whole world is a buffer copy ---- */

    snapshot(): Snapshot {
        const used = this.world.arena.used;
        return {
            bytes: new Uint8Array(
                new Uint8Array(this.world.arena.buf as ArrayBuffer, 0, used),
            ).slice(),
            tick: this.tick,
            rng: this.rng,
        };
    }

    restore(s: Snapshot): void {
        new Uint8Array(this.world.arena.buf as ArrayBuffer, 0, s.bytes.length).set(s.bytes);
        this.tick = s.tick;
        this.rng = s.rng;
    }
}

/** Byte-compare two snapshots; returns the first differing byte or -1. */
export function diffSnapshots(a: Snapshot, b: Snapshot): number {
    if (a.tick !== b.tick) return -2;
    if (a.rng !== b.rng) return -3;
    if (a.bytes.length !== b.bytes.length) return -4;
    for (let i = 0; i < a.bytes.length; i++) if (a.bytes[i] !== b.bytes[i]) return i;
    return -1;
}

/* ------------------------------------------------------------- recording */

/** A replayable session: a seed plus every input frame that was latched. */
export interface Recording {
    seed: number;
    players: number;
    ticks: number;
    frames: Int32Array;
}

export class Recorder {
    private readonly frames: number[] = [];
    private readonly scratch = new Int32Array(InputBuffer.STRIDE);

    readonly sim: Sim;
    readonly seed: number;

    constructor(sim: Sim, seed: number) {
        this.sim = sim;
        this.seed = seed;
    }

    /** Capture the frames latched for `tick`, after stepping it. */
    capture(tick: number): void {
        for (let p = 0; p < this.sim.input.players; p++) {
            this.sim.input.readFrame(p, tick, this.scratch);
            for (let w = 0; w < InputBuffer.STRIDE; w++) this.frames.push(this.scratch[w]);
        }
    }

    finish(ticks: number): Recording {
        return {
            seed: this.seed,
            players: this.sim.input.players,
            ticks,
            frames: Int32Array.from(this.frames),
        };
    }
}

/** Feed a recording back into a fresh sim, one tick at a time. */
export function replay(sim: Sim, rec: Recording): Sim {
    const stride = InputBuffer.STRIDE;
    for (let t = 0; t < rec.ticks; t++) {
        for (let p = 0; p < rec.players; p++) {
            const base = (t * rec.players + p) * stride;
            sim.input.inject(p, t, rec.frames.subarray(base, base + stride));
        }
        // step without re-latching: the injected frame is the truth
        sim.stepReplay();
    }
    return sim;
}
