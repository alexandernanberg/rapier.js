/**
 * Spike 04 — the headless harness.
 *
 * Two kinds of check here, and the distinction matters.
 *
 *   Gameplay assertions are what an agent writes when it implements a feature:
 *   drive input, step ticks, assert on world state. No canvas, no human.
 *
 *   Property checks are what the harness owes the agent in return — determinism,
 *   replay fidelity, snapshot/restore. If these fail, every gameplay assertion
 *   above them is worthless, so they run first.
 */

import {
    buildScene,
    Character,
    GRAVITY,
    GROUND_Y,
    JUMP_SPEED,
    Position,
    Velocity,
} from "../src/demo-scene.ts";
import {Actions} from "../src/input.ts";
import {diffSnapshots, FIXED_DT, Recorder, replay, Sim, type Recording} from "../src/sim.ts";

/* ------------------------------------------------------------ tiny runner */

let passed = 0;
let failed = 0;
const failures: string[] = [];

function check(name: string, fn: () => void) {
    try {
        fn();
        passed++;
        console.log(`  \x1b[32m✓\x1b[0m ${name}`);
    } catch (err) {
        failed++;
        const msg = err instanceof Error ? err.message : String(err);
        failures.push(`${name}: ${msg}`);
        console.log(`  \x1b[31m✗\x1b[0m ${name}\n      ${msg}`);
    }
}

function eq(actual: unknown, expected: unknown, what = "value") {
    if (actual !== expected) {
        throw new Error(`${what}: expected ${String(expected)}, got ${String(actual)}`);
    }
}

function close(actual: number, expected: number, tol = 1e-4, what = "value") {
    if (Math.abs(actual - expected) > tol) {
        throw new Error(`${what}: expected ~${expected} (±${tol}), got ${actual}`);
    }
}

function ok(cond: boolean, what: string) {
    if (!cond) throw new Error(what);
}

function section(title: string) {
    console.log(`\n  \x1b[1m${title}\x1b[0m\n  ${"-".repeat(66)}`);
}

/* ------------------------------------------------------------- helpers */

function newSim(seed = 12345): Sim & {scene: ReturnType<typeof buildScene>} {
    const scene = buildScene();
    const sim = new Sim({world: scene.world, stages: scene.stages, seed}) as Sim & {
        scene: ReturnType<typeof buildScene>;
    };
    sim.scene = scene;
    return sim;
}

/* ================================================================ */

console.log("\n\x1b[1m═══ Spike 04 · headless harness ═══\x1b[0m");
console.log(`  node ${process.version} · fixed timestep, seeded rng, input as data`);

section("Properties the harness must guarantee");

check("same inputs twice produce byte-identical worlds", () => {
    const run = () => {
        const s = newSim();
        s.input.hold(0, Actions.moveRight);
        s.step(20);
        s.input.press(0, Actions.jump);
        s.step(40);
        return s.snapshot();
    };
    const d = diffSnapshots(run(), run());
    ok(d === -1, `worlds diverged at byte ${d}`);
});

check("a recording replays to a byte-identical world", () => {
    // record
    const a = newSim();
    const rec = new Recorder(a, 12345);
    const TICKS = 90;
    for (let t = 0; t < TICKS; t++) {
        if (t === 5) a.input.hold(0, Actions.moveRight);
        if (t === 30) a.input.tap(0, Actions.jump);
        if (t === 55) a.input.release(0, Actions.moveRight);
        a.step(1);
        rec.capture(t);
    }
    const recording: Recording = rec.finish(TICKS);
    const original = a.snapshot();

    // replay into a fresh sim, feeding only the recorded frames
    const b = newSim();
    replay(b, recording);
    const d = diffSnapshots(original, b.snapshot());
    ok(d === -1, `replay diverged at byte ${d}`);
});

check("recording is 16 bytes per player per tick", () => {
    const a = newSim();
    const rec = new Recorder(a, 1);
    for (let t = 0; t < 60; t++) {
        a.step(1);
        rec.capture(t);
    }
    const r = rec.finish(60);
    eq(r.frames.byteLength, 60 * 1 * 16, "recording size");
});

check("snapshot then restore rewinds the world exactly", () => {
    const s = newSim();
    s.input.hold(0, Actions.moveRight);
    s.step(30);
    const mark = s.snapshot();
    const yAtMark = s.get(s.scene.player, Position, "py");

    s.input.tap(0, Actions.jump);
    s.step(20);
    ok(s.get(s.scene.player, Position, "py") !== yAtMark, "world did not advance");

    s.restore(mark);
    eq(s.get(s.scene.player, Position, "py"), yAtMark, "restored py");
    eq(s.tick, mark.tick, "restored tick");
    ok(diffSnapshots(s.snapshot(), mark) === -1, "restored world differs");
});

check("a missing component produces an error that names the fix", () => {
    const s = newSim();
    let msg = "";
    try {
        s.get(s.scene.pickups[0], Velocity, "vx");
    } catch (e) {
        msg = e instanceof Error ? e.message : String(e);
    }
    ok(msg.includes("Velocity.vx"), `message should name the component: "${msg}"`);
    ok(msg.includes("Position"), `message should list what the entity does have: "${msg}"`);
});

section("Gameplay assertions — what an agent writes");

check("the player falls and lands on the ground", () => {
    const s = newSim();
    close(s.get(s.scene.player, Position, "py"), 5, 1e-6, "start height");
    s.step(120);
    close(s.get(s.scene.player, Position, "py"), GROUND_Y, 1e-6, "resting height");
    eq(s.get(s.scene.player, Character, "grounded"), 1, "grounded flag");
    close(s.get(s.scene.player, Velocity, "vy"), 0, 1e-6, "resting vy");
});

check("jump only works while grounded", () => {
    const s = newSim();
    s.input.tap(0, Actions.jump); // airborne at tick 0
    s.step(1);
    eq(s.get(s.scene.player, Character, "jumps"), 0, "jumps while airborne");

    s.step(120); // land
    eq(s.get(s.scene.player, Character, "grounded"), 1, "grounded after falling");
    s.input.tap(0, Actions.jump);
    s.step(1);
    eq(s.get(s.scene.player, Character, "jumps"), 1, "jumps after landing");
    // Not JUMP_SPEED: gravity runs after the impulse in the same tick, so the
    // observable post-tick velocity is JUMP_SPEED + g*dt. That is correct
    // semi-implicit Euler, and it is exactly the kind of ordering consequence
    // that silently surprises whoever tunes the jump height.
    close(
        s.get(s.scene.player, Velocity, "vy"),
        JUMP_SPEED + GRAVITY * FIXED_DT,
        1e-5,
        "launch velocity after in-tick gravity",
    );
});

check("a tap held for less than a tick still registers", () => {
    const s = newSim();
    s.step(120); // land first
    // down and up before the next latch — the sticky bit must survive
    s.input.tap(0, Actions.jump);
    s.step(1);
    eq(s.get(s.scene.player, Character, "jumps"), 1, "jumps from a sub-tick tap");
});

check("a held action produces no second rising edge", () => {
    const s = newSim();
    s.step(120);
    s.input.press(0, Actions.jump); // held, never released
    s.step(1);
    eq(s.get(s.scene.player, Character, "jumps"), 1, "first jump");
    s.step(120); // land again, jump still held
    eq(s.get(s.scene.player, Character, "grounded"), 1, "grounded again");
    s.step(1);
    eq(s.get(s.scene.player, Character, "jumps"), 1, "no auto-rejump while held");
});

check("holding right moves the player right at move speed", () => {
    const s = newSim();
    s.step(120);
    const x0 = s.get(s.scene.player, Position, "px");
    s.input.hold(0, Actions.moveRight);
    s.step(60);
    const dx = s.get(s.scene.player, Position, "px") - x0;
    close(dx, 6, 0.2, "distance travelled in one second");
});

check("analog move is quantized identically on both sides", () => {
    const s = newSim();
    s.step(120);
    s.input.setMove(0, 0.5, 0);
    s.step(1);
    // 0.5 -> round(0.5*127) = 64 -> 64/127, not 0.5
    close(s.get(s.scene.player, Velocity, "vx"), (64 / 127) * 6, 1e-5, "quantized vx");
});

check("walking over pickups collects them, via the command buffer", () => {
    const s = newSim();
    s.step(120);
    eq(s.scene.collected.count, 0, "collected before moving");
    s.input.hold(0, Actions.moveRight);
    s.step(90); // travels ~9 units, past pickups at x=2 and x=4 but not x=60
    eq(s.scene.collected.count, 2, "pickups collected");
});

check("despawn during iteration does not corrupt the world", () => {
    const s = newSim();
    s.step(120);
    s.input.hold(0, Actions.moveRight);
    s.step(90);
    // the far pickup must still be intact and addressable
    close(s.get(s.scene.pickups[2], Position, "px"), 60, 1e-6, "surviving pickup x");
});

/* ---------------------------------------------------------------- report */

console.log(`\n  ${"-".repeat(68)}`);
if (failed === 0) {
    console.log(`  \x1b[32m${passed} passed\x1b[0m, 0 failed\n`);
} else {
    console.log(`  \x1b[32m${passed} passed\x1b[0m, \x1b[31m${failed} failed\x1b[0m`);
    for (const f of failures) console.log(`    - ${f}`);
    console.log();
}
process.exit(failed === 0 ? 0 : 1);
