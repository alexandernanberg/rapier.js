import RAPIER, {init, scratch} from "@alexandernanberg/rapier3d/compat";
import {describe, test, expect, beforeAll} from "vitest";

beforeAll(async () => {
    await init();
});

const GRAVITY = {x: 0, y: -9.81, z: 0};

/** Deterministic xorshift32, so a failing run can be replayed from its seed. */
function rng(seed: number) {
    let s = seed >>> 0 || 1;
    return () => {
        s ^= s << 13;
        s >>>= 0;
        s ^= s >>> 17;
        s ^= s << 5;
        s >>>= 0;
        return s / 0x100000000;
    };
}

/**
 * Randomised differential check on the shared transform buffers.
 *
 * The buffers are refreshed incrementally, from a list of the bodies that can
 * have moved (see `transform_sync.test.ts`). Whether that list is complete
 * depends on how sleeping, waking, creation, removal and direct mutation
 * interleave, which is exactly the kind of state space that is easier to search
 * than to enumerate. So: drive the world randomly, and after every single step
 * assert that every buffered value still bit-matches what WASM reports.
 *
 * Seeds are fixed, so this is deterministic; a regression reproduces by running
 * the seed it failed on.
 */
describe("transform buffer fuzz", () => {
    for (const seed of [1, 2, 3]) {
        test(`buffered transforms match WASM under random churn (seed ${seed})`, () => {
            const rnd = rng(seed);
            const ri = (n: number) => Math.floor(rnd() * n);
            const rf = (a: number, b: number) => a + rnd() * (b - a);
            const vec = (s = 1) => ({x: rf(-s, s), y: rf(-s, s), z: rf(-s, s)});

            const world = new RAPIER.World(GRAVITY);
            const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
            world.createCollider(RAPIER.ColliderDesc.cuboid(60, 0.1, 60), ground);

            const mismatches: string[] = [];
            const near = (a: number, b: number) => a === b;

            /**
             * Snapshots the WASM-side value before touching the public getter:
             * both now go through the same scratch buffer, and the getter falls
             * back to WASM whenever the transform buffer is stale, which would
             * otherwise overwrite what we are comparing against.
             */
            function wasmRead(read: () => void, n: number) {
                read();
                const s = scratch();
                return [s[0], s[1], s[2], s[3]].slice(0, n);
            }

            function audit(step: number) {
                if (mismatches.length > 0) return;
                world.bodies.forEach((b) => {
                    const wt = wasmRead(() => world.bodies.raw.rbTranslation(b.handle), 3);
                    const t = b.translation();
                    if (!near(t.x, wt[0]) || !near(t.y, wt[1]) || !near(t.z, wt[2]))
                        mismatches.push(
                            `step ${step}: body translation ${JSON.stringify(t)} vs wasm [${wt.join(", ")}]`,
                        );

                    const wv = wasmRead(() => world.bodies.raw.rbLinvel(b.handle), 3);
                    const v = b.linvel();
                    if (!near(v.x, wv[0]) || !near(v.y, wv[1]) || !near(v.z, wv[2]))
                        mismatches.push(`step ${step}: body linvel ${JSON.stringify(v)}`);

                    const wr = wasmRead(() => world.bodies.raw.rbRotation(b.handle), 4);
                    const r = b.rotation();
                    if (
                        !near(r.x, wr[0]) ||
                        !near(r.y, wr[1]) ||
                        !near(r.z, wr[2]) ||
                        !near(r.w, wr[3])
                    )
                        mismatches.push(`step ${step}: body rotation ${JSON.stringify(r)}`);
                });
                world.colliders.forEach((c) => {
                    const wt = wasmRead(() => world.colliders.raw.coTranslation(c.handle), 3);
                    const t = c.translation();
                    if (!near(t.x, wt[0]) || !near(t.y, wt[1]) || !near(t.z, wt[2]))
                        mismatches.push(`step ${step}: collider translation ${JSON.stringify(t)}`);
                });
            }

            const bodies: RAPIER.RigidBody[] = [];
            function spawn(fixed: boolean) {
                const desc = fixed ? RAPIER.RigidBodyDesc.fixed() : RAPIER.RigidBodyDesc.dynamic();
                const p = vec(20);
                desc.setTranslation(p.x, Math.abs(p.y) * 0.3 + 0.6, p.z);
                const b = world.createRigidBody(desc);
                world.createCollider(RAPIER.ColliderDesc.cuboid(0.4, 0.4, 0.4), b);
                bodies.push(b);
                return b;
            }
            for (let i = 0; i < 30; i++) spawn(i % 7 === 0);

            for (let step = 0; step < 300; step++) {
                // Occasionally a burst big enough to trip the pending-list cap,
                // which switches the sync over to rewriting everything.
                const burst = rnd() < 0.05 ? 80 : ri(4);
                for (let m = 0; m < burst; m++) {
                    const live = bodies.filter((b) => b.isValid());
                    if (live.length === 0) break;
                    const b = live[ri(live.length)];
                    const wake = rnd() < 0.5;
                    switch (ri(9)) {
                        case 0:
                            b.setTranslation(
                                {x: rf(-25, 25), y: rf(0.5, 12), z: rf(-25, 25)},
                                wake,
                            );
                            break;
                        case 1:
                            b.setLinvel(vec(4), wake);
                            break;
                        case 2:
                            b.applyImpulse(vec(3), wake);
                            break;
                        case 3:
                            if (rnd() < 0.5) b.sleep();
                            else b.wakeUp();
                            break;
                        case 4:
                            b.collider(ri(b.numColliders())).setTranslationWrtParent(vec(1.2));
                            break;
                        case 5:
                            b.setAngvel(vec(2), wake);
                            break;
                        case 6:
                            // Recycling an arena index is the interesting case: a
                            // fixed newcomer is never reported active, so only the
                            // refresh list can ever write its slot.
                            if (bodies.length < 70) spawn(ri(3) === 0);
                            break;
                        case 7:
                            if (live.length > 10 && rnd() < 0.5) {
                                const victim = live[ri(live.length)];
                                world.removeRigidBody(victim);
                                bodies.splice(bodies.indexOf(victim), 1);
                            }
                            break;
                        case 8:
                            if (b.numColliders() > 1)
                                world.removeCollider(b.collider(ri(b.numColliders())), true);
                            else world.createCollider(RAPIER.ColliderDesc.ball(0.35), b);
                            break;
                    }
                }

                if (rnd() < 0.02) world.propagateModifiedBodyPositionsToColliders();

                world.step();
                audit(step);
                if (mismatches.length > 0) break;
            }

            expect(mismatches).toEqual([]);
            world.free();
        });
    }
});
