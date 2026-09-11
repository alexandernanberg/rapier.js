import {describe, expect, it} from "vitest";
import {Rng} from "../core/rand";

describe("Rng", () => {
    /**
     * Golden values. If these change, every recorded replay is invalidated —
     * which is the point of pinning them.
     */
    it("produces a stable sequence for a known seed", () => {
        const rng = new Rng(0x5eed);
        const drawn = Array.from({length: 8}, () => rng.nextU32());

        expect(drawn).toMatchInlineSnapshot(`
          [
            3272804103,
            1323078912,
            1893449574,
            2910211328,
            4141569412,
            3407657872,
            2564510724,
            182147850,
          ]
        `);
    });

    it("replays identically from the same seed", () => {
        const a = new Rng(12345);
        const b = new Rng(12345);

        for (let i = 0; i < 1000; i++) {
            expect(a.nextU32()).toBe(b.nextU32());
        }
    });

    it("diverges for different seeds", () => {
        const a = new Rng(1);
        const b = new Rng(2);

        expect(a.nextU32()).not.toBe(b.nextU32());
    });

    it("clones its state so a world can be forked mid-match", () => {
        const rng = new Rng(99);
        for (let i = 0; i < 17; i++) rng.nextU32();

        const fork = rng.clone();
        expect(fork.state).toEqual(rng.state);
        expect(fork.nextU32()).toBe(rng.nextU32());
    });

    it("keeps nextFloat inside [0, 1)", () => {
        const rng = new Rng(7);
        for (let i = 0; i < 10000; i++) {
            const v = rng.nextFloat();
            expect(v).toBeGreaterThanOrEqual(0);
            expect(v).toBeLessThan(1);
        }
    });

    it("keeps nextInt inside [0, n)", () => {
        const rng = new Rng(7);
        for (let i = 0; i < 10000; i++) {
            const v = rng.nextInt(6);
            expect(v).toBeGreaterThanOrEqual(0);
            expect(v).toBeLessThan(6);
            expect(Number.isInteger(v)).toBe(true);
        }
    });

    it("survives a degenerate seed", () => {
        const rng = new Rng(0);
        expect(rng.state.some((v) => v !== 0)).toBe(true);
        expect(rng.nextU32()).not.toBe(rng.nextU32());
    });
});
