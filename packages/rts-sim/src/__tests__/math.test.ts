import {describe, expect, it} from "vitest";
import {atan2, cos, length, sin, TWO_PI} from "../core/math";

/**
 * These compare against `Math.*` only to prove the replacements are *accurate*.
 * Their determinism comes from being pure arithmetic — `Math.sin` and friends
 * are engine-dependent approximations, which is exactly why the sim cannot use
 * them.
 */
describe("deterministic math", () => {
    it("matches Math.sin and Math.cos across several full turns", () => {
        for (let i = -2000; i <= 2000; i++) {
            const x = (i / 2000) * 4 * TWO_PI;
            expect(sin(x)).toBeCloseTo(Math.sin(x), 9);
            expect(cos(x)).toBeCloseTo(Math.cos(x), 9);
        }
    });

    it("matches Math.atan2 over every octant", () => {
        const values = [-8, -3, -1, -0.25, 0, 0.25, 1, 3, 8];
        for (const y of values) {
            for (const x of values) {
                if (x === 0 && y === 0) continue;
                expect(atan2(y, x)).toBeCloseTo(Math.atan2(y, x), 9);
            }
        }
    });

    it("handles the axes atan2 has to special-case", () => {
        expect(atan2(0, 0)).toBe(0);
        expect(atan2(1, 0)).toBeCloseTo(Math.PI / 2, 12);
        expect(atan2(-1, 0)).toBeCloseTo(-Math.PI / 2, 12);
        expect(atan2(0, -1)).toBeCloseTo(Math.PI, 9);
        expect(atan2(0, 1)).toBe(0);
    });

    it("returns bit-identical results for repeated calls", () => {
        for (let i = 0; i < 500; i++) {
            const x = i * 0.37 - 90;
            expect(sin(x)).toBe(sin(x));
            expect(atan2(x, x * 0.5 + 1)).toBe(atan2(x, x * 0.5 + 1));
        }
    });

    it("computes length exactly, without Math.hypot", () => {
        expect(length(3, 4)).toBe(5);
        expect(length(0, 0)).toBe(0);
    });
});
