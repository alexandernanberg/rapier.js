/**
 * Deterministic math for the simulation.
 *
 * Float determinism in JS is better than its reputation: `+ - * /` and
 * `Math.sqrt` are IEEE-754 exact, and `Math.abs`, `Math.floor`, `Math.ceil`,
 * `Math.round`, `Math.sign`, `Math.min`, `Math.max` and `Math.imul` are all
 * exactly specified by ECMAScript. Those are safe to use anywhere.
 *
 * The transcendentals are NOT. `Math.sin`, `cos`, `tan`, `asin`, `acos`,
 * `atan`, `atan2`, `exp`, `log`, `pow`, `hypot` and `cbrt` are only required to
 * be *approximations*, and engines differ — so two clients can disagree in the
 * last bits and desync. They are banned in `sim/`; use the replacements here,
 * which are pure arithmetic and therefore bit-identical everywhere.
 *
 * Accuracy is verified against `Math.*` in the test suite. These are slightly
 * less accurate than the native versions; that is the trade for reproducibility.
 */

export const PI = 3.141592653589793;
export const TWO_PI = 6.283185307179586;
export const HALF_PI = 1.5707963267948966;

// pi/2 split into two doubles so range reduction keeps its low bits.
const HALF_PI_HI = 1.5707963267341256;
const HALF_PI_LO = 6.077100506506192e-11;
const TWO_OVER_PI = 0.6366197723675814;

/**
 * Deterministic replacement for `Math.sin`. Measured error is below 2e-10 for
 * arguments within a few turns of zero; it grows slowly with |x| as range
 * reduction loses bits, so keep accumulated angles normalised.
 */
export function sin(x: number): number {
    const k = Math.round(x * TWO_OVER_PI);
    const r = x - k * HALF_PI_HI - k * HALF_PI_LO;
    switch (k & 3) {
        case 0:
            return sinPoly(r);
        case 1:
            return cosPoly(r);
        case 2:
            return -sinPoly(r);
        default:
            return -cosPoly(r);
    }
}

/** Deterministic replacement for `Math.cos`. Same accuracy and caveat as `sin`. */
export function cos(x: number): number {
    const k = Math.round(x * TWO_OVER_PI);
    const r = x - k * HALF_PI_HI - k * HALF_PI_LO;
    switch (k & 3) {
        case 0:
            return cosPoly(r);
        case 1:
            return -sinPoly(r);
        case 2:
            return -cosPoly(r);
        default:
            return sinPoly(r);
    }
}

/**
 * Deterministic replacement for `Math.atan2`. Measured error is below 5e-10 rad
 * — far beyond what a unit facing angle can show.
 */
export function atan2(y: number, x: number): number {
    if (x === 0) {
        if (y > 0) return HALF_PI;
        if (y < 0) return -HALF_PI;
        return 0;
    }

    const ax = Math.abs(x);
    const ay = Math.abs(y);
    // Keep the ratio inside [-1, 1] so the polynomial stays in its fitted range.
    const swap = ay > ax;
    const z = swap ? ax / ay : ay / ax;
    let a = atanPoly(z);
    if (swap) a = HALF_PI - a;
    if (x < 0) a = PI - a;
    return y < 0 ? -a : a;
}

/** Taylor series for sin on [-pi/4, pi/4], through r^11. */
function sinPoly(r: number): number {
    const r2 = r * r;
    return (
        r *
        (1 +
            r2 *
                (-0.16666666666666666 +
                    r2 *
                        (0.008333333333333333 +
                            r2 *
                                (-1.984126984126984e-4 +
                                    r2 * (2.7557319223985893e-6 + r2 * -2.505210838544172e-8)))))
    );
}

/** Taylor series for cos on [-pi/4, pi/4], through r^10. */
function cosPoly(r: number): number {
    const r2 = r * r;
    return (
        1 +
        r2 *
            (-0.5 +
                r2 *
                    (0.041666666666666664 +
                        r2 *
                            (-0.001388888888888889 +
                                r2 * (2.48015873015873e-5 + r2 * -2.7557319223985893e-7))))
    );
}

const TAN_PI_8 = 0.41421356237309503;
const QUARTER_PI = 0.7853981633974483;

/**
 * atan on [0, 1].
 *
 * Halving the interval at tan(pi/8) via
 * `atan(z) = pi/4 + atan((z - 1) / (z + 1))` is what makes a plain Taylor
 * series practical here: on [0, 1] it would converge far too slowly, but on
 * [-tan(pi/8), tan(pi/8)] the terms through z^19 already land near 3e-9.
 */
function atanPoly(z: number): number {
    if (z > TAN_PI_8) {
        return QUARTER_PI + atanSmall((z - 1) / (z + 1));
    }
    return atanSmall(z);
}

/** Taylor series for atan on [-tan(pi/8), tan(pi/8)]; coefficients are 1/(2k+1). */
function atanSmall(z: number): number {
    const z2 = z * z;
    return (
        z *
        (1 +
            z2 *
                (-0.3333333333333333 +
                    z2 *
                        (0.2 +
                            z2 *
                                (-0.14285714285714285 +
                                    z2 *
                                        (0.1111111111111111 +
                                            z2 *
                                                (-0.09090909090909091 +
                                                    z2 *
                                                        (0.07692307692307693 +
                                                            z2 *
                                                                (-0.06666666666666667 +
                                                                    z2 *
                                                                        (0.058823529411764705 +
                                                                            z2 *
                                                                                -0.05263157894736842)))))))))
    );
}

/** Squared length — prefer this to `length` when you only need to compare. */
export function lengthSq(x: number, y: number): number {
    return x * x + y * y;
}

/** `Math.sqrt` is IEEE-exact, so this is safe. Never use `Math.hypot`. */
export function length(x: number, y: number): number {
    return Math.sqrt(x * x + y * y);
}

export function clamp(v: number, min: number, max: number): number {
    return v < min ? min : v > max ? max : v;
}
