/**
 * Seeded PRNG for the deterministic simulation.
 *
 * `Math.random()` is banned in `sim/` — every client must draw the same numbers
 * in the same order. This is xoshiro128**, which needs only `Math.imul` and
 * bitwise ops, both of which are exactly specified by ECMAScript and therefore
 * bit-identical on every platform.
 *
 * The state is a plain `Uint32Array` so it can be hashed and snapshotted along
 * with the rest of the world.
 */
export class Rng {
    readonly state: Uint32Array;

    constructor(seed: number) {
        this.state = new Uint32Array(4);
        this.seed(seed);
    }

    /** Re-seeds from a single integer via splitmix32. */
    seed(seed: number): void {
        let s = seed >>> 0;
        for (let i = 0; i < 4; i++) {
            s = (s + 0x9e3779b9) >>> 0;
            let z = s;
            z = Math.imul(z ^ (z >>> 16), 0x21f0aaad) >>> 0;
            z = Math.imul(z ^ (z >>> 15), 0x735a2d97) >>> 0;
            this.state[i] = (z ^ (z >>> 15)) >>> 0;
        }
        // An all-zero state is a fixed point for xoshiro; nudge it.
        if ((this.state[0] | this.state[1] | this.state[2] | this.state[3]) === 0) {
            this.state[0] = 1;
        }
    }

    /** Uniform in [0, 2^32). */
    nextU32(): number {
        const s = this.state;
        const result = (rotl(Math.imul(s[1], 5), 7) * 9) >>> 0;
        const t = (s[1] << 9) >>> 0;

        s[2] ^= s[0];
        s[3] ^= s[1];
        s[1] ^= s[2];
        s[0] ^= s[3];
        s[2] ^= t;
        s[3] = rotl(s[3], 11);

        return result;
    }

    /** Uniform in [0, 1). Exact: a u32 scaled by a power of two is lossless in f64. */
    nextFloat(): number {
        return this.nextU32() * 2.3283064365386963e-10;
    }

    /** Uniform in [min, max). */
    nextRange(min: number, max: number): number {
        return min + this.nextFloat() * (max - min);
    }

    /** Uniform integer in [0, n). */
    nextInt(n: number): number {
        return Math.floor(this.nextFloat() * n);
    }

    clone(): Rng {
        const copy = new Rng(0);
        copy.state.set(this.state);
        return copy;
    }
}

function rotl(x: number, k: number): number {
    return ((x << k) | (x >>> (32 - k))) >>> 0;
}
