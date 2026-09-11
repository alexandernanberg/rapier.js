/**
 * Canonical state hashing.
 *
 * A lockstep sim is only as trustworthy as its desync detection: every client
 * hashes its whole world each checksum tick and compares. A mismatch means the
 * simulations have diverged and the match is already invalid.
 *
 * Two 32-bit FNV-1a lanes with different primes give a 64-bit digest, which is
 * plenty to make an accidental collision between two diverged worlds a
 * non-event, and needs no BigInt.
 */

const scratchF64 = new Float64Array(1);
const scratchU32 = new Uint32Array(scratchF64.buffer);

// Typed-array views use platform endianness. Every platform that matters is
// little-endian (WASM mandates it), but a big-endian client would otherwise
// hash the same state differently, so normalise the word order instead of
// assuming.
const IS_LITTLE_ENDIAN = (() => {
    scratchF64[0] = 1;
    const little = scratchU32[0] === 0 && scratchU32[1] === 0x3ff00000;
    scratchF64[0] = 0;
    return little;
})();

export class Hasher {
    private h1 = 0x811c9dc5;
    private h2 = 0x01000193;

    reset(): this {
        this.h1 = 0x811c9dc5;
        this.h2 = 0x01000193;
        return this;
    }

    writeU32(v: number): this {
        this.h1 = Math.imul(this.h1 ^ (v >>> 0), 0x01000193) >>> 0;
        this.h2 = Math.imul(this.h2 ^ (v >>> 0), 0x85ebca6b) >>> 0;
        return this;
    }

    /**
     * Hashes the exact bit pattern, so `0` and `-0` differ. That is deliberate:
     * identical operations produce identical bits, so a `-0`/`0` split is real
     * divergence and normalising it away would hide a bug. NaN payloads can
     * vary between engines — a NaN in sim state is itself a defect, and this
     * will surface it.
     */
    writeF64(v: number): this {
        scratchF64[0] = v;
        const lo = IS_LITTLE_ENDIAN ? scratchU32[0] : scratchU32[1];
        const hi = IS_LITTLE_ENDIAN ? scratchU32[1] : scratchU32[0];
        return this.writeU32(lo).writeU32(hi);
    }

    writeU32Array(a: Uint32Array): this {
        for (let i = 0; i < a.length; i++) this.writeU32(a[i]);
        return this;
    }

    /** 16-char hex digest. */
    digest(): string {
        return this.h1.toString(16).padStart(8, "0") + this.h2.toString(16).padStart(8, "0");
    }
}
