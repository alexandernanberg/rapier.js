/**
 * Deterministic stand-in for `Math.random`, installed while `build` runs.
 *
 * How much a query costs (and allocates) depends on how often it hits, so a
 * scene that differs run to run makes the numbers wander by far more than any
 * regression worth catching. Every benchmark that builds a randomized scene or
 * query set should do so under this.
 */
export function withSeededRandom<T>(seed: number, build: () => T): T {
    const original = Math.random;
    let state = seed;
    Math.random = () => {
        // xorshift32
        state ^= state << 13;
        state ^= state >>> 17;
        state ^= state << 5;
        return ((state >>> 0) % 1_000_000) / 1_000_000;
    };

    try {
        return build();
    } finally {
        Math.random = original;
    }
}
