/**
 * System bodies, imported by both the main thread and the workers so that
 * serial and parallel runs execute byte-identical code. Every kernel operates on
 * a half-open entity range so it can be chunked.
 */

export const DT = 0.016;

export const JOB = {
    integrate: 0,
    heavy: 1,
    sumY: 2,
    collect: 3,
    shutdown: -1,
} as const;

export interface Slice {
    start: number;
    end: number;
}

/** Contiguous chunking. Chunk `i` of `n` over `len` items. */
export function slice(len: number, i: number, n: number): Slice {
    const per = Math.ceil(len / n);
    const start = Math.min(i * per, len);
    return {start, end: Math.min(start + per, len)};
}

/** Memory-bound: 6 reads, 3 writes, minimal arithmetic. */
export function integrate(c: Float32Array[], s: number, e: number): number {
    const px = c[0],
        py = c[1],
        pz = c[2],
        vx = c[3],
        vy = c[4],
        vz = c[5];
    for (let i = s; i < e; i++) {
        px[i] += vx[i] * DT;
        py[i] += vy[i] * DT;
        pz[i] += vz[i] * DT;
    }
    return 0;
}

/** Compute-dense: same traffic, far more arithmetic per element. */
export function heavy(c: Float32Array[], s: number, e: number): number {
    const px = c[0],
        py = c[1],
        pz = c[2],
        vx = c[3],
        vy = c[4],
        vz = c[5];
    for (let i = s; i < e; i++) {
        let x = px[i],
            y = py[i],
            z = pz[i];
        const ax = vx[i],
            ay = vy[i],
            az = vz[i];
        // a short integration chain — stands in for anything solver-ish
        for (let k = 0; k < 8; k++) {
            const d = x * ax + y * ay + z * az;
            x += (ax - x * d) * DT;
            y += (ay - y * d) * DT;
            z += (az - z * d) * DT;
        }
        px[i] = x;
        py[i] = y;
        pz[i] = z;
    }
    return 0;
}

/**
 * A reduction accumulated in f32. Float addition is not associative, so the
 * order in which chunk partials are combined is observable — this is the trap
 * the design's "no cross-entity accumulation without a deterministic reduce"
 * rule exists to prevent.
 */
export function sumY(c: Float32Array[], s: number, e: number): number {
    const py = c[1];
    const f = Math.fround;
    let acc = 0;
    for (let i = s; i < e; i++) acc = f(acc + py[i]);
    return acc;
}

/**
 * Stands in for a system that queues structural changes. Each chunk writes the
 * entity indices it matched into its own region, so merge order is decided by
 * the caller rather than by which thread happened to finish first.
 */
export function collect(
    c: Float32Array[],
    s: number,
    e: number,
    out: Int32Array,
    outBase: number,
): number {
    const py = c[1];
    let n = 0;
    for (let i = s; i < e; i++) {
        if (py[i] > 0.9) out[outBase + n++] = i;
    }
    return n;
}

export function runJob(
    job: number,
    c: Float32Array[],
    s: number,
    e: number,
    out: Int32Array,
    outBase: number,
): number {
    switch (job) {
        case JOB.integrate:
            return integrate(c, s, e);
        case JOB.heavy:
            return heavy(c, s, e);
        case JOB.sumY:
            return sumY(c, s, e);
        case JOB.collect:
            return collect(c, s, e, out, outBase);
        default:
            return 0;
    }
}
