import * as os from "node:os";

/** One timed benchmark, all durations in milliseconds. */
export interface BenchResult {
    name: string;
    mean: number;
    /** Median — what comparisons use, since it ignores GC and scheduler outliers. */
    p50: number;
    min: number;
    max: number;
    p25: number;
    p75: number;
    p99: number;
    /**
     * Interquartile range relative to the median: how much a difference has to
     * be to mean anything. Quartiles rather than a standard deviation because a
     * single GC pause in the samples would otherwise dominate the figure.
     */
    spread: number;
    samples: number;
}

export interface MemoryResultEntry {
    name: string;
    bytesPerOp: number;
    gcPerMillionOps: number;
    gcPauseMsPerMillionOps: number;
}

/** Enough about the machine to tell whether two runs are comparable at all. */
export interface HostInfo {
    cpu: string;
    cores: number;
    node: string;
    platform: string;
    arch: string;
}

/** The JSON a run writes to `results/`, and what `report.ts` consumes. */
export interface ResultsFile {
    version: 2;
    timestamp: string;
    dim: "2d" | "3d";
    package: "fork" | "official";
    /** Published version of the package under test, where it has one. */
    packageVersion?: string;
    simd: boolean;
    quick: boolean;
    host: HostInfo;
    results: BenchResult[];
    memory: MemoryResultEntry[];
}

export function describeHost(): HostInfo {
    return {
        cpu: os.cpus()[0]?.model ?? "unknown",
        cores: os.cpus().length,
        node: process.version,
        platform: process.platform,
        arch: process.arch,
    };
}

export function formatTime(ms: number): string {
    if (ms < 0.001) {
        return `${(ms * 1000000).toFixed(0)}ns`;
    } else if (ms < 1) {
        return `${(ms * 1000).toFixed(1)}µs`;
    } else {
        return `${ms.toFixed(3)}ms`;
    }
}

export function formatPercent(fraction: number, signed = true): string {
    const pct = (fraction * 100).toFixed(0);
    return signed && fraction >= 0 ? `+${pct}%` : `${pct}%`;
}
