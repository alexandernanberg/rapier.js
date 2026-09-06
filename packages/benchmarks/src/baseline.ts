import * as fs from "node:fs";
import * as path from "node:path";
import {formatBytes, type MemoryResult} from "./memory.js";
import {formatPercent, formatTime, type BenchResult, type HostInfo} from "./results.js";

/**
 * The baseline is per machine and lives outside version control: a timing
 * recorded on one CPU says nothing about another, which is how a committed
 * baseline drifts into meaninglessness. Record it on master with
 * `--save-baseline`, then compare a branch against it.
 */
export interface BaselineEntry {
    /** Median time in milliseconds. */
    p50: number;
    /** Relative interquartile range when the baseline was recorded. */
    spread: number;
    /** Per-benchmark override of the regression threshold. */
    tolerance?: number;
}

export interface MemoryBaselineEntry {
    bytesPerOp: number;
}

export interface BaselineData {
    version: number;
    created: string;
    host?: HostInfo;
    thresholds: {
        warning: number;
        regression: number;
    };
    "2d": Record<string, BaselineEntry>;
    "3d": Record<string, BaselineEntry>;
    memory?: {
        "2d": Record<string, MemoryBaselineEntry>;
        "3d": Record<string, MemoryBaselineEntry>;
    };
}

export type ComparisonStatus = "pass" | "warning" | "regression" | "new";

export interface ComparisonResult {
    name: string;
    /** The measured value: median milliseconds, or bytes per op for memory rows. */
    value: number;
    /** Relative interquartile range of the measurement, where known. */
    spread: number | null;
    baseline: number | null;
    percentChange: number | null;
    status: ComparisonStatus;
}

const BASELINE_VERSION = 2;

const DEFAULT_THRESHOLDS = {
    warning: 0.15,
    regression: 0.3,
};

/**
 * Allocation thresholds, calibrated against the measured run-to-run spread: the
 * rows that allocate hundreds of bytes per op repeat to within a few percent,
 * while the near-zero rows wander by a few tens of bytes. Hence the absolute
 * floor — below it, a change is measurement noise rather than a real one.
 */
const MEMORY_THRESHOLDS = {
    warning: 0.25,
    regression: 0.5,
    minBytesDelta: 128,
};

function getBaselinePath(): string {
    return path.join(new URL("..", import.meta.url).pathname, "baseline.json");
}

export function loadBaseline(): BaselineData | null {
    const baselinePath = getBaselinePath();

    if (!fs.existsSync(baselinePath)) {
        return null;
    }

    try {
        const content = fs.readFileSync(baselinePath, "utf-8");
        const baseline = JSON.parse(content) as BaselineData;
        if (baseline.version !== BASELINE_VERSION) {
            console.warn(
                `Warning: baseline.json is version ${baseline.version}, expected ${BASELINE_VERSION}; ` +
                    "re-record it with --save-baseline.",
            );
            return null;
        }
        return baseline;
    } catch (err) {
        console.warn(`Warning: Could not load baseline file: ${err}`);
        return null;
    }
}

/** Which of the host fields differ, as a short description, or null if they match. */
export function hostMismatch(baseline: BaselineData, host: HostInfo): string | null {
    if (!baseline.host) return "no host recorded";
    const diffs: string[] = [];
    if (baseline.host.cpu !== host.cpu) diffs.push(`cpu: ${baseline.host.cpu}`);
    if (baseline.host.node !== host.node) diffs.push(`node: ${baseline.host.node}`);
    if (baseline.host.arch !== host.arch) diffs.push(`arch: ${baseline.host.arch}`);
    return diffs.length > 0 ? diffs.join(", ") : null;
}

export function saveBaseline(
    dim: "2d" | "3d",
    results: BenchResult[],
    memory: MemoryResult[],
    host: HostInfo,
): void {
    const baselinePath = getBaselinePath();
    let baseline: BaselineData | null = null;

    // Keep the other dimension's entries — unless they came from another
    // machine, in which case they'd only ever compare wrongly.
    if (fs.existsSync(baselinePath)) {
        try {
            const existing = JSON.parse(fs.readFileSync(baselinePath, "utf-8")) as BaselineData;
            if (existing.version === BASELINE_VERSION && !hostMismatch(existing, host)) {
                baseline = existing;
            }
        } catch {
            baseline = null;
        }
    }
    baseline ??= createEmptyBaseline();
    baseline.host = host;

    baseline[dim] = {};
    for (const result of results) {
        baseline[dim][result.name] = {p50: result.p50, spread: result.spread};
    }

    baseline.memory ??= {"2d": {}, "3d": {}};
    baseline.memory[dim] = {};
    for (const result of memory) {
        baseline.memory[dim][result.name] = {bytesPerOp: result.bytesPerOp};
    }

    // Update timestamp
    baseline.created = new Date().toISOString();

    fs.writeFileSync(baselinePath, JSON.stringify(baseline, null, 2));
    console.log(`\nBaseline saved to ${baselinePath}`);
}

function createEmptyBaseline(): BaselineData {
    return {
        version: BASELINE_VERSION,
        created: new Date().toISOString(),
        thresholds: DEFAULT_THRESHOLDS,
        "2d": {},
        "3d": {},
        memory: {"2d": {}, "3d": {}},
    };
}

export function compareMemoryToBaseline(
    dim: "2d" | "3d",
    results: MemoryResult[],
    baseline: BaselineData,
): ComparisonResult[] {
    const entries = baseline.memory?.[dim] ?? {};

    return results.map((result) => {
        const entry = entries[result.name];

        if (!entry) {
            return {
                name: result.name,
                value: result.bytesPerOp,
                spread: null,
                baseline: null,
                percentChange: null,
                status: "new" as ComparisonStatus,
            };
        }

        const delta = result.bytesPerOp - entry.bytesPerOp;

        // Below the floor the change is noise, and a ratio against a near-zero
        // baseline would report it as a spectacular percentage. Call it flat.
        if (Math.abs(delta) < MEMORY_THRESHOLDS.minBytesDelta) {
            return {
                name: result.name,
                value: result.bytesPerOp,
                spread: null,
                baseline: entry.bytesPerOp,
                percentChange: 0,
                status: "pass" as ComparisonStatus,
            };
        }

        const percentChange = entry.bytesPerOp > 0 ? delta / entry.bytesPerOp : 1;

        let status: ComparisonStatus;
        if (percentChange > MEMORY_THRESHOLDS.regression) {
            status = "regression";
        } else if (percentChange > MEMORY_THRESHOLDS.warning) {
            status = "warning";
        } else {
            status = "pass";
        }

        return {
            name: result.name,
            value: result.bytesPerOp,
            spread: null,
            baseline: entry.bytesPerOp,
            percentChange,
            status,
        };
    });
}

export function printMemoryComparisonTable(comparisons: ComparisonResult[]): void {
    console.log("\nAllocation Baseline Comparison:");
    console.log("┌─────────────────────────────────┬──────────┬──────────┬────────────┐");
    console.log("│ Benchmark                       │ Bytes/op │ Baseline │ Status     │");
    console.log("├─────────────────────────────────┼──────────┼──────────┼────────────┤");
    for (const c of comparisons) {
        const name = c.name.slice(0, 31).padEnd(31);
        const bytes = formatBytes(c.value).padStart(8);
        const base = c.baseline !== null ? formatBytes(c.baseline).padStart(8) : "     N/A";
        const status = formatStatus(c.percentChange, c.status).padEnd(10);
        console.log(`│ ${name} │ ${bytes} │ ${base} │ ${status} │`);
    }
    console.log("└─────────────────────────────────┴──────────┴──────────┴────────────┘");
}

export function compareToBaseline(
    dim: "2d" | "3d",
    results: BenchResult[],
    baseline: BaselineData,
): ComparisonResult[] {
    const baselineEntries = baseline[dim];
    const thresholds = baseline.thresholds ?? DEFAULT_THRESHOLDS;

    return results.map((result) => {
        const entry = baselineEntries[result.name];

        if (!entry) {
            return {
                name: result.name,
                value: result.p50,
                spread: result.spread,
                baseline: null,
                percentChange: null,
                status: "new" as ComparisonStatus,
            };
        }

        const percentChange = (result.p50 - entry.p50) / entry.p50;
        const tolerance = entry.tolerance ?? thresholds.regression;

        // A change smaller than the spread of either run is noise, whatever the
        // threshold says.
        const noise = Math.max(result.spread, entry.spread);

        let status: ComparisonStatus;
        if (percentChange > tolerance && percentChange > noise) {
            status = "regression";
        } else if (percentChange > thresholds.warning && percentChange > noise) {
            status = "warning";
        } else {
            status = "pass";
        }

        return {
            name: result.name,
            value: result.p50,
            spread: result.spread,
            baseline: entry.p50,
            percentChange,
            status,
        };
    });
}

export function hasRegression(comparisons: ComparisonResult[]): boolean {
    return comparisons.some((c) => c.status === "regression");
}

export function summarizeComparison(comparisons: ComparisonResult[]): {
    warnings: number;
    regressions: number;
    newBenchmarks: number;
} {
    let warnings = 0;
    let regressions = 0;
    let newBenchmarks = 0;

    for (const c of comparisons) {
        if (c.status === "warning") warnings++;
        else if (c.status === "regression") regressions++;
        else if (c.status === "new") newBenchmarks++;
    }

    return {warnings, regressions, newBenchmarks};
}

export function printComparisonTable(comparisons: ComparisonResult[]): void {
    console.log("\nBaseline Comparison (median):");
    console.log("┌─────────────────────────────────┬──────────┬───────┬──────────┬────────────┐");
    console.log("│ Benchmark                       │ p50      │ ±IQR  │ Baseline │ Status     │");
    console.log("├─────────────────────────────────┼──────────┼───────┼──────────┼────────────┤");
    for (const c of comparisons) {
        const name = c.name.slice(0, 31).padEnd(31);
        const value = formatTime(c.value).padStart(8);
        const spread = (c.spread !== null ? formatPercent(c.spread, false) : "").padStart(5);
        const baseline = c.baseline !== null ? formatTime(c.baseline).padStart(8) : "     N/A";
        const status = formatStatus(c.percentChange, c.status).padEnd(10);
        console.log(`│ ${name} │ ${value} │ ${spread} │ ${baseline} │ ${status} │`);
    }
    console.log("└─────────────────────────────────┴──────────┴───────┴──────────┴────────────┘");
}

function formatStatus(percentChange: number | null, status: ComparisonStatus): string {
    if (status === "new") {
        return "NEW";
    }

    if (percentChange === null) {
        return "N/A";
    }

    const pct = formatPercent(percentChange);

    switch (status) {
        case "pass":
            return `${pct} \u2713`;
        case "warning":
            return `${pct} \u26a0\ufe0f`;
        case "regression":
            return `${pct} \u274c`;
        default:
            return pct;
    }
}
