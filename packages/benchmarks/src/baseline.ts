import * as fs from "node:fs";
import * as path from "node:path";

export interface BenchResult {
    name: string;
    /** Average time in nanoseconds (as returned by mitata) */
    avg: number;
}

export interface BaselineEntry {
    mean: number;
    tolerance?: number;
}

export interface BaselineData {
    version: number;
    created: string;
    thresholds: {
        warning: number;
        regression: number;
    };
    "2d": Record<string, BaselineEntry>;
    "3d": Record<string, BaselineEntry>;
}

export type ComparisonStatus = "pass" | "warning" | "regression" | "new";

export interface ComparisonResult {
    name: string;
    mean: number;
    baseline: number | null;
    percentChange: number | null;
    status: ComparisonStatus;
}

const DEFAULT_THRESHOLDS = {
    warning: 0.15,
    regression: 0.3,
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
        return JSON.parse(content) as BaselineData;
    } catch (err) {
        console.warn(`Warning: Could not load baseline file: ${err}`);
        return null;
    }
}

export function saveBaseline(dim: "2d" | "3d", results: BenchResult[]): void {
    const baselinePath = getBaselinePath();
    let baseline: BaselineData;

    // Load existing baseline or create new one
    if (fs.existsSync(baselinePath)) {
        try {
            baseline = JSON.parse(fs.readFileSync(baselinePath, "utf-8"));
        } catch {
            baseline = createEmptyBaseline();
        }
    } else {
        baseline = createEmptyBaseline();
    }

    // Update the dimension's results (convert ns → ms for storage)
    baseline[dim] = {};
    for (const result of results) {
        baseline[dim][result.name] = {
            mean: result.avg / 1e6,
        };
    }

    // Update timestamp
    baseline.created = new Date().toISOString();

    fs.writeFileSync(baselinePath, JSON.stringify(baseline, null, 2));
    console.log(`\nBaseline saved to ${baselinePath}`);
}

function createEmptyBaseline(): BaselineData {
    return {
        version: 1,
        created: new Date().toISOString(),
        thresholds: DEFAULT_THRESHOLDS,
        "2d": {},
        "3d": {},
    };
}

export function compareToBaseline(
    dim: "2d" | "3d",
    results: BenchResult[],
    baseline: BaselineData,
): ComparisonResult[] {
    const baselineEntries = baseline[dim];
    const thresholds = baseline.thresholds ?? DEFAULT_THRESHOLDS;

    return results.map((result) => {
        // Convert ns → ms for comparison with baseline
        const meanMs = result.avg / 1e6;
        const entry = baselineEntries[result.name];

        if (!entry) {
            return {
                name: result.name,
                mean: meanMs,
                baseline: null,
                percentChange: null,
                status: "new" as ComparisonStatus,
            };
        }

        const percentChange = (meanMs - entry.mean) / entry.mean;
        const tolerance = entry.tolerance ?? thresholds.regression;

        let status: ComparisonStatus;
        if (percentChange > tolerance) {
            status = "regression";
        } else if (percentChange > thresholds.warning) {
            status = "warning";
        } else {
            status = "pass";
        }

        return {
            name: result.name,
            mean: meanMs,
            baseline: entry.mean,
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
    console.log("\nBaseline Comparison:");
    console.log("┌─────────────────────────────────┬──────────┬──────────┬────────────┐");
    console.log("│ Benchmark                       │ Mean     │ Baseline │ Status     │");
    console.log("├─────────────────────────────────┼──────────┼──────────┼────────────┤");
    for (const c of comparisons) {
        const name = c.name.slice(0, 31).padEnd(31);
        const mean = formatTime(c.mean).padStart(8);
        const baseline = c.baseline !== null ? formatTime(c.baseline).padStart(8) : "     N/A";
        const status = formatStatus(c.percentChange, c.status).padEnd(10);
        console.log(`│ ${name} │ ${mean} │ ${baseline} │ ${status} │`);
    }
    console.log("└─────────────────────────────────┴──────────┴──────────┴────────────┘");
}

function formatStatus(percentChange: number | null, status: ComparisonStatus): string {
    if (status === "new") {
        return "NEW";
    }

    if (percentChange === null) {
        return "N/A";
    }

    const sign = percentChange >= 0 ? "+" : "";
    const pct = `${sign}${(percentChange * 100).toFixed(0)}%`;

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

function formatTime(ms: number): string {
    if (ms < 0.001) {
        return `${(ms * 1000000).toFixed(0)}ns`;
    } else if (ms < 1) {
        return `${(ms * 1000).toFixed(1)}µs`;
    } else {
        return `${ms.toFixed(3)}ms`;
    }
}
