import * as fs from "node:fs";
import * as path from "node:path";
import type {ComparisonResult, ComparisonStatus} from "./baseline.js";
import type {BenchResult} from "./runner.js";

export function printTable(results: BenchResult[]): void {
    console.log("┌─────────────────────────────────┬──────────┬──────────┬──────────┐");
    console.log("│ Benchmark                       │ Mean     │ Min      │ Max      │");
    console.log("├─────────────────────────────────┼──────────┼──────────┼──────────┤");
    for (const r of results) {
        const name = r.name.slice(0, 31).padEnd(31);
        const mean = formatTime(r.mean).padStart(8);
        const min = formatTime(r.min).padStart(8);
        const max = formatTime(r.max).padStart(8);
        console.log(`│ ${name} │ ${mean} │ ${min} │ ${max} │`);
    }
    console.log("└─────────────────────────────────┴──────────┴──────────┴──────────┘");
}

export function printComparisonTable(comparisons: ComparisonResult[]): void {
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

export function saveResults(results: BenchResult[], filePath: string): void {
    const data = {
        timestamp: new Date().toISOString(),
        node: process.version,
        platform: process.platform,
        arch: process.arch,
        results,
    };

    const dir = path.dirname(filePath);
    fs.mkdirSync(dir, {recursive: true});
    fs.writeFileSync(filePath, JSON.stringify(data, null, 2));
    console.log(`\nResults saved to ${filePath}`);
}
