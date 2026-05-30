import {run} from "mitata";
import * as fs from "node:fs";
import * as path from "node:path";
import {
    loadBaseline,
    saveBaseline,
    compareToBaseline,
    hasRegression,
    summarizeComparison,
    printComparisonTable,
    type BenchResult,
} from "./baseline.js";
import {benchGetters} from "./scenarios/getters.js";
import {benchLifecycle} from "./scenarios/lifecycle.js";
import {benchQueries} from "./scenarios/queries.js";
import {benchSetters} from "./scenarios/setters.js";
import {benchSimulation} from "./scenarios/simulation.js";

async function importRapier(dim: "2d" | "3d", simd: boolean, official: boolean) {
    if (official) {
        if (dim === "2d") {
            return simd
                ? await import("@dimforge/rapier2d-simd-compat")
                : await import("@dimforge/rapier2d-compat");
        } else {
            return simd
                ? await import("@dimforge/rapier3d-simd-compat")
                : await import("@dimforge/rapier3d-compat");
        }
    }
    // Our fork
    if (dim === "2d") {
        return simd
            ? await import("@alexandernanberg/rapier2d/compat-simd")
            : await import("@alexandernanberg/rapier2d/compat");
    } else {
        return simd
            ? await import("@alexandernanberg/rapier3d/compat-simd")
            : await import("@alexandernanberg/rapier3d/compat");
    }
}

const args = process.argv.slice(2);

function parseArgs() {
    let dim: "2d" | "3d" = "3d";
    let quick = false;
    let saveBaselineFlag = false;
    let noCompare = false;
    let simd = false;
    let official = false;

    for (const arg of args) {
        if (arg === "--dim=2d") dim = "2d";
        else if (arg === "--dim=3d") dim = "3d";
        else if (arg === "--quick") quick = true;
        else if (arg === "--save-baseline") saveBaselineFlag = true;
        else if (arg === "--no-compare") noCompare = true;
        else if (arg === "--simd") simd = true;
        else if (arg === "--official") official = true;
        else if (arg === "--help" || arg === "-h") {
            console.log(`
Rapier.js Benchmark Suite

Usage: pnpm bench [options]

Options:
  --dim=2d          Run 2D benchmarks
  --dim=3d          Run 3D benchmarks (default)
  --simd            Use SIMD variant (requires simd128 support)
  --official        Use official @dimforge/rapier packages instead of fork
  --quick           Run with fewer bodies (faster setup, same measurement precision)
  --save-baseline   Save current results as new baseline
  --no-compare      Run without baseline comparison
  --help, -h        Show this help message

Examples:
  pnpm bench                    # Run fork and compare against baseline
  pnpm bench --simd             # Run fork with SIMD and compare
  pnpm bench --official         # Run with official @dimforge packages
  pnpm bench --save-baseline    # Save current results as new baseline
  pnpm bench --no-compare       # Run without baseline comparison
  pnpm bench:2d                 # Full 2D benchmark
  pnpm bench --quick            # Quick 3D benchmark
`);
            process.exit(0);
        }
    }

    return {dim, quick, saveBaselineFlag, noCompare, simd, official};
}

async function main() {
    const {dim, quick, saveBaselineFlag, noCompare, simd, official} = parseArgs();
    const is3D = dim === "3d";

    const modifiers = [
        quick ? "quick mode" : null,
        simd ? "SIMD" : null,
        official ? "official @dimforge" : null,
    ].filter(Boolean);
    const modifierStr = modifiers.length > 0 ? ` (${modifiers.join(", ")})` : "";

    console.log(`\nRapier ${dim.toUpperCase()} Benchmarks${modifierStr}\n`);

    // Import the appropriate package
    const RAPIER = await importRapier(dim, simd, official);

    await RAPIER.init();

    // Register all benchmarks (mitata collects them globally)
    benchSimulation(RAPIER, is3D, quick);
    benchLifecycle(RAPIER, is3D, quick);
    benchQueries(RAPIER, is3D, quick);
    benchGetters(RAPIER, is3D, quick);
    benchSetters(RAPIER, is3D, quick);

    // Run all registered benchmarks
    const {benchmarks} = await run();

    // Extract results from mitata's format
    const results: BenchResult[] = benchmarks.flatMap((trial) =>
        trial.runs
            .filter((r) => r.stats != null)
            .map((r) => ({
                name: r.name,
                avg: r.stats!.avg,
            })),
    );

    // Save results JSON for CI (convert ns → ms to match expected format)
    const resultsDir = path.join(new URL("..", import.meta.url).pathname, "results");
    fs.mkdirSync(resultsDir, {recursive: true});
    const resultsFile = path.join(resultsDir, `${dim}-${Date.now()}.json`);
    fs.writeFileSync(
        resultsFile,
        JSON.stringify(
            {
                timestamp: new Date().toISOString(),
                node: process.version,
                platform: process.platform,
                arch: process.arch,
                results: benchmarks.flatMap((trial) =>
                    trial.runs
                        .filter((r) => r.stats != null)
                        .map((r) => ({
                            name: r.name,
                            mean: r.stats!.avg / 1e6,
                            min: r.stats!.min / 1e6,
                            max: r.stats!.max / 1e6,
                            p50: r.stats!.p50 / 1e6,
                            p99: r.stats!.p99 / 1e6,
                            samples: r.stats!.samples.length,
                        })),
                ),
            },
            null,
            2,
        ),
    );
    console.log(`\nResults saved to ${resultsFile}`);

    // Handle baseline operations
    if (saveBaselineFlag) {
        saveBaseline(dim, results);
    } else if (!noCompare) {
        const baseline = loadBaseline();

        if (baseline && Object.keys(baseline[dim]).length > 0) {
            const comparisons = compareToBaseline(dim, results, baseline);
            printComparisonTable(comparisons);

            const summaryResult = summarizeComparison(comparisons);
            const parts: string[] = [];

            if (summaryResult.newBenchmarks > 0) {
                parts.push(`${summaryResult.newBenchmarks} new`);
            }
            if (summaryResult.warnings > 0) {
                parts.push(
                    `\u26a0\ufe0f ${summaryResult.warnings} warning${summaryResult.warnings > 1 ? "s" : ""}`,
                );
            }
            if (summaryResult.regressions > 0) {
                parts.push(
                    `\u274c ${summaryResult.regressions} regression${summaryResult.regressions > 1 ? "s" : ""}`,
                );
            }

            if (parts.length > 0) {
                console.log(`\n${parts.join(", ")}`);
            } else {
                console.log("\n\u2713 All benchmarks within tolerance");
            }

            if (hasRegression(comparisons)) {
                process.exit(1);
            }
        } else {
            console.log("\nNo baseline found. Run with --save-baseline to create one.");
        }
    }
}

main().catch((err) => {
    console.error("Benchmark failed:", err);
    process.exit(1);
});
