import type {BenchResult} from "./runner.js";
import {
    loadBaseline,
    saveBaseline,
    compareToBaseline,
    hasRegression,
    summarizeComparison,
} from "./baseline.js";
import {printTable, printComparisonTable, saveResults} from "./results.js";
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
            ? await import("@alexandernanberg/rapier-2d/compat-simd")
            : await import("@alexandernanberg/rapier-2d/compat");
    } else {
        return simd
            ? await import("@alexandernanberg/rapier-3d/compat-simd")
            : await import("@alexandernanberg/rapier-3d/compat");
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
  --quick           Run with fewer iterations (faster, less accurate)
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

    const results: BenchResult[] = [];

    console.log("Running simulation benchmarks...");
    results.push(...(await benchSimulation(RAPIER, is3D, quick)));

    console.log("Running lifecycle benchmarks...");
    results.push(...(await benchLifecycle(RAPIER, is3D, quick)));

    console.log("Running query benchmarks...");
    results.push(...(await benchQueries(RAPIER, is3D, quick)));

    console.log("Running getter benchmarks...");
    results.push(...(await benchGetters(RAPIER, is3D, quick)));

    console.log("Running setter benchmarks...");
    results.push(...(await benchSetters(RAPIER, is3D, quick)));

    console.log("");

    // Handle baseline operations
    if (saveBaselineFlag) {
        printTable(results);
        saveBaseline(dim, results);
    } else if (noCompare) {
        printTable(results);
    } else {
        // Try to compare against baseline
        const baseline = loadBaseline();

        if (baseline && Object.keys(baseline[dim]).length > 0) {
            const comparisons = compareToBaseline(dim, results, baseline);
            printComparisonTable(comparisons);

            const summary = summarizeComparison(comparisons);
            const parts: string[] = [];

            if (summary.newBenchmarks > 0) {
                parts.push(`${summary.newBenchmarks} new`);
            }
            if (summary.warnings > 0) {
                parts.push(
                    `\u26a0\ufe0f ${summary.warnings} warning${summary.warnings > 1 ? "s" : ""}`,
                );
            }
            if (summary.regressions > 0) {
                parts.push(
                    `\u274c ${summary.regressions} regression${summary.regressions > 1 ? "s" : ""}`,
                );
            }

            if (parts.length > 0) {
                console.log(`\n${parts.join(", ")}`);
            } else {
                console.log("\n\u2713 All benchmarks within tolerance");
            }

            // Exit with error code if regression detected
            if (hasRegression(comparisons)) {
                const timestamp = Date.now();
                const resultsDir = new URL("../results", import.meta.url).pathname;
                saveResults(results, `${resultsDir}/${dim}-${timestamp}.json`);
                process.exit(1);
            }
        } else {
            // No baseline exists, just print results
            printTable(results);
            console.log("\nNo baseline found. Run with --save-baseline to create one.");
        }
    }

    const timestamp = Date.now();
    const resultsDir = new URL("../results", import.meta.url).pathname;
    saveResults(results, `${resultsDir}/${dim}-${timestamp}.json`);
}

main().catch((err) => {
    console.error("Benchmark failed:", err);
    process.exit(1);
});
