import * as fs from "node:fs";
import * as path from "node:path";
import type { BenchResult } from "./runner.js";

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
  regression: 0.30,
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

export function saveBaseline(
  dim: "2d" | "3d",
  results: BenchResult[]
): void {
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

  // Update the dimension's results
  baseline[dim] = {};
  for (const result of results) {
    baseline[dim][result.name] = {
      mean: result.mean,
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
  baseline: BaselineData
): ComparisonResult[] {
  const baselineEntries = baseline[dim];
  const thresholds = baseline.thresholds ?? DEFAULT_THRESHOLDS;

  return results.map((result) => {
    const entry = baselineEntries[result.name];

    if (!entry) {
      return {
        name: result.name,
        mean: result.mean,
        baseline: null,
        percentChange: null,
        status: "new" as ComparisonStatus,
      };
    }

    const percentChange = (result.mean - entry.mean) / entry.mean;
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
      mean: result.mean,
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

  return { warnings, regressions, newBenchmarks };
}
