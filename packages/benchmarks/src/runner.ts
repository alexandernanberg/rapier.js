export interface BenchResult {
    name: string;
    category: string;
    mean: number;
    min: number;
    max: number;
    stdDev: number;
    samples: number;
}

export interface BenchOptions {
    warmup: number;
    iterations: number;
}

const defaultOptions: BenchOptions = {
    warmup: 100,
    iterations: 1000,
};

export function bench(name: string, fn: () => void, opts: Partial<BenchOptions> = {}): BenchResult {
    const {warmup, iterations} = {...defaultOptions, ...opts};

    // Warmup
    for (let i = 0; i < warmup; i++) fn();

    // Collect samples
    const times: number[] = [];
    for (let i = 0; i < iterations; i++) {
        const start = performance.now();
        fn();
        times.push(performance.now() - start);
    }

    const mean = times.reduce((a, b) => a + b, 0) / times.length;
    const variance = times.reduce((sum, t) => sum + (t - mean) ** 2, 0) / times.length;

    return {
        name,
        category: "",
        mean,
        min: Math.min(...times),
        max: Math.max(...times),
        stdDev: Math.sqrt(variance),
        samples: times.length,
    };
}

export function statsFrom(name: string, times: number[]): BenchResult {
    if (times.length === 0) {
        return {
            name,
            category: "",
            mean: 0,
            min: 0,
            max: 0,
            stdDev: 0,
            samples: 0,
        };
    }

    const mean = times.reduce((a, b) => a + b, 0) / times.length;
    const variance = times.reduce((sum, t) => sum + (t - mean) ** 2, 0) / times.length;

    return {
        name,
        category: "",
        mean,
        min: Math.min(...times),
        max: Math.max(...times),
        stdDev: Math.sqrt(variance),
        samples: times.length,
    };
}
