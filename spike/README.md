# Engine spike

Throwaway-quality code kept for its measurements. Nothing here is engine API —
it exists to answer specific questions before any of it gets designed for real.

Not part of the pnpm workspace (`packages/*`), so it never enters a build.

```bash
node spike/bench/iteration.ts          # spike 02 — ECS iteration through archetypes
node spike/bench/kernels/bench.mjs     # spike 01 — JS vs Rust/WASM kernels
node spike/bench/kernels/bench2.mjs    # spike 01 — fairness + boundary costs
node spike/bench/kernels/divergence.mjs # spike 01 — JS/Rust f32 agreement
node spike/bench/parallel.ts           # spike 03 — parallel executor + equivalence
```

The kernel benchmarks need the wasm built first:

```bash
cd spike/bench/kernels
rustc --target wasm32-unknown-unknown -O --crate-type=cdylib -o kernels_scalar.wasm kernels.rs
rustc --target wasm32-unknown-unknown -O -C target-feature=+simd128 --crate-type=cdylib -o kernels_simd.wasm kernels.rs
```

## Spike 01 — is a JS/WASM hybrid worth it?

Identical kernels, same linear memory, 1M entities, Node 22 / x64.

| kernel                | JS       | wasm           | wasm+simd      |
| --------------------- | -------- | -------------- | -------------- |
| integrate (mem-bound) | 6.09 ms  | 1.86 ms (3.3x) | 0.75 ms (8.2x) |
| qmul (compute-dense)  | 10.94 ms | 5.15 ms (2.1x) | 2.19 ms (5.0x) |

The JS baseline is the fastest of four hand-tuned variants (split loops, hoisted
locals, `Math.fround`), so it isn't a strawman.

**The win is SIMD, not Rust.** V8 does not auto-vectorize; LLVM does. Scalar wasm
buys 2–3x, vectorized wasm buys 5–8x. Kernels that don't vectorize are worth far
less than they look.

**Boundary calls are cheap; allocation across them is not.**

| per-entity pattern       | cost    |
| ------------------------ | ------- |
| pure JS, no call         | 1.8 ns  |
| bare wasm call           | 3.0 ns  |
| wasm call + scratch read | 5.8 ns  |
| wasm call + object alloc | 15.6 ns |

A `{x,y,z}` allocation costs 1.8 ns alone because V8's escape analysis deletes
it, and 15.6 ns when a wasm call is in the loop, because the call blocks that
optimization. The rule is _don't let allocations escape_, not _don't allocate_.

**JS and Rust disagree on f32.** Same quaternion product, same inputs: 48.8% of
results differ, up to 10,478 ULP. JS keeps f64 intermediates and rounds on store;
Rust rounds every operation. Porting a system between languages is a behaviour
change, not an optimization.

## Spike 02 — does the cursor survive a real archetype?

1M entities across 5 archetypes, columns as views into one shared WASM memory.

| pattern               | time     | throughput | vs raw |
| --------------------- | -------- | ---------- | ------ |
| raw columns           | 5.89 ms  | 170 M/s    | 1.00x  |
| cursor, generated     | 8.34 ms  | 120 M/s    | 1.41x  |
| cursor, array-indexed | 11.81 ms | 85 M/s     | 2.00x  |
| each(cb) + generated  | 14.66 ms | 68 M/s     | 2.49x  |
| cursor, keyed access  | 80.56 ms | 12 M/s     | 13.67x |

**This reverses spike 01's iteration result.** An earlier microbenchmark had the
cursor _beating_ raw column access (0.75x). That was an artifact of
closure-captured module constants which V8 could hoist entirely. Through real
archetype resolution the cursor is 1.41x slower at best. The lesson is about
microbenchmarks, not about cursors.

**How the accessor is built matters ~10x.** `this[slot][this.i]` with `slot` a
string variable is a _keyed_ load and costs 13.67x. Generating `this._3[this.i]`
as a real named load costs 1.41x. Same shape, same indirection, an order of
magnitude apart.

**The per-entity callback costs another ~1.8x** on top of the generated cursor.
Ergonomics are not free here, which argues for a tiered API: `each(cb)` by
default, raw chunk access as a documented escape hatch for hot systems.

**SharedArrayBuffer is free** — 0.96x versus a plain `ArrayBuffer`, within noise.
The memory model's central assumption holds.

**Structural change is ~100x more expensive than iteration.**

| operation            | throughput  |
| -------------------- | ----------- |
| spawn                | 1.7–2.1 M/s |
| add+remove component | 1.8 M/s     |
| despawn              | 16.1 M/s    |

At ~550 ns each, ten thousand component add/removes would cost 5.5 ms — a third
of a 60 Hz frame. Deferring them to a command buffer is a correctness feature
that turns out to be a performance one too. Note this implementation is naive:
it does a `Map` lookup per column per move, where a real ECS caches archetype
transition edges, so treat these as a floor rather than a ceiling.

## Spike 03 — the parallel executor

One archetype, 1M entities, columns in a shared WASM memory, chunked across
worker threads. The main thread takes a chunk and spins rather than
`Atomics.wait`-ing, because a browser main thread may not block.

| job                   | 1t      | 2t      | 3t      | 4t      | speedup |
| --------------------- | ------- | ------- | ------- | ------- | ------- |
| integrate (mem-bound) | 6.30ms  | 3.25ms  | 2.15ms  | 1.63ms  | 3.86x   |
| heavy (compute-dense) | 56.30ms | 28.27ms | 18.75ms | 14.26ms | 3.95x   |

**Scaling is near-linear at four cores, even for the memory-bound kernel.** The
worry that streaming systems would saturate bandwidth before saturating cores
does not show up at this core count — it likely would at 16.

**Serial and parallel runs are byte-identical** for per-entity systems, over ten
ticks at four threads. That proves the absence of races and torn writes. It does
not prove order-insensitivity, which is tested separately below.

### The chunk count is part of the observable result

| reduction (f32 accumulate over 1M values) | result     |
| ----------------------------------------- | ---------- |
| serial, 1 chunk                           | 499923.438 |
| serial, 4 chunks                          | 499923.094 |
| parallel, 4 threads                       | 499923.094 |

Parallel matches serial exactly **at the same chunk count**, and differs from a
single-chunk serial run. Float addition is not associative, so chunking changes
the summation tree.

The consequence is a design rule sharper than the one the doc originally
carried: **the chunk count must be a constant of the build, never derived from
`hardwareConcurrency`.** If chunking tracks core count, the simulation result
depends on the player's CPU, and replays stop reproducing across machines. Fix
the chunk count (say 8) and distribute those chunks over however many workers
happen to exist.

### Command buffer merge order is load-bearing

100,122 entities matched a predicate across four chunks. Merging their per-chunk
regions by chunk index versus by completion order produces **different
sequences** — confirming that per-worker command buffers must be merged in a
fixed order, not as threads finish. Merge order for the 4-way reduction happened
not to matter on this run, which is a good illustration of why this class of bug
survives casual testing.

## Caveat on codegen

The generated cursor uses `new Function`, which a strict CSP blocks. Since system
declarations are static, the robust path is generating cursors at build time from
the component registry rather than at runtime.

## Still open

- GC pause _distribution_ under a realistic system mix — throughput is settled,
  frame-time consistency is not.
- Whether the 1.41x holds once queries carry change-detection and optional
  components.
