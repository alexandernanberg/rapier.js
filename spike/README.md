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
node spike/test/harness.ts             # spike 04 — headless harness (13 checks)
node spike/test/physics.ts             # spike 05 — Rapier over ECS columns (7 checks)
node spike/test/koota-eval.ts          # spike 06 — evaluate pmndrs/koota as the ECS
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

## Spike 04 — the headless harness

A deterministic, steppable simulation with input as per-tick data, plus a small
platformer to assert against. 13 checks, all passing.

```bash
node spike/test/harness.ts     # exit 0 / 1, no browser, no canvas
```

The properties the harness owes an agent, all verified:

| property                              | result           |
| ------------------------------------- | ---------------- |
| same inputs twice → identical world   | byte-identical   |
| recording replays → identical world   | byte-identical   |
| snapshot then restore rewinds exactly | byte-identical   |
| recording size                        | 16 B/player/tick |

Replay works by injecting recorded frames and stepping without re-latching, so
the recorded frame is the only source of truth. A 90-tick session with jumps,
movement and pickup despawns replays to the byte.

### Writing the tests found two real problems

Both were found on first use, which is the argument for building the harness
before the renderer rather than after.

**`press()` without `release()` silently suppresses later edges.** A test pressed
jump while airborne, landed, pressed again, and the second jump never fired —
because the action was still held, so there was no rising edge. The fix is a
`tap()` primitive (down and up inside one tick, riding the sticky bit), which is
what a test almost always means and what a fast human tap actually is. There is
now a regression test asserting that a held action does _not_ auto-repeat.

**Jump launch velocity is not `JUMP_SPEED`.** It is `JUMP_SPEED + g*dt`, because
gravity runs after the impulse within the same tick. That is correct
semi-implicit Euler and the sim is right — but nothing in the source makes it
visible, and it is exactly what would confuse whoever tunes jump height. The
assertion now states the real value and explains the ordering.

Neither is an exotic bug. Both are the kind of thing that ships quietly and gets
discovered as "the jump feels wrong", which is the failure mode a machine-checkable
oracle exists to prevent.

## Spike 05 — Rapier over ECS columns

A purpose-built Rapier backend (`spike/crates/physics-ecs`) with a raw C ABI and
**no wasm-bindgen**. It exposes no objects: the hot path is push kinematic, step,
pull transforms — three calls per frame regardless of body count. The ECS arena is
allocated _inside this module's linear memory_, so Rapier writes transforms into
their final home in the component columns.

```bash
cargo build --target wasm32-unknown-unknown -p physics-ecs   # 1.79 MB artifact
node spike/test/physics.ts                                   # 7 checks
```

The tier-1 zero-copy claim holds. Verified: the column's backing buffer _is_ the
wasm memory, and stepping mutates the ECS columns directly with no intermediate.

### Sync is 2.6% of the step it follows

| 20,000 bodies          | time     |
| ---------------------- | -------- |
| `phys_step`            | 11.67 ms |
| `pull_transforms` bulk | 0.30 ms  |
| per body               | 14.9 ns  |

The thing the design spent pages worrying about is noise next to the solver. This
also retires the tier-1/tier-2 distinction as a practical concern: if bulk sync is
2.6% when it is free, a foreign module's memcpy version would still be a rounding
error. **Pluggability was never going to cost what it looked like it would.**

### The trap: growing the heap detaches every view

The first run failed in a way worth recording. Rapier allocates during `step`, the
heap grew, and growing a non-shared `WebAssembly.Memory` **detaches every JS view
over it**. The failure mode is nasty: a detached typed array reports
`byteOffset === 0` rather than throwing, so Rust was handed a null pointer and
panicked from deep inside a slice constructor. Nothing in the stack trace pointed
at the actual cause.

Two fixes, both kept:

1. `phys_new` takes a `reserve_bytes` and claims allocator headroom up front, then
   frees it — so later Rapier allocations come from that pool and never call
   `memory.grow`. With it, neither body creation nor stepping moves the memory.
2. The seam checks for detached views and raises an error that names the cause and
   the fix, rather than letting a null pointer through.

This is the strongest argument yet for the design's note that a shared memory
would be preferable: a `SharedArrayBuffer`-backed memory does not detach on grow,
which removes the whole class of bug rather than mitigating it.

## Spike 06 — can Koota replace the custom ECS?

[pmndrs/koota](https://github.com/pmndrs/koota) 0.6.6 converges on the same shape
this design arrived at independently: schema-declared traits, SoA stores, and a
callback query API. So the question is only whether it can carry what the engine
actually needs from storage.

```bash
node spike/test/koota-eval.ts
```

**Both critical gates pass.** Snapshots round-trip exactly via `getStore`, and two
identical runs stay bit-identical. At 2,000 entities a snapshot costs 0.008 ms, so
per-frame rewind is free.

| 1M entities, 5 trait combos | throughput |
| --------------------------- | ---------- |
| koota `updateEach`          | 9 M/s      |
| koota raw `getStore` loop   | 68 M/s     |
| custom ECS, `each(cb)`      | 68 M/s     |
| custom ECS, raw columns     | 170 M/s    |

Koota's ergonomic API costs **13x** against its own raw store access — a much
wider spread than the custom ECS's 2.5x, and change detection is not the cause
(disabling it changed nothing). Its raw path lands exactly where the custom
ergonomic path does. At the 2,000-entity target scale `updateEach` costs 0.18 ms,
so none of this decides anything on speed.

**Koota wins on structural change**: 3.3 M ops/s against the custom archetype
move's 1.6 M/s, exactly as a sparse entity-indexed layout should.

### Two measurement traps worth recording

Both produced wrong numbers before being caught, and both are properties of Koota
worth knowing:

- **Entity values pack a generation.** After a recycle an entity reads as
  `1048578`, not `2`. The store is indexed by `unpackEntity(e).entityId`. Using the
  raw value as an index silently builds a huge sparse array and drops V8 into
  dictionary mode — measured 1 M/s instead of 68 M/s, with no error.
- **Entity ids are allocated per process, not per world.** A world created after
  several others starts at a high id, so its stores carry a large empty prefix and
  degrade the same way. Benchmarks must build their world first.

### The blocker: no typed arrays

`getStore` returns plain `Array`s. There is no `Float32Array`, `ArrayBuffer` or
`SharedArrayBuffer` anywhere in koota's public types, and
[multi-threading](https://github.com/pmndrs/koota/issues/91) is an open initiative
at 0 of 10 items since April 2025 — its first step is literally "implementing
buffers within the trait schema".

Plain arrays cannot be shared with a worker. So adopting Koota today forecloses
the threading path until that initiative lands, which is the one requirement that
makes typed-array storage non-negotiable.

**Verdict: keep custom typed-array storage, steal Koota's API design.** Relations
with data (`relation({ store: {...} })`, `autoDestroy`, exclusive, ordered) are the
biggest gap in the current design and exactly what a contraption game needs. The
`updateEach` / `useStores` tiering is the same conclusion spike 02 reached by
measurement. If the threading requirement were dropped, Koota would be the better
choice on every other axis.

## Caveat on codegen

The generated cursor uses `new Function`, which a strict CSP blocks. Since system
declarations are static, the robust path is generating cursors at build time from
the component registry rather than at runtime.

## Still open

- GC pause _distribution_ under a realistic system mix — throughput is settled,
  frame-time consistency is not.
- Whether the 1.41x holds once queries carry change-detection and optional
  components.
