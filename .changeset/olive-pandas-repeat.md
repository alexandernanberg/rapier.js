---
"@alexandernanberg/rapier2d": minor
"@alexandernanberg/rapier3d": patch
---

fix: `World.restoreSnapshot` in 2D reports decode failures instead of hiding them

`SerializationPipeline.deserializeAll` is `Option<RawDeserializedWorld>` on the
Rust side — it returns nothing when the snapshot cannot be decoded, which is the
documented outcome of restoring a snapshot taken by a different version of the
engine. 3D propagated that as `World | null`, but 2D forced it away with two
non-null assertions:

```ts
return World.fromRaw(this.raw.deserializeAll(data)!)!;
```

So `World.restoreSnapshot(badSnapshot)` in 2D was typed `World` while actually
evaluating to `null`, and the failure only surfaced later as a confusing
`TypeError` on the first property access.

**Breaking (2D only, types):** `World.restoreSnapshot` and
`SerializationPipeline.deserializeAll` now return `World | null`, matching 3D.
Callers who know their snapshot is good can assert with `!`; everyone else
should branch on the result.

Both dimensions gain a test covering the failure path.
