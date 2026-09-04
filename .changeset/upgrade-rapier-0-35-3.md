---
"@alexandernanberg/rapier2d": minor
"@alexandernanberg/rapier3d": minor
---

Upgrade the underlying physics engine from Rapier 0.35.1 to 0.35.3. Both
releases are upstream bug fixes, but two of them change simulation results, so
they are worth calling out.

- Contact impulses (and the `ContactForceEvent`s derived from them) are now the
  total impulse applied over the step, rather than one substep too many. They
  drop by `1 / (numSolverIterations + 1)` and no longer vary with
  `warmstartCoefficient`. Any threshold tuned against the old, inflated values —
  `contactForceEventThreshold` in particular — needs to come down by the same
  factor.
- Multibody joint motors and limits now honor their compliance: the CFM term was
  previously left out of the row's effective mass, which made the velocity gain
  passed to `configureMotorVelocity` (and the position gains) effectively
  infinite, so a motor tracked its target exactly no matter how soft it was
  configured to be. Gains that relied on that now need to be stiffer to settle
  as fast as before. Impulse joint motors are unaffected.

Also fixes multibody instabilities from the semi-implicit Coriolis term when the
mass matrix is near-singular, and a case where an axis carrying both a motor and
a limit emitted more jacobian rows than it reserved.

No binding API changed; the generated WASM glue is byte-identical.
