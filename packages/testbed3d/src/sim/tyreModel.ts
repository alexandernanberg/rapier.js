/**
 * A brush/Pacejka-style tyre model.
 *
 * This is the piece the built-in Rapier vehicle controller does not have. Its
 * tyre is linear right up to a hard Coulomb clamp, so grip never *peaks* and
 * never progressively falls away — which is exactly the part of "feel" a driver
 * reads to find (and catch) the limit.
 *
 * Here instead:
 *
 *  - grip follows the Magic Formula shape: it rises with slip, peaks a little
 *    past the linear region, then falls away to a lower sliding plateau;
 *  - longitudinal and lateral slip share one budget (combined slip), so power
 *    genuinely trades against cornering;
 *  - the friction coefficient drops as vertical load rises (load sensitivity),
 *    which is what makes weight transfer change the handling balance.
 *
 * Everything here is pure maths on plain numbers: no physics engine, no state.
 */

/** Pacejka "Magic Formula" shape coefficients. */
export interface TyreCurve {
    /** Stiffness factor: how quickly grip builds with slip. */
    B: number;
    /** Shape factor: sets the height of the sliding plateau after the peak. */
    C: number;
    /** Curvature factor: shifts where the peak sits. */
    E: number;
}

export interface TyreModelOptions {
    curve?: TyreCurve;
    /** Peak friction coefficient at the nominal load. */
    peakFriction?: number;
    /** Load (N) at which `peakFriction` applies. */
    nominalLoad?: number;
    /**
     * How much the friction coefficient sags as load rises above nominal.
     * `0` = load-independent (unrealistic), `0.3` ≈ a road tyre.
     */
    loadSensitivity?: number;
    /** Speed (m/s) below which slip is regularised to avoid a divide-by-zero. */
    minSpeed?: number;
    /**
     * Relaxation length (m): the distance the tyre must roll for its force to
     * build up to the steady-state value.
     *
     * Real carcasses deform before they grip, so force lags a steering input by
     * roughly half a wheel revolution. Modelling that lag is what gives the car
     * its transient feel — and it also keeps the (very stiff) slip curve from
     * ringing at low speed, where a tiny sideways velocity would otherwise ask
     * for a huge instantaneous force.
     */
    relaxationLength?: number;
}

export const DEFAULT_TYRE: Required<TyreModelOptions> = {
    // C alone sets the sliding plateau, at sin(C * pi/2) of the peak: 1.45
    // gives ~0.76, so a slide costs about a quarter of the grip -- enough to
    // feel the limit let go, gentle enough to be catchable.
    //
    // The peak then sits where the inner term reaches tan(pi / 2C), which B
    // and E place. B = 40 puts it at a normalised slip of ~0.15, i.e. a peak
    // slip ratio of ~0.18 and a peak slip angle of ~8.5 degrees -- both in the
    // right range for a road tyre.
    curve: {B: 40.0, C: 1.45, E: 0.9},
    peakFriction: 1.5,
    nominalLoad: 4000,
    loadSensitivity: 0.3,
    minSpeed: 1.0,
    relaxationLength: 0.4,
};

/**
 * First-order lag factor for one timestep of tyre relaxation.
 *
 * Returns the fraction of the way the tyre's state should move towards its
 * steady-state value: `0` = frozen, `1` = instantaneous.
 */
export function relaxationFactor(
    speed: number,
    dt: number,
    o: Required<TyreModelOptions> = DEFAULT_TYRE,
): number {
    if (o.relaxationLength <= 0) return 1;
    // Distance rolled this step, as a fraction of the relaxation length. A
    // floor keeps the tyre responsive when creeping or stationary.
    const travelled = Math.max(Math.abs(speed), o.minSpeed * 0.25) * dt;
    return Math.min(1, travelled / o.relaxationLength);
}

/**
 * The Magic Formula, normalised so its peak is exactly `1`.
 *
 * `sin(C * atan(...))` reaches 1 when the inner term hits `pi / (2C)`, so the
 * peak value is independent of `B`/`E` — they only move where the peak sits and
 * how fast it falls off afterwards.
 */
export function magicFormula(slip: number, curve: TyreCurve = DEFAULT_TYRE.curve): number {
    const {B, C, E} = curve;
    const Bs = B * slip;
    return Math.sin(C * Math.atan(Bs - E * (Bs - Math.atan(Bs))));
}

/**
 * The friction coefficient at a given vertical load.
 *
 * Real tyres are *less* efficient the harder you press them: doubling the load
 * gives less than double the grip. That single fact is why load transfer
 * changes a car's balance, and why a stiffer anti-roll bar on one axle makes
 * that axle lose grip first.
 */
export function frictionAtLoad(load: number, o: Required<TyreModelOptions>): number {
    if (load <= 0) return 0;
    const ratio = load / o.nominalLoad;
    return o.peakFriction * (1 - o.loadSensitivity * (ratio - 1));
}

/** The slip state of a rolling wheel. */
export interface SlipState {
    /** Slip ratio: `(omega * r - vLong) / |vLong|`. Positive = wheelspin. */
    slipRatio: number;
    /** Slip angle (radians): how far the contact patch is sliding sideways. */
    slipAngle: number;
}

/**
 * Slip ratio and slip angle from the contact-patch velocity and wheel spin.
 *
 * Both are regularised by `minSpeed` so they stay finite at a standstill.
 */
export function computeSlip(
    vLong: number,
    vLat: number,
    wheelOmega: number,
    radius: number,
    o: Required<TyreModelOptions> = DEFAULT_TYRE,
): SlipState {
    const ref = Math.max(Math.abs(vLong), o.minSpeed);
    return {
        slipRatio: (wheelOmega * radius - vLong) / ref,
        slipAngle: Math.atan2(vLat, ref),
    };
}

export interface TyreForces {
    /** Longitudinal force (N): positive drives the car forwards. */
    fx: number;
    /** Lateral force (N): opposes sideways sliding. */
    fy: number;
    /** Combined normalised slip magnitude (0 = rolling true, >~0.2 = sliding). */
    slip: number;
    /** The load-sensitive friction coefficient actually used. */
    friction: number;
}

/**
 * Combined-slip tyre forces.
 *
 * Longitudinal and lateral slip are normalised into a single slip vector, the
 * Magic Formula is evaluated **once** on its magnitude, and the resulting force
 * is shared between the two axes in proportion. That is what produces a proper
 * friction *circle*: a tyre already saturated putting power down has nothing
 * left for cornering, so throttle genuinely costs you grip.
 */
export function tyreForces(
    slipState: SlipState,
    load: number,
    options: Required<TyreModelOptions> = DEFAULT_TYRE,
): TyreForces {
    if (load <= 0) return {fx: 0, fy: 0, slip: 0, friction: 0};

    const {slipRatio, slipAngle} = slipState;
    const denom = 1 + Math.abs(slipRatio);
    const sx = slipRatio / denom;
    const sy = Math.tan(slipAngle) / denom;
    const slip = Math.hypot(sx, sy);

    const friction = frictionAtLoad(load, options);
    if (slip < 1e-6) return {fx: 0, fy: 0, slip: 0, friction};

    const magnitude = friction * load * magicFormula(slip, options.curve);

    return {
        // Positive slip ratio (wheel outrunning the road) drives the car on.
        fx: magnitude * (sx / slip),
        // Lateral force always opposes the direction of sideways slip.
        fy: -magnitude * (sy / slip),
        slip,
        friction,
    };
}

/**
 * The normalised slip at which the curve peaks — the "edge of grip".
 *
 * Found by sampling, because it depends on all three coefficients. Handy for
 * traction/stability control and for tests.
 */
export function peakSlip(curve: TyreCurve = DEFAULT_TYRE.curve): number {
    let best = 0;
    let bestValue = -Infinity;
    for (let s = 0.001; s <= 1.5; s += 0.001) {
        const value = magicFormula(s, curve);
        if (value > bestValue) {
            bestValue = value;
            best = s;
        }
    }
    return best;
}
