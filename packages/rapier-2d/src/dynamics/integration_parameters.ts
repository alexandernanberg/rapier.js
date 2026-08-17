import {RawIntegrationParameters} from "../raw";

export class IntegrationParameters {
    raw: RawIntegrationParameters;

    constructor(raw?: RawIntegrationParameters) {
        this.raw = raw || new RawIntegrationParameters();
    }

    /**
     * Free the WASM memory used by these integration parameters.
     */
    public free() {
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined!;
    }

    /**
     * The timestep length (default: `1.0 / 60.0`)
     */
    get dt(): number {
        return this.raw.dt;
    }

    /**
     * The Error Reduction Parameter in `[0, 1]` is the proportion of
     * the positional error to be corrected at each time step (default: `0.2`).
     */
    get contact_erp(): number {
        return this.raw.contact_erp;
    }

    /**
     * The natural frequency (in Hz) of the spring-like contact constraints (default: `30.0`).
     */
    get contactNaturalFrequency(): number {
        return this.raw.contactNaturalFrequency;
    }

    /**
     * The damping ratio of the spring-like contact constraints (default: `5.0`).
     */
    get contactDampingRatio(): number {
        return this.raw.contactDampingRatio;
    }

    /**
     * The natural frequency (in Hz) of the spring-like contact constraints where one
     * of the two colliders is attached to a fixed body (default: `2.0 * contactNaturalFrequency`).
     */
    get staticContactNaturalFrequency(): number {
        return this.raw.staticContactNaturalFrequency;
    }

    /**
     * The damping ratio of the spring-like contact constraints where one of the two
     * colliders is attached to a fixed body (default: `5.0`).
     */
    get staticContactDampingRatio(): number {
        return this.raw.staticContactDampingRatio;
    }

    /**
     * The coefficient in `[0, 1]` applied to the impulses of the previous step when
     * warm-starting the solver (default: `1.0`).
     */
    get warmstartCoefficient(): number {
        return this.raw.warmstartCoefficient;
    }

    /**
     * Are impulse-joint constraints warm-started like contacts? (default: `true`).
     */
    get warmstartJoints(): boolean {
        return this.raw.warmstartJoints;
    }

    /**
     * The minimum timestep size when using CCD with multiple substeps (default: `dt / 100.0`).
     *
     * When CCD with multiple substeps is enabled, the timestep is subdivided into smaller
     * pieces. This timestep subdivision won’t generate timestep lengths smaller than this value.
     */
    get minCcdDt(): number {
        return this.raw.minCcdDt;
    }

    get lengthUnit(): number {
        return this.raw.lengthUnit;
    }

    /**
     * Normalized amount of penetration the engine won’t attempt to correct (default: `0.005m`).
     *
     * This threshold considered by the physics engine is this value multiplied by the `lengthUnit`.
     */
    get normalizedAllowedLinearError(): number {
        return this.raw.normalizedAllowedLinearError;
    }

    /**
     * The maximal normalized velocity at which the penetration of two objects is corrected
     * by the biased solver pass (default: `10.0`).
     *
     * This threshold considered by the physics engine is this value multiplied by the `lengthUnit`.
     */
    get normalizedMaxCorrectiveVelocity(): number {
        return this.raw.normalizedMaxCorrectiveVelocity;
    }

    /**
     * The maximal normalized distance separating two objects that will generate predictive contacts (default: `0.02`).
     *
     * This threshold considered by the physics engine is this value multiplied by the `lengthUnit`.
     */
    get normalizedPredictionDistance(): number {
        return this.raw.normalizedPredictionDistance;
    }

    /**
     * The maximal normalized linear velocity a rigid-body can have after each solver
     * substep (default: `100.0`).
     *
     * This threshold considered by the physics engine is this value multiplied by the `lengthUnit`.
     */
    get normalizedMaxLinearVelocity(): number {
        return this.raw.normalizedMaxLinearVelocity;
    }

    /**
     * The number of solver iterations run by the constraints solver for calculating forces (default: `4`).
     */
    get numSolverIterations(): number {
        return this.raw.numSolverIterations;
    }

    /**
     * Number of internal Project Gauss Seidel (PGS) iterations run at each solver iteration (default: `1`).
     */
    get numInternalPgsIterations(): number {
        return this.raw.numInternalPgsIterations;
    }

    /**
     * The number of stabilization iterations run at each solver iteration (default: `2`).
     */
    get numInternalStabilizationIterations(): number {
        return this.raw.numInternalStabilizationIterations;
    }

    /**
     * Maximum number of substeps performed by the  solver (default: `1`).
     */
    get maxCcdSubsteps(): number {
        return this.raw.maxCcdSubsteps;
    }

    /**
     * Is contact-manifold clustering enabled? (default: `true`).
     *
     * Clustering merges the contacts of a manifold into fewer solver contacts, which is
     * faster to solve but slightly less accurate.
     */
    get contactClustering(): boolean {
        return this.raw.contactClustering;
    }

    /**
     * Is contact recycling enabled? (default: `true`).
     *
     * When enabled, a contact pair skips its full narrow-phase update as long as the
     * relative pose of its two colliders barely moved since the last update.
     */
    get contactRecycling(): boolean {
        return this.raw.contactRecycling;
    }

    /**
     * The maximal normalized relative-pose drift below which a contact pair can be
     * recycled instead of being recomputed (default: `0.002`).
     *
     * This threshold considered by the physics engine is this value multiplied by the `lengthUnit`.
     * Only has an effect if `contactRecycling` is enabled.
     */
    get normalizedContactRecycleDistance(): number {
        return this.raw.normalizedContactRecycleDistance;
    }

    /**
     * Is friction resolved during the biased solver pass? (default: `false`).
     */
    get frictionInBiasPass(): boolean {
        return this.raw.frictionInBiasPass;
    }

    set dt(value: number) {
        this.raw.dt = value;
    }

    /**
     * Sets the natural frequency (in Hz) of the spring-like contact constraints.
     *
     * This is an alias of {@link IntegrationParameters.contactNaturalFrequency}, kept for
     * compatibility with the upstream bindings.
     */
    set contact_natural_frequency(value: number) {
        this.raw.contact_natural_frequency = value;
    }

    set contactNaturalFrequency(value: number) {
        this.raw.contactNaturalFrequency = value;
    }

    set contactDampingRatio(value: number) {
        this.raw.contactDampingRatio = value;
    }

    set staticContactNaturalFrequency(value: number) {
        this.raw.staticContactNaturalFrequency = value;
    }

    set staticContactDampingRatio(value: number) {
        this.raw.staticContactDampingRatio = value;
    }

    /**
     * Sets the coefficient in `[0, 1]` applied to the impulses of the previous step when
     * warm-starting the solver. Setting this to `0.0` disables warm-starting.
     */
    set warmstartCoefficient(value: number) {
        this.raw.warmstartCoefficient = value;
    }

    set warmstartJoints(value: boolean) {
        this.raw.warmstartJoints = value;
    }

    set minCcdDt(value: number) {
        this.raw.minCcdDt = value;
    }

    set lengthUnit(value: number) {
        this.raw.lengthUnit = value;
    }

    set normalizedAllowedLinearError(value: number) {
        this.raw.normalizedAllowedLinearError = value;
    }

    set normalizedMaxCorrectiveVelocity(value: number) {
        this.raw.normalizedMaxCorrectiveVelocity = value;
    }

    set normalizedPredictionDistance(value: number) {
        this.raw.normalizedPredictionDistance = value;
    }

    set normalizedMaxLinearVelocity(value: number) {
        this.raw.normalizedMaxLinearVelocity = value;
    }

    /**
     * Sets the number of solver iterations run by the constraints solver for calculating forces (default: `4`).
     */
    set numSolverIterations(value: number) {
        this.raw.numSolverIterations = value;
    }

    /**
     * Sets the number of internal Project Gauss Seidel (PGS) iterations run at each solver iteration (default: `1`).
     */
    set numInternalPgsIterations(value: number) {
        this.raw.numInternalPgsIterations = value;
    }

    /**
     * Sets the number of stabilization iterations run at each solver iteration (default: `2`).
     */
    set numInternalStabilizationIterations(value: number) {
        this.raw.numInternalStabilizationIterations = value;
    }

    set maxCcdSubsteps(value: number) {
        this.raw.maxCcdSubsteps = value;
    }

    set contactClustering(value: boolean) {
        this.raw.contactClustering = value;
    }

    set contactRecycling(value: boolean) {
        this.raw.contactRecycling = value;
    }

    set normalizedContactRecycleDistance(value: number) {
        this.raw.normalizedContactRecycleDistance = value;
    }

    set frictionInBiasPass(value: boolean) {
        this.raw.frictionInBiasPass = value;
    }
}
