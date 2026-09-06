/**
 * A rule applied to combine coefficients.
 *
 * Use this when configuring the `ColliderDesc` to specify
 * how friction and restitution coefficient should be combined
 * in a contact.
 */
export enum CoefficientCombineRule {
    /** Average the two values (the default). */
    Average = 0,
    /** Use the smaller value. */
    Min = 1,
    /** Multiply the two values. */
    Multiply = 2,
    /** Use the larger value. */
    Max = 3,
    /** The sum of the two values, clamped to `[0, 1]`. */
    ClampedSum = 4,
    /** The square root of the product of the two values. */
    GeometricMean = 5,
}
