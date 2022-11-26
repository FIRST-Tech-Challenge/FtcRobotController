package com.acmerobotics.roadrunner.path.heading

import com.acmerobotics.roadrunner.path.ParametricCurve

/**
 * Interpolator for specifying the heading for holonomic paths.
 */
abstract class HeadingInterpolator {
    /**
     * Base parametric curve
     */
    protected lateinit var curve: ParametricCurve

    /**
     * Initialize the interpolator with a [curve].
     *
     *  @param curve parametric curve
     */
    open fun init(curve: ParametricCurve) {
        this.curve = curve
    }

    /**
     * Returns the heading at the specified [s].
     */
    @JvmOverloads
    operator fun get(s: Double, t: Double = curve.reparam(s)) = internalGet(s, t)

    /**
     * Returns the heading derivative at the specified [s].
     */
    @JvmOverloads
    fun deriv(s: Double, t: Double = curve.reparam(s)) = internalDeriv(s, t)

    /**
     * Returns the heading second derivative at the specified [s].
     */
    @JvmOverloads
    fun secondDeriv(s: Double, t: Double = curve.reparam(s)) = internalSecondDeriv(s, t)

    /**
     * Returns the start heading.
     */
    fun start() = get(0.0, 0.0)

    /**
     * Returns the start heading derivative.
     */
    fun startDeriv() = deriv(0.0, 0.0)

    /**
     * Returns the start heading second derivative.
     */
    fun startSecondDeriv() = secondDeriv(0.0, 0.0)

    /**
     * Returns the end heading.
     */
    fun end() = get(curve.length(), 1.0)

    /**
     * Returns the end heading derivative.
     */
    fun endDeriv() = deriv(curve.length(), 1.0)

    /**
     * Returns the end heading second derivative.
     */
    fun endSecondDeriv() = secondDeriv(curve.length(), 1.0)

    internal abstract fun internalGet(s: Double, t: Double): Double
    internal abstract fun internalDeriv(s: Double, t: Double): Double
    internal abstract fun internalSecondDeriv(s: Double, t: Double): Double
}
