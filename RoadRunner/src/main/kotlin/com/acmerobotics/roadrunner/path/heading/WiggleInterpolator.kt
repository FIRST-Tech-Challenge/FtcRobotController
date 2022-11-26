package com.acmerobotics.roadrunner.path.heading

import com.acmerobotics.roadrunner.path.ParametricCurve
import com.acmerobotics.roadrunner.path.QuinticPolynomial
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

private const val K = 0.5 // fraction of a period replaced by a path on either side

/**
 * Heading interpolator that wraps another interpolator and adds sinusoidal oscillations ("wiggles") while preserving
 * continuity. More specifically, the wiggle function is composed of a sine wave with a quintic spline on either end.
 *
 * @param amplitude amplitude of the wiggle oscillations
 * @param desiredPeriod period fo the wiggle oscillations
 * @param baseInterpolator base interpolator to add oscillations to (e.g., oscillations relative to the tangent)
 */
class WiggleInterpolator(
    private val amplitude: Double,
    private val desiredPeriod: Double,
    private val baseInterpolator: HeadingInterpolator = TangentInterpolator()
) : HeadingInterpolator() {

    private var period: Double = 0.0
    private lateinit var beginSpline: QuinticPolynomial
    private lateinit var endSpline: QuinticPolynomial

    override fun init(curve: ParametricCurve) {
        super.init(curve)

        baseInterpolator.init(curve)

        val n = (curve.length() / desiredPeriod).toInt()
        period = 1.0 / n

        val t1 = K * period
        val t2 = 1.0 - t1

        beginSpline = QuinticPolynomial(
            0.0,
            0.0,
            0.0,
            waveGet(t1),
            waveDeriv(t1) * (K * period),
            waveSecondDeriv(t1) * (K * K * period * period)
        )

        endSpline = QuinticPolynomial(
            waveGet(t2),
            waveDeriv(t2) * (1 - K * period),
            waveSecondDeriv(t2) * ((1 - K * period) * (1 - K * period)),
            0.0,
            0.0,
            0.0
        )
    }

    private fun waveGet(t: Double) = amplitude * sin(2.0 * PI * t / period)

    private fun waveDeriv(t: Double) = 2.0 * PI * amplitude / period * cos(2.0 * PI * t / period)

    private fun waveSecondDeriv(t: Double) = 4.0 * PI * PI * amplitude / (period * period) *
            sin(2.0 * PI * t / period)

    override fun internalGet(s: Double, t: Double): Double {
        val heading = when {
            t < K * period ->
                beginSpline[t / (K * period)]
            t > (1.0 - K * period) ->
                endSpline[t / (1 - K * period) - 1.0]
            else ->
                waveGet(t)
        }

        return heading + baseInterpolator[s, t]
    }

    override fun internalDeriv(s: Double, t: Double): Double {
        val headingDeriv = when {
            t < K * period ->
                beginSpline.deriv(t / (K * period)) / (K * period)
            t > (1.0 - K * period) ->
                endSpline.deriv(t / (1 - K * period) - 1.0) / (1 - K * period)
            else ->
                waveDeriv(t)
        }

        return headingDeriv * curve.paramDeriv(t) + baseInterpolator.deriv(s, t)
    }

    override fun internalSecondDeriv(s: Double, t: Double): Double {
        val headingDeriv = when {
            t < K * period ->
                beginSpline.deriv(t / (K * period)) / (K * period)
            t > (1.0 - K * period) ->
                endSpline.deriv(t / (1 - K * period) - 1.0) / (1 - K * period)
            else ->
                waveDeriv(t)
        }

        val headingSecondDeriv = when {
            t < K * period ->
                beginSpline.secondDeriv(t / (K * period)) / (K * K * period * period)
            t > (1.0 - K * period) ->
                endSpline.secondDeriv(t / (1 - K * period) - 1.0) / ((1 - K * period) * (1 - K * period))
            else ->
                waveSecondDeriv(t)
        }

        return headingSecondDeriv * curve.paramDeriv(t) * curve.paramDeriv(t) +
                headingDeriv * curve.paramSecondDeriv(t) + baseInterpolator.secondDeriv(s, t)
    }
}
