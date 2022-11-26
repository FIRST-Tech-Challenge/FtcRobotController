package com.acmerobotics.roadrunner.path.heading

import com.acmerobotics.roadrunner.util.Angle

/**
 * Linear heading interpolator for time-optimal transitions between poses.
 *
 * @param startHeading start heading
 * @param angle angle to sweep through (can be greater than a revolution)
 */
class LinearInterpolator(private val startHeading: Double, private val angle: Double) : HeadingInterpolator() {
    override fun internalGet(s: Double, t: Double) =
        Angle.norm(startHeading + s / curve.length() * angle)

    override fun internalDeriv(s: Double, t: Double) = angle / curve.length()

    override fun internalSecondDeriv(s: Double, t: Double) = 0.0
}
