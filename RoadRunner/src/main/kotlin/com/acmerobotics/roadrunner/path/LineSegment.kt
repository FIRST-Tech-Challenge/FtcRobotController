package com.acmerobotics.roadrunner.path

import com.acmerobotics.roadrunner.geometry.Vector2d

/**
 * Parametric representation of a line.
 *
 * @param start start vector
 * @param end end vector
 */
class LineSegment(private val start: Vector2d, end: Vector2d) : ParametricCurve() {
    private val diff = end - start

    override fun length() = diff.norm()

    override fun internalGet(t: Double) = start + diff * t

    override fun internalDeriv(t: Double) = diff

    override fun internalSecondDeriv(t: Double) = Vector2d(0.0, 0.0)

    override fun internalThirdDeriv(t: Double) = Vector2d(0.0, 0.0)

    override fun reparam(s: Double) = s / length()

    override fun paramDeriv(t: Double) = 1.0 / length()

    override fun paramSecondDeriv(t: Double) = 0.0

    override fun paramThirdDeriv(t: Double) = 0.0

    override fun toString() = "(${start.x}+${diff.x}*t,${start.y}+${diff.y}*t)"
}
