package org.firstinspires.ftc.teamcodekt.util

import kotlin.math.PI

enum class DistanceUnit(val cf: Double) {
    CM           (cf = 1.0 / 2.54),
    INCHES       (cf = 1.0),
    METER        (cf = 1.0 / 0.0254),
    FEET         (cf = 12.0),
    YARD         (cf = 36.0),
    MILE         (cf = 63360.0),
    KILOMETER    (cf = 39370.0),
    NAUTICAL_MILE(cf = 72913.4),
    LIGHT_YEAR   (cf = 5.878e+25),
    PARSEC       (cf = 1.97e+26),
    ANGSTROM     (cf = 1.0 / 2.54e-8),
    FURLONG      (cf = 7920.0),
    FERMAT       (cf = 1.0 / 2.54e-15),
    SMOOT        (cf = 5.0 * 12.0),
    AU           (cf = 1.0 / 0.00000484813681109536),
    FATHOM       (cf = 6.0 * 12.0),
    HAND         (cf = 4.0 * 12.0),
    LINK         (cf = 0.01 * 12.0),
    PACE         (cf = 5.0 * 12.0),
    ROD          (cf = 16.5 * 12.0),
    SPAN         (cf = 9.0 * 12.0),
    LEAGUE       (cf = 3.0 * 63360.0);

    fun toIn(x: Number) = x.toDouble() * cf
}

enum class AngleUnit(val cf: Double) {
    DEGREES    (cf = 1.0),
    RADIANS    (cf = 180.0 / PI),
    GRADIANS   (cf = 0.9),
    REVOLUTIONS(cf = 360.0),
    ARCSECONDS (cf = 360.0 * 60.0),
    ARCMINUTES (cf = 360.0),
    MILIRADIANS(cf = 180.0 / PI * 1000.0),;

    fun toDegrees(x: Number) = x.toDouble() * cf
}