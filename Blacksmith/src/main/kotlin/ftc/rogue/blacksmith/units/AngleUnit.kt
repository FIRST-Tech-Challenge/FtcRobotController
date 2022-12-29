package ftc.rogue.blacksmith.units

import kotlin.math.PI

enum class AngleUnit(private val degreesConversionFactor: Double) {
    DEGREES    (degreesConversionFactor = 1.0),
    RADIANS    (degreesConversionFactor = 180.0 / PI),
    GRADIANS   (degreesConversionFactor = 0.9),
    REVOLUTIONS(degreesConversionFactor = 360.0),
    ARCSECONDS (degreesConversionFactor = 360.0 * 60.0),
    ARCMINUTES (degreesConversionFactor = 360.0),
    MILIRADIANS(degreesConversionFactor = 180.0 / PI * 1000.0);

    fun toDeg(x: Number) = x.toDouble() * degreesConversionFactor

    fun toRad(x: Number) = toDeg(x) * PI / 180

    fun to(unit: AngleUnit, x: Number) = unit.toDeg(x) / degreesConversionFactor
}