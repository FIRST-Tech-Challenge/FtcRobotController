@file:Suppress("SpellCheckingInspection")

package ftc.rogue.blacksmith.units

enum class DistanceUnit(private val inchConversionFactor: Double) {
    CENTIMETERS   (inchConversionFactor = 1.0 / 2.54),
    INCHES        (inchConversionFactor = 1.0),
    METERS        (inchConversionFactor = 1.0 / 0.0254),
    FEET          (inchConversionFactor = 12.0),
    YARDS         (inchConversionFactor = 36.0),
    MILES         (inchConversionFactor = 63360.0),
    KILOMETERS    (inchConversionFactor = 39370.0),
    NAUTICAL_MILES(inchConversionFactor = 72913.4),
    LIGHT_YEARS   (inchConversionFactor = 5.878e+25),
    PARSECS       (inchConversionFactor = 1.97e+26),
    ANGSTROMS     (inchConversionFactor = 1.0 / 2.54e-8),
    FURLONGS      (inchConversionFactor = 7920.0),
    FERMATS       (inchConversionFactor = 1.0 / 2.54e-15),
    SMOOTS        (inchConversionFactor = 1 / 67.0),
    AUS           (inchConversionFactor = 1.0 / 0.00000484813681109536),
    FATHOMS       (inchConversionFactor = 6.0 * 12.0),
    HANDS         (inchConversionFactor = 4.0 * 12.0),
    LINKS         (inchConversionFactor = 0.01 * 12.0),
    PACES         (inchConversionFactor = 5.0 * 12.0),
    RODS          (inchConversionFactor = 16.5 * 12.0),
    SPANS         (inchConversionFactor = 9.0 * 12.0),
    LEAGUES       (inchConversionFactor = 3.0 * 63360.0);

    fun toIn(x: Number) = x.toDouble() * inchConversionFactor

    fun toCm(x: Number) = x.toDouble() * inchConversionFactor * 2.54

    fun toOtherDistanceUnit(x: Number, other: DistanceUnit) = x.toDouble() * (inchConversionFactor / other.inchConversionFactor)
}
