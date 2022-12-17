@file:Suppress("SpellCheckingInspection")

package ftc.rouge.blacksmith.units

enum class DistanceUnit(private val inchConversionFactor: Double) {
    CM           (inchConversionFactor = 1.0 / 2.54),
    INCHES       (inchConversionFactor = 1.0),
    METER        (inchConversionFactor = 1.0 / 0.0254),
    FEET         (inchConversionFactor = 12.0),
    YARD         (inchConversionFactor = 36.0),
    MILE         (inchConversionFactor = 63360.0),
    KILOMETER    (inchConversionFactor = 39370.0),
    NAUTICAL_MILE(inchConversionFactor = 72913.4),
    LIGHT_YEAR   (inchConversionFactor = 5.878e+25),
    PARSEC       (inchConversionFactor = 1.97e+26),
    ANGSTROM     (inchConversionFactor = 1.0 / 2.54e-8),
    FURLONG      (inchConversionFactor = 7920.0),
    FERMAT       (inchConversionFactor = 1.0 / 2.54e-15),
    SMOOT        (inchConversionFactor = 5.0 * 12.0),
    AU           (inchConversionFactor = 1.0 / 0.00000484813681109536),
    FATHOM       (inchConversionFactor = 6.0 * 12.0),
    HAND         (inchConversionFactor = 4.0 * 12.0),
    LINK         (inchConversionFactor = 0.01 * 12.0),
    PACE         (inchConversionFactor = 5.0 * 12.0),
    ROD          (inchConversionFactor = 16.5 * 12.0),
    SPAN         (inchConversionFactor = 9.0 * 12.0),
    LEAGUE       (inchConversionFactor = 3.0 * 63360.0);

    fun toIn(x: Number) = x.toDouble() * inchConversionFactor

    fun toCm(x: Number) = x.toDouble() * inchConversionFactor * 2.54

    fun toOtherDistanceUnit(x: Number, other: DistanceUnit) = x.toDouble() * (inchConversionFactor / other.inchConversionFactor)
}
