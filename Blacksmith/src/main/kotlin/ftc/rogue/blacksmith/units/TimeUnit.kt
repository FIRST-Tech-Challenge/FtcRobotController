package ftc.rogue.blacksmith.units

enum class TimeUnit(private val msConversionFactor: Double) {
    NANOSECONDS (msConversionFactor = 1.0 / 1e+6),
    MICROSECONDS(msConversionFactor = 1.0 / 1e+3),
    MILLISECONDS(msConversionFactor = 1.0),
    SECONDS     (msConversionFactor = 1000.0),
    MINUTES     (msConversionFactor = 60000.0),
    HOURS       (msConversionFactor = 3600000.0),
    DAYS        (msConversionFactor = 86400000.0),
    WEEKS       (msConversionFactor = 604800000.0),
    MONTHS      (msConversionFactor = 2.628e+9),
    YEARS       (msConversionFactor = 3.154e+10),
    CENTURIES   (msConversionFactor = 3.154e+11),
    MILLENNIA   (msConversionFactor = 3.154e+12);

    fun toSec(x: Number) = x.toDouble() * msConversionFactor.toInt() / 1000.0

    fun toMs(x: Number) = x.toInt() * msConversionFactor.toInt()

    fun to(unit: TimeUnit, x: Number) = unit.toMs(x) / msConversionFactor
}
