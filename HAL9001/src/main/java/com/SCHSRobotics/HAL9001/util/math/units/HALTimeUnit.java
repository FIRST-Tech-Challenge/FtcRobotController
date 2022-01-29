package com.SCHSRobotics.HAL9001.util.math.units;

/**
 * An enum representing common units of time.
 * <p>
 * Creation Date:
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see com.SCHSRobotics.HAL9001.util.misc.Timer
 * @since 1.1.0
 */
public enum HALTimeUnit {
    NANOSECONDS(1, "ns"),
    MILLISECONDS(1e6, "ms"),
    SECONDS(MILLISECONDS.nanoConversionFactor * 1000, "s");

    public final String abbreviation;
    //The conversion factor to convert 1 unit of this time into nanoseconds.
    private final double nanoConversionFactor;

    /**
     * The constructor for HALTimeUnit.
     *
     * @param nanoConversionFactor The amount of nanoseconds in one unit of this time unit.
     */
    HALTimeUnit(double nanoConversionFactor, String abbreviation) {
        this.nanoConversionFactor = nanoConversionFactor;
        this.abbreviation = abbreviation;
    }

    /**
     * Converts from one time unit to another.
     *
     * @param timeIn   The time input value.
     * @param fromUnit The time input unit.
     * @param toUnit   The desired unit of time.
     * @return The converted time.
     */
    public static double convert(double timeIn, HALTimeUnit fromUnit, HALTimeUnit toUnit) {
        double timeInNanos = timeIn * fromUnit.nanoConversionFactor;
        return timeInNanos / toUnit.nanoConversionFactor;
    }

    /**
     * Converts from one time unit to another.
     *
     * @param timeIn   The time input value.
     * @param fromUnit The time input unit.
     * @param toUnit   The desired unit of time.
     * @return The converted time.
     */
    public static double convert(long timeIn, HALTimeUnit fromUnit, HALTimeUnit toUnit) {
        return convert((double) timeIn, fromUnit, toUnit);
    }
}
