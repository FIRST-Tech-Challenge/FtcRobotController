package com.SCHSRobotics.HAL9001.util.math.units;

/**
 * An enum representing different common units of distance.
 * <p>
 * Creation Date: 7/19/19
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @since 1.0.0
 */
public enum HALDistanceUnit {
    MILLIMETERS(0.001, "mm"),
    MEGAMETERS(MILLIMETERS.conversionFactor, "MM"),
    CENTIMETERS(0.01, "cm"),
    METERS(1.0, "m"),
    INCHES(0.0254, "in"),
    FEET(0.3048, "ft"),
    FOOTS(FEET.conversionFactor, FEET.abbreviation),
    YARDS(0.9144, "yd"),
    MILES(1609.34, "mi"),
    TILES(0.6096, "Ti");

    //The number that you multiply by to get meters.
    public double conversionFactor;
    //Their common abbreviation.
    public String abbreviation;

    /**
     * Constructor for Units.
     *
     * @param meterConversion The conversion factor used to convert that unit to meters.
     * @param abbreviation    The abbreviation of the unit.
     */
    HALDistanceUnit(double meterConversion, String abbreviation) {
        conversionFactor = meterConversion;
        this.abbreviation = abbreviation;
    }

    /**
     * Converts from one unit of distance to another.
     *
     * @param input    The value to convert.
     * @param fromUnit The unit of the value to convert.
     * @param toUnit   The unit to convert to.
     * @return The converted value.
     */
    public static double convert(double input, HALDistanceUnit fromUnit, HALDistanceUnit toUnit) {
        double meters = input * fromUnit.conversionFactor;
        return meters / toUnit.conversionFactor;
    }
}
