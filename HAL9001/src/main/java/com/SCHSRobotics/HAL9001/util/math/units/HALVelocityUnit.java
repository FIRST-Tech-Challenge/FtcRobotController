package com.SCHSRobotics.HAL9001.util.math.units;

public enum HALVelocityUnit {

    MILLIMETERS_PER_SECOND(HALDistanceUnit.MILLIMETERS, HALTimeUnit.SECONDS),
    MILLIMETERS_PER_MILLISECOND(HALDistanceUnit.MILLIMETERS, HALTimeUnit.MILLISECONDS),
    MILLIMETERS_PER_NANOSECOND(HALDistanceUnit.MILLIMETERS, HALTimeUnit.NANOSECONDS),

    MEGAMETERS_PER_SECOND(HALDistanceUnit.MEGAMETERS, HALTimeUnit.SECONDS),
    MEGAMETERS_PER_MILLISECOND(HALDistanceUnit.MEGAMETERS, HALTimeUnit.MILLISECONDS),
    MEGAMETERS_PER_NANOSECOND(HALDistanceUnit.MEGAMETERS, HALTimeUnit.NANOSECONDS),

    CENTIMETERS_PER_SECOND(HALDistanceUnit.CENTIMETERS, HALTimeUnit.SECONDS),
    CENTIMETERS_PER_MILLISECOND(HALDistanceUnit.CENTIMETERS, HALTimeUnit.MILLISECONDS),
    CENTIMETERS_PER_NANOSECOND(HALDistanceUnit.CENTIMETERS, HALTimeUnit.NANOSECONDS),

    METERS_PER_SECOND(HALDistanceUnit.METERS, HALTimeUnit.SECONDS),
    METERS_PER_MILLISECOND(HALDistanceUnit.METERS, HALTimeUnit.MILLISECONDS),
    METERS_PER_NANOSECOND(HALDistanceUnit.METERS, HALTimeUnit.NANOSECONDS),

    INCHES_PER_SECOND(HALDistanceUnit.INCHES, HALTimeUnit.SECONDS),
    INCHES_PER_MILLISECOND(HALDistanceUnit.INCHES, HALTimeUnit.MILLISECONDS),
    INCHES_PER_NANOSECOND(HALDistanceUnit.INCHES, HALTimeUnit.NANOSECONDS),

    FEET_PER_SECOND(HALDistanceUnit.FEET, HALTimeUnit.SECONDS),
    FEET_PER_MILLISECOND(HALDistanceUnit.FEET, HALTimeUnit.MILLISECONDS),
    FEET_PER_NANOSECOND(HALDistanceUnit.FEET, HALTimeUnit.NANOSECONDS),

    FOOTS_PER_SECOND(HALDistanceUnit.FOOTS, HALTimeUnit.SECONDS),
    FOOTS_PER_MILLISECOND(HALDistanceUnit.FOOTS, HALTimeUnit.MILLISECONDS),
    FOOTS_PER_NANOSECOND(HALDistanceUnit.FOOTS, HALTimeUnit.NANOSECONDS),

    YARDS_PER_SECOND(HALDistanceUnit.YARDS, HALTimeUnit.SECONDS),
    YARDS_PER_MILLISECOND(HALDistanceUnit.YARDS, HALTimeUnit.MILLISECONDS),
    YARDS_PER_NANOSECOND(HALDistanceUnit.YARDS, HALTimeUnit.NANOSECONDS),

    MILES_PER_SECOND(HALDistanceUnit.MILES, HALTimeUnit.SECONDS),
    MILES_PER_MILLISECOND(HALDistanceUnit.MILES, HALTimeUnit.MILLISECONDS),
    MILES_PER_NANOSECOND(HALDistanceUnit.MILES, HALTimeUnit.NANOSECONDS),

    TILES_PER_SECOND(HALDistanceUnit.TILES, HALTimeUnit.SECONDS),
    TILES_PER_MILLISECOND(HALDistanceUnit.TILES, HALTimeUnit.MILLISECONDS),
    TILES_PER_NANOSECOND(HALDistanceUnit.TILES, HALTimeUnit.NANOSECONDS);

    public final HALDistanceUnit distanceUnit;
    public final HALTimeUnit timeUnit;
    public final String abbreviation;

    HALVelocityUnit(HALDistanceUnit distanceUnit, HALTimeUnit timeUnit) {
        this.distanceUnit = distanceUnit;
        this.timeUnit = timeUnit;
        abbreviation = distanceUnit.abbreviation + '/' + timeUnit.abbreviation;
    }

    public static double convert(double input, HALVelocityUnit fromUnit, HALVelocityUnit toUnit) {
        double distanceConversion = HALDistanceUnit.convert(input, fromUnit.distanceUnit, toUnit.distanceUnit);
        double timeConversion = HALTimeUnit.convert(1.0 / distanceConversion, fromUnit.timeUnit, toUnit.timeUnit);
        return 1.0 / timeConversion;
    }
}
