package com.SCHSRobotics.HAL9001.util.math.units;

public enum HALAccelerationUnit {

    MILLIMETERS_PER_SECOND_SQUARED(HALDistanceUnit.MILLIMETERS, HALTimeUnit.SECONDS),
    MILLIMETERS_PER_MILLISECOND_SQUARED(HALDistanceUnit.MILLIMETERS, HALTimeUnit.MILLISECONDS),
    MILLIMETERS_PER_NANOSECOND_SQUARED(HALDistanceUnit.MILLIMETERS, HALTimeUnit.NANOSECONDS),

    MEGAMETERS_PER_SECOND_SQUARED(HALDistanceUnit.MEGAMETERS, HALTimeUnit.SECONDS),
    MEGAMETERS_PER_MILLISECOND_SQUARED(HALDistanceUnit.MEGAMETERS, HALTimeUnit.MILLISECONDS),
    MEGAMETERS_PER_NANOSECOND_SQUARED(HALDistanceUnit.MEGAMETERS, HALTimeUnit.NANOSECONDS),

    CENTIMETERS_PER_SECOND_SQUARED(HALDistanceUnit.CENTIMETERS, HALTimeUnit.SECONDS),
    CENTIMETERS_PER_MILLISECOND_SQUARED(HALDistanceUnit.CENTIMETERS, HALTimeUnit.MILLISECONDS),
    CENTIMETERS_PER_NANOSECOND_SQUARED(HALDistanceUnit.CENTIMETERS, HALTimeUnit.NANOSECONDS),

    METERS_PER_SECOND_SQUARED(HALDistanceUnit.METERS, HALTimeUnit.SECONDS),
    METERS_PER_MILLISECOND_SQUARED(HALDistanceUnit.METERS, HALTimeUnit.MILLISECONDS),
    METERS_PER_NANOSECOND_SQUARED(HALDistanceUnit.METERS, HALTimeUnit.NANOSECONDS),

    INCHES_PER_SECOND_SQUARED(HALDistanceUnit.INCHES, HALTimeUnit.SECONDS),
    INCHES_PER_MILLISECOND_SQUARED(HALDistanceUnit.INCHES, HALTimeUnit.MILLISECONDS),
    INCHES_PER_NANOSECOND_SQUARED(HALDistanceUnit.INCHES, HALTimeUnit.NANOSECONDS),

    FEET_PER_SECOND_SQUARED(HALDistanceUnit.FEET, HALTimeUnit.SECONDS),
    FEET_PER_MILLISECOND_SQUARED(HALDistanceUnit.FEET, HALTimeUnit.MILLISECONDS),
    FEET_PER_NANOSECOND_SQUARED(HALDistanceUnit.FEET, HALTimeUnit.NANOSECONDS),

    FOOTS_PER_SECOND_SQUARED(HALDistanceUnit.FOOTS, HALTimeUnit.SECONDS),
    FOOTS_PER_MILLISECOND_SQUARED(HALDistanceUnit.FOOTS, HALTimeUnit.MILLISECONDS),
    FOOTS_PER_NANOSECOND_SQUARED(HALDistanceUnit.FOOTS, HALTimeUnit.NANOSECONDS),

    YARDS_PER_SECOND_SQUARED(HALDistanceUnit.YARDS, HALTimeUnit.SECONDS),
    YARDS_PER_MILLISECOND_SQUARED(HALDistanceUnit.YARDS, HALTimeUnit.MILLISECONDS),
    YARDS_PER_NANOSECOND_SQUARED(HALDistanceUnit.YARDS, HALTimeUnit.NANOSECONDS),

    MILES_PER_SECOND_SQUARED(HALDistanceUnit.MILES, HALTimeUnit.SECONDS),
    MILES_PER_MILLISECOND_SQUARED(HALDistanceUnit.MILES, HALTimeUnit.MILLISECONDS),
    MILES_PER_NANOSECOND_SQUARED(HALDistanceUnit.MILES, HALTimeUnit.NANOSECONDS),

    TILES_PER_SECOND_SQUARED(HALDistanceUnit.TILES, HALTimeUnit.SECONDS),
    TILES_PER_MILLISECOND_SQUARED(HALDistanceUnit.TILES, HALTimeUnit.MILLISECONDS),
    TILES_PER_NANOSECOND_SQUARED(HALDistanceUnit.TILES, HALTimeUnit.NANOSECONDS);

    public final HALDistanceUnit distanceUnit;
    public final HALTimeUnit timeUnit;
    public final String abbreviation;

    HALAccelerationUnit(HALDistanceUnit distanceUnit, HALTimeUnit timeUnit) {
        this.distanceUnit = distanceUnit;
        this.timeUnit = timeUnit;
        abbreviation = distanceUnit.abbreviation + '/' + timeUnit.abbreviation + "^2";
    }

    public static double convert(double input, HALAccelerationUnit fromUnit, HALAccelerationUnit toUnit) {
        double distanceConversion = HALDistanceUnit.convert(input, fromUnit.distanceUnit, toUnit.distanceUnit);
        double timeConversion1 = HALTimeUnit.convert(1.0 / distanceConversion, fromUnit.timeUnit, toUnit.timeUnit);
        double timeConversion2 = HALTimeUnit.convert(timeConversion1, fromUnit.timeUnit, toUnit.timeUnit);
        return 1.0 / timeConversion2;
    }
}
