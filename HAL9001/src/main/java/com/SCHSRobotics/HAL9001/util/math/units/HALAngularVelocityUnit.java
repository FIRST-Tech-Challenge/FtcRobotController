package com.SCHSRobotics.HAL9001.util.math.units;

public enum HALAngularVelocityUnit {
    RADIANS_PER_SECOND(HALAngleUnit.RADIANS, HALTimeUnit.SECONDS),
    RADIANS_PER_MILLISECOND(HALAngleUnit.RADIANS, HALTimeUnit.MILLISECONDS),
    RADIANS_PER_NANOSECOND(HALAngleUnit.RADIANS, HALTimeUnit.NANOSECONDS),

    DEGREES_PER_SECOND(HALAngleUnit.DEGREES, HALTimeUnit.SECONDS),
    DEGREES_PER_MILLISECOND(HALAngleUnit.DEGREES, HALTimeUnit.MILLISECONDS),
    DEGREES_PER_NANOSECOND(HALAngleUnit.DEGREES, HALTimeUnit.NANOSECONDS);

    private final HALAngleUnit angleUnit;
    private final HALTimeUnit timeUnit;

    HALAngularVelocityUnit(HALAngleUnit angleUnit, HALTimeUnit timeUnit) {
        this.angleUnit = angleUnit;
        this.timeUnit = timeUnit;
    }

    public static double convert(double input, HALAngularVelocityUnit fromUnit, HALAngularVelocityUnit toUnit) {
        double angleConversion = fromUnit.angleUnit.convertTo(toUnit.angleUnit).apply(input);
        double timeConversion = HALTimeUnit.convert(1.0 / angleConversion, fromUnit.timeUnit, toUnit.timeUnit);
        return 1.0 / timeConversion;
    }
}
