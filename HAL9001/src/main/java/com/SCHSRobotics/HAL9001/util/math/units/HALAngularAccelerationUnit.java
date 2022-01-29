package com.SCHSRobotics.HAL9001.util.math.units;

public enum HALAngularAccelerationUnit {
    RADIANS_PER_SECOND_SQUARED(HALAngleUnit.RADIANS, HALTimeUnit.SECONDS),
    RADIANS_PER_MILLISECOND_SQUARED(HALAngleUnit.RADIANS, HALTimeUnit.MILLISECONDS),
    RADIANS_PER_NANOSECOND_SQUARED(HALAngleUnit.RADIANS, HALTimeUnit.NANOSECONDS),

    DEGREES_PER_SECOND_SQUARED(HALAngleUnit.DEGREES, HALTimeUnit.SECONDS),
    DEGREES_PER_MILLISECOND_SQUARED(HALAngleUnit.DEGREES, HALTimeUnit.MILLISECONDS),
    DEGREES_PER_NANOSECOND_SQUARED(HALAngleUnit.DEGREES, HALTimeUnit.NANOSECONDS);

    private final HALAngleUnit angleUnit;
    private final HALTimeUnit timeUnit;

    HALAngularAccelerationUnit(HALAngleUnit angleUnit, HALTimeUnit timeUnit) {
        this.angleUnit = angleUnit;
        this.timeUnit = timeUnit;
    }

    public static double convert(double input, HALAngularAccelerationUnit fromUnit, HALAngularAccelerationUnit toUnit) {
        double angleConversion = fromUnit.angleUnit.convertTo(toUnit.angleUnit).apply(input);
        double timeConversion1 = HALTimeUnit.convert(1.0 / angleConversion, fromUnit.timeUnit, toUnit.timeUnit);
        double timeConversion2 = HALTimeUnit.convert(timeConversion1, fromUnit.timeUnit, toUnit.timeUnit);
        return 1.0 / timeConversion2;
    }
}
