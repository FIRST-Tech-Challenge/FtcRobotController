package com.SCHSRobotics.HAL9001.util.math.units;

import com.SCHSRobotics.HAL9001.util.functional_interfaces.Function;

/**
 * An angle unit class that allows for easy conversion from and to any angle unit.
 * <p>
 * Creation Date: 5/27/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @since 1.1.0
 */
public enum HALAngleUnit {
    DEGREES, RADIANS;

    /**
     * Gets a function for converting from this angle unit to another angle unit.
     *
     * @param angleUnit The angle unit to convert to.
     * @return A function for converting from this angle unit to another angle unit.
     */
    public Function<Double, Double> convertTo(HALAngleUnit angleUnit) {
        if (this.equals(angleUnit)) return Double::doubleValue;
        else if (angleUnit.equals(RADIANS)) return Math::toRadians;
        else return Math::toDegrees;
    }
}
