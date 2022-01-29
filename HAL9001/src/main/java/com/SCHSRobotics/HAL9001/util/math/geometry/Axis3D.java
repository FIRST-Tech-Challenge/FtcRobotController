package com.SCHSRobotics.HAL9001.util.math.geometry;

import org.jetbrains.annotations.NotNull;

/**
 * A mathematical class representing a 3D axis.
 * <p>
 * Creation Date: 5/27/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see Axis
 * @see Axis2D
 * @see Vector3D
 * @since 1.1.0
 */
public class Axis3D implements Axis<Vector3D> {
    //Common 3D axes.
    public static final Axis3D
            POSITIVE_X_AXIS = new Axis3D(new Vector3D(1, 0, 0)),
            POSITIVE_Y_AXIS = new Axis3D(new Vector3D(0, 1, 0)),
            POSITIVE_Z_AXIS = new Axis3D(new Vector3D(0, 0, 1)),
            NEGATIVE_X_AXIS = new Axis3D(new Vector3D(-1, 0, 0)),
            NEGATIVE_Y_AXIS = new Axis3D(new Vector3D(0, -1, 0)),
            NEGATIVE_Z_AXIS = new Axis3D(new Vector3D(0, 0, -1));

    //The unit vector going in the direction of the axis.
    private final Vector3D axisUnitVector;

    /**
     * The constructor for a 3D axis.
     *
     * @param axisVector A vector pointing in the direction of the axis.
     */
    public Axis3D(@NotNull Vector3D axisVector) {
        axisUnitVector = axisVector.normalize();
    }

    @Override
    public Vector3D getAxisVector() {
        return axisUnitVector.copy();
    }

    /**
     * Gets the axis unit vector's x component.
     *
     * @return The axis unit vector's x component.
     */
    public double getX() {
        return axisUnitVector.getX();
    }

    /**
     * Gets the axis unit vector's y component.
     *
     * @return The axis unit vector's y component.
     */
    public double getY() {
        return axisUnitVector.getY();
    }

    /**
     * Gets the axis unit vector's z component.
     *
     * @return The axis unit vector's z component.
     */
    public double getZ() {
        return axisUnitVector.getZ();
    }
}