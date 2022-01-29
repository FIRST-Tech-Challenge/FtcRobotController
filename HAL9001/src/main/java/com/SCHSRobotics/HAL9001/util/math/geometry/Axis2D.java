package com.SCHSRobotics.HAL9001.util.math.geometry;

import org.jetbrains.annotations.NotNull;

/**
 * A mathematical class representing a 2D axis.
 * <p>
 * Creation Date: 9/30/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see Axis
 * @see Axis3D
 * @see Vector2D
 * @since 1.1.0
 */
public class Axis2D implements Axis<Vector2D> {
    //Common 2D axes.
    public static final Axis2D
            POSITIVE_X_AXIS = new Axis2D(new Vector2D(1, 0)),
            POSITIVE_Y_AXIS = new Axis2D(new Vector2D(0, 1)),
            NEGATIVE_X_AXIS = new Axis2D(new Vector2D(-1, 0)),
            NEGATIVE_Y_AXIS = new Axis2D(new Vector2D(0, -1));

    //The unit vector going in the direction of the axis.
    private final Vector2D axisUnitVector;

    /**
     * The constructor for a 2D axis.
     *
     * @param axisVector A vector pointing in the direction of the axis.
     */
    public Axis2D(@NotNull Vector2D axisVector) {
        axisUnitVector = axisVector.normalize();
    }

    @Override
    public Vector2D getAxisVector() {
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
}