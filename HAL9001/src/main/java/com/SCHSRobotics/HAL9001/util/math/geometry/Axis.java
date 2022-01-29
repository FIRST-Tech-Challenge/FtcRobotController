package com.SCHSRobotics.HAL9001.util.math.geometry;

/**
 * The base interface for all axes classes.
 * <p>
 * Creation Date: 9/30/20
 *
 * @param <V> The type of vector associated with the axis.
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see Axis2D
 * @see Axis3D
 * @see VectorND
 * @see Vector2D
 * @see Vector3D
 * @since 1.1.0
 */
public interface Axis<V extends Vector<V>> {
    /**
     * Gets the unit vector associated with this axis.
     *
     * @return The unit vector associated with this axis.
     * @see VectorND
     * @see Vector2D
     * @see Vector3D
     */
    V getAxisVector();
}
