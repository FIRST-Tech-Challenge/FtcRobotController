package com.technototes.control.gamepad;

import com.technototes.control.Periodic;

import java.util.function.DoubleSupplier;

/** Interface for objects that behave as sticks
 * @author Alex Stedman
 */
public interface Stick extends Periodic {
    /** Return x axis double
     *
     * @return The double
     */
    double getXAxis();
    /** Return y axis double
     *
     * @return The double
     */
    double getYAxis();
    /** Return x axis supplier
     *
     * @return The double supplier
     */
    default DoubleSupplier getXSupplier(){
        return this::getXAxis;
    }
    /** Return y axis supplier
     *
     * @return The double supplier
     */
    default DoubleSupplier getYSupplier(){
        return this::getYAxis;
    }

    /** Returns the angle of the stick
     *
     * @return The angle
     */
    default double getAngle(){
        return -Math.atan2(getYAxis(), getXAxis());
    }

    /** Returns the stick's distance from the center
     *
     * @return The distance
     */
    default double getDistanceFromCenter(){
        return Math.sqrt(getXAxis()*getXAxis()+getYAxis()*getYAxis());
    }
}
