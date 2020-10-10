package com.technototes.control.gamepad;

import com.technototes.control.Periodic;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** The class to extend custom gamepad axis from
 * @author Alex Stedman
 */
public class GamepadAxis extends GamepadButton implements DoubleSupplier, Periodic {
    public static final double defaultTriggerThreshold = 0.5;
    public double triggerThreshold;
    public DoubleSupplier doubleSupplier;

    /** Make a GamepadAxis with the supplier
     *
     * @param d The supplier to make the axis around
     */
    public GamepadAxis(DoubleSupplier d){
        new GamepadAxis(d, defaultTriggerThreshold);
    }
    /** Make a GamepadAxis with the supplier and the threshold for the stick to behave as a button
     *
     * @param d The supplier to make the axis around
     * @param t The threshold
     */
    public GamepadAxis(DoubleSupplier d, double t){
        super(() -> Math.abs(d.getAsDouble())>t);
        doubleSupplier = d;
        triggerThreshold = t;
    }

    public GamepadAxis(){
        triggerThreshold = defaultTriggerThreshold;
    }

    /** Set the supplier for the axis
     *
     * @param d The double supplier
     * @return This
     */
    public GamepadButton setSupplier(DoubleSupplier d) {
        super.setSupplier(() -> Math.abs(d.getAsDouble())>triggerThreshold);
        doubleSupplier = d;
        return this;
    }

    /** Returns the double from the axis
     *
     * @return The double
     */
    @Override
    public double getAsDouble() {
        return doubleSupplier.getAsDouble();
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
