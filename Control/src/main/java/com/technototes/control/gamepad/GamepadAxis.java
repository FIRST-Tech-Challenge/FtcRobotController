package com.technototes.control.gamepad;

import com.technototes.control.Periodic;

import java.util.function.DoubleSupplier;

/** The class to extend custom gamepad axis from
 * @author Alex Stedman
 */
public class GamepadAxis extends GamepadButton implements DoubleSupplier, Periodic {
    /** The default trigger threshold
     *
     */
    public static final double DEFAULT_TRIGGER_THRESHOLD = 0.5;
    private double triggerThreshold;
    protected DoubleSupplier doubleSupplier;

    /** Make a GamepadAxis with the supplier
     *
     * @param d The supplier to make the axis around
     */
    public GamepadAxis(DoubleSupplier d){
        new GamepadAxis(d, DEFAULT_TRIGGER_THRESHOLD);
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

    protected GamepadAxis(){
        triggerThreshold = DEFAULT_TRIGGER_THRESHOLD;
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

    /** Gets the trigger threshold
     *
     * @return The threshold
     */
    public double getTriggerThreshold() {
        return triggerThreshold;
    }

    /** Set threshold
     * @param threshold the new threshold
     */
    public void setTriggerThreshold(double threshold){
        triggerThreshold = threshold;
    }
}
