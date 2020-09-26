package org.firstinspires.ftc.teamcode.support.hardware;

/**
 * Core interface to be implemented by robot components or hardware devices
 *  that can be configured during robot initialization or runtime.
 * All configurable properties should have public getter / setter methods according
 *  to JavaBeans standard; getter methods for properties adjustable at runtime
 *  should be annotated with <code>@Adjustable</code>.
 * Currently only <code>double</code> type is supported for configurable properties.
 * @see Adjustable
 */
public interface Configurable {

    /**
     * Unique (within robot) name for this component or device.
     * Hardware devices generally should use their name as specified in <code>HardwareMap</code>
     */
    public String getUniqueName();

    /**
     * Turns configuration adjustment mode on or off.
     * When in adjustment mode, changing one of the <code>@Adjustable</code> settings
     *  should cause device to react accordingly (e.g. changing left stop on a servo
     *  should change its position to currently set left stop value)
     * @param on <code>true</code> for turning adjustment mode on
     */
    public void setAdjustmentMode(boolean on);
}
