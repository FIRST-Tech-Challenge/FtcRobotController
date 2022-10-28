package org.firstinspires.ftc.teamcode.util;

public interface PositionChangeSensor {
    /** Gets the Robot's Estimated State Change.
     *
     * State change is returned in terms of the Robot's motion in its forward direction, strafe
     * (left is positive), and heading change (in radians)
     *
     * @return double[] {forward, strafe, heading}
     */
    public double[] getStateChange();
    /** Gets the Robot's Estimated State Change.
     *
     * State change is returned in terms of the Robot's motion in its forward direction, strafe
     * (left is positive), and heading change (in degrees).
     *
     * @return double[] {forward, strafe, heading}
     */
    public double[] getStateChangeDegrees();
}
