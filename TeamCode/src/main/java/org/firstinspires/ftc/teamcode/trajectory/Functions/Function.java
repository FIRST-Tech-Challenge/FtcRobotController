package org.firstinspires.ftc.teamcode.trajectory.Functions;

/** Serves as a function for motion planning
 * @author TheConverseEngineer
 */
public interface Function {

    /** Returns the point x in this function
     * @param x     The x value
     * @return      The y value
     */
    double getPoint(double x);

    /** Returns the derivatice of point x in this function
     * @param x     The x value
     * @return      The derivative at that  value
     */
    double getDeriv(double x);
}
