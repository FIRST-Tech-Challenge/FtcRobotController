package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

public class PIDControllerLift extends PIDController {
    public final double kg;

    /**
     * Construct the PID controller
     * @param p - Proportional coefficient
     * @param i - Integral coefficient
     * @param d - Derivative coefficient
     * @param g - The power required to hold lift against gravity
     */
    public PIDControllerLift(double p, double i, double d, double g) {
        super(p, i, d);
        kg = g;
    }
    /**
     * Update the PID output
     * @param target where we would like to be, also called the reference
     * @param state where we currently are, I.E. motor position
     */
    public double update (double target, double state) {
        return super.update(target, state) + kg;
    }
}
