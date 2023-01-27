package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

public class PIDControllerTurret extends PIDController{
    public final double kpMin;
    public final double kpMinDeadZone;

    /**
     * Construct the PID controller
     * @param p - Proportional coefficient
     * @param i - Integral coefficient
     * @param d - Derivative coefficient
     * @param pMin - Minimum power to start turret motion
     * @param pMinDeadZone - The value close enough to target to cut off minimum power
     */
    public PIDControllerTurret(double p, double i, double d, double pMin, double pMinDeadZone) {
        super(p, i, d);
        kpMin = pMin;
        kpMinDeadZone = pMinDeadZone;
    }
    /**
     * Update the PID output
     * @param target where we would like to be, also called the reference
     * @param state where we currently are, I.E. motor position
     */
    public double update (double target, double state) {
        double kpMinValue;
        double pidValue = super.update(target, state);
        // The turret requires a non-zero minimum power just to start moving
        // Augment the PID result with that power (unless we're inside our
        // tolerance, in which case we want to drop to zero power and stop)
        if(abs(state) <= kpMinDeadZone) {
            kpMinValue = 0.0;
        } else if(state > 0) {
            kpMinValue = -kpMin;
        } else {
            kpMinValue = +kpMin;
        }

        return pidValue + kpMinValue;
    }
}
