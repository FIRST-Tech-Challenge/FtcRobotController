package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

public class PIDControllerTurret extends PIDController{
    public final double kStatic;
    public final double kStaticDeadZone;

    /**
     * Construct the PID controller
     * @param p - Proportional coefficient
     * @param i - Integral coefficient
     * @param d - Derivative coefficient
     * @param pStatic - Minimum power to start turret motion
     * @param pStaticDeadZone - The value close enough to target to cut off minimum power
     */
    public PIDControllerTurret(double p, double i, double d, double pStatic, double pStaticDeadZone) {
        super(p, i, d);
        kStatic = pStatic;
        kStaticDeadZone = pStaticDeadZone;
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
        if(abs(state) <= kStaticDeadZone) {
            kpMinValue = 0.0;
        } else {
            kpMinValue = Math.signum(-state) * kStatic;
        }

        return pidValue + kpMinValue;
    }
}
