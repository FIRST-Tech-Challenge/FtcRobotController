package org.firstinspires.ftc.teamcode.util;


public class AnglePIDControl extends SimplePIDControl {
    public double wrapAngle = 360;
    public double angleError;

    public AnglePIDControl(double p, double i, double d, double wrapAngle){
        super(p,i,d);
        this.wrapAngle = wrapAngle;
    }

    @Override
    public double update(double measurement) {
        return super.update(measurement);
    }

    /**
     * Compute the measured error, accounting for the fact that angles wrap at 360 degrees!
     *
     * See https://www.desmos.com/calculator/yflwv0vx1n to understand this function.
     *
     * @param measurement The measured value
     * @return an error value between -wrapAngle/2 and wrapAngle/2
     */
    @Override
    public double measuredError(double measurement) {
        double error = super.measuredError(measurement);
        angleError = ( ( error + 3*wrapAngle/2 ) % wrapAngle ) - wrapAngle/2;
        return angleError;
    }
}
