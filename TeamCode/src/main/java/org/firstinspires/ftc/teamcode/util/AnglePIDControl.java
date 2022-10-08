package org.firstinspires.ftc.teamcode.util;


public class AnglePIDControl extends SimplePIDControl {
    public double wrapAngle;

    public AnglePIDControl(double p, double i, double d, double wrapAngle){
        super(p,i,d);
        this.wrapAngle = wrapAngle;
    }

    @Override
    public double measuredError(double measurement) {
        double error = super.measuredError(measurement);
        return  ( ( error + 3*wrapAngle/2 ) % wrapAngle ) - wrapAngle/2;
    }
}
