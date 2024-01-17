package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.Mechanisms;

public class PID {
    private final double p;
    private final double i;
    private final double d;

    public PID (double kp, double ki, double kd) {
        p = kp;
        i = ki;
        d = kd;
    }

    public double update(double state, double target) {
        double error = target - state;
        double output = (p * error);
        return output;
    }
}