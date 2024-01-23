package org.firstinspires.ftc.teamcode.subsystems;

public class PID {
    private final double p;
    private final double i;
    private final double d;

    public PID(double kp, double ki, double kd) {
        p = kp;
        i = ki;
        d = kd;
    }

    public int update(double state, double target) {
        double error = target - state;
        int output = Math.toIntExact(Math.round(p * error));
        return output;
    }
}