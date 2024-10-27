package org.firstinspires.ftc.teamcode.PID;

public class PIDController {
    private final double Kp;
    // todo: integral & derivative
    private double output;

    public PIDController(double P) {
        this.Kp = P;
    }

    public double calculate(double current, double target) {
        double error = target - current;
        double proportional = Kp * error;

        output = proportional;
        return proportional;
    }

}
