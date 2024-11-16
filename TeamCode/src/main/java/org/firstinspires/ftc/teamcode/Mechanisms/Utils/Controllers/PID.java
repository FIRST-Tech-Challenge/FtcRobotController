package org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    public double kP;
    public double kI;
    public double kD;
    public double eIntegralSum;
    public double eDerivative;
    public double ePrev;

    public ElapsedTime timer = new ElapsedTime();

    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        eIntegralSum = 0;
        eDerivative = 0;
        ePrev = 0;
    }

    public double calculate(double target, double currentState) {
        double error = target - currentState;
        double dt = timer.seconds();
        eIntegralSum += (error - ePrev) * dt;
        eDerivative = (error - ePrev) / dt;
        ePrev = error;
        timer.reset();
        return (kP * error) + (kI * eIntegralSum) + (kD * eDerivative);
    }
}
