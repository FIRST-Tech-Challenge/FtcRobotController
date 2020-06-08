package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Controllables.Controllable;

public class PIDFController {

    private Controllable last;
    private Controllable current;
    private Controllable target;

    private double kP;
    private double kI;
    private double kD;
    private double kV;
    private double kA;

    private double iSum;

    public PIDFController(double kp, double ki, double kd, Controllable start, Controllable target)
    {
        last = start;
        current = start;
        this.target = target;

        kP = kp;
        kI = ki;
        kD = kd;
        kV = 0;
        kA = 0;

        iSum = 0;
    }

    public PIDFController(double kp, double ki, double kd, double kv, double ka, Controllable start, Controllable target)
    {
        last = start;
        current = start;
        this.target = target;

        kP = kp;
        kI = ki;
        kD = kd;
        kV = kv;
        kA = ka;

        iSum = 0;
    }

    public double getOutputPID(Controllable current)
    {
        double error = current.getVal() - target.getVal();
        iSum += error;
        double derivative = (current.getVal() - last.getVal()) / (current.getTime() - last.getTime());

        double pTerm = error * kP;
        double iTerm = iSum * kI;
        double dTerm = derivative * kD;

        return pTerm + iTerm + dTerm;
    }

    public double getOutputPIDF(Controllable current, Controllable currentTarget)
    {
        double error = current.getVal() - target.getVal();
        iSum += error;
        double derivative = (current.getVal() - last.getVal()) / (current.getTime() - last.getTime());

        double pTerm = kP * error;
        double iTerm = kI * iSum;
        double dTerm = kD * derivative;
        double vTerm = kV * currentTarget.getVelocity();
        double aTerm = kA * currentTarget.getAcceleration();

        return pTerm + iTerm + dTerm + vTerm + aTerm;
    }

}
