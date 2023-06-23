package org.firstinspires.ftc.teamcode.commandBased.classes.poofypid;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PoofyPIDController extends PoofyFeedForwardController{

    private double kP;
    private double kI;
    private double kD;

    private boolean hasRun;
    private final ElapsedTime timer = new ElapsedTime();

    private double target;
    private double previousError;
    private double integralSum;
    private double derivative;

    public PoofyPIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    @Override
    public double calculate(double measuredPosition) {
        double dt = getDT();
        double error = calculateError(target, measuredPosition);
        calculateDerivative(error, dt);
        integrate(error, dt);
        previousError = error;
        double output = error * kP
                      + integralSum * kI
                      + derivative * kD;
        return output;
    }

    @Override
    public void setTargetPosition(double targetPosition) {
        target = targetPosition;
    }


    public void setP(double kP) {
        this.kP = kP;
    }

    public void setI(double kI) {
        this.kI = kI;
    }

    public void setD(double kD) {
        this.kD = kD;
    }

    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }


    private double getDT() {
        if (!hasRun) {
            hasRun = true;
            timer.reset();
        }
        double dt = timer.seconds();
        timer.reset();
        return dt;
    }

    private double calculateError(double reference, double state) {
        return reference - state;
    }

    private void integrate(double error, double dt) {
        integralSum += ((error + previousError) / 2) * dt;
    }

    private void calculateDerivative(double error, double dt) {
        derivative = (error - previousError) / dt;
    }

}
