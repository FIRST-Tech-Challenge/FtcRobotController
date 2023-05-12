package org.firstinspires.ftc.teamcode.classes.pid;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDControllerEx {

    PIDCoeffs pidCoeffs;

    boolean hasRun = false;

    ElapsedTime timer = new ElapsedTime();

    double previousError;

    double integralSum;

    double derivative;

    public PIDControllerEx(PIDCoeffs coefficients) {
        this.pidCoeffs = coefficients;
    }

    public double calculate(double reference, double state) {
        double dt = getDT();
        double error = calculateError(reference, state);
        double derivative = calculateDerivative(error, dt);
        integrate(error, dt);
        previousError = error;
        return error * pidCoeffs.Kp
                + integralSum * pidCoeffs.Ki
                + derivative * pidCoeffs.Kd;
    }

    public double getDT() {
        if (!hasRun) {
            hasRun = true;
            timer.reset();
        }
        double dt = timer.seconds();
        timer.reset();
        return dt;
    }

    protected double calculateError(double reference, double state) {
        return reference - state;
    }

    protected void integrate(double error, double dt) {
        integralSum += ((error + previousError) / 2) * dt;
    }

    protected double calculateDerivative(double error, double dt) {
        derivative = (error - previousError) / dt;
        return derivative;
    }
}
