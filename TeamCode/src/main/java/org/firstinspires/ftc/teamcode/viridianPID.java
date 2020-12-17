package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/*
This class implements a PID controller.
 */
public class viridianPID {

    ElapsedTime time = new ElapsedTime();

    private double[] coeff = {0.02, 0.02, 0.01};

    public double expected, actual;
    private double error, errorOld, derivativeError, ToIerror = 0, integralError = 0;
    private double lastTime;

    public viridianPID() {
        expected = 0;
        actual = 0;
        time.reset();
    }
    // Use pointers
    public viridianPID(double expected, double actual) {
        this.expected = expected;
        this.actual = actual;
        time.reset();
    }
    public void setCoeff(double[] coeff) {
        this.coeff = coeff;
    }
    public double calculateError() {
        // Calculate P = error
        error = (expected-actual)/actual;
        // Calculate D = derivative of error = dE/dt
        double currentTime = time.milliseconds();
        derivativeError = (error-errorOld)/(currentTime - lastTime);
        errorOld = error;
        // Calculate I = integral of error = (by Euler's method) I_old + error*dt
        ToIerror = currentTime;
        integralError += error*(lastTime-currentTime);
        lastTime = currentTime;
        // Calculate total = sum of (coeff * P, I, D)
        return coeff[0]*error + coeff[1]*integralError + coeff[2]*derivativeError;
    }
}
