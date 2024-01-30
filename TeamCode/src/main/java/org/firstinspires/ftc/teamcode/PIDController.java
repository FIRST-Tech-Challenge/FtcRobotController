package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    public final double kp;
    public final double ki;
    public final double kd;
    public double pFactor;
    public double iFactor;
    public double dFactor;
    public double integralSum = 0.0;
    public double lastError = 0.0;
    public double previousFilterEstimate = 0.0;
    public double error;
    public double derivative;
    protected ElapsedTime timer;

    /**
     * Construct the PID controller
     * @param p - Proportional coefficient
     * @param i - Integral coefficient
     * @param d - Derivative coefficient
     */
    public PIDController(double p, double i, double d) {
        kp = p;
        ki = i;
        kd = d;
        timer = new ElapsedTime();
    }
    /**
     * Update the PID output
     * @param target where we would like to be, also called the reference
     * @param state where we currently are, I.E. motor position
     */
    public double update (double target, double state) {
        double errorChange;
        double a = 0.10;  // 0.707
        double currentFilterEstimate;
        double result;

        // The sign is backwards because centralOffset is negative of the power we need.
        error = target - state;
        errorChange = error - lastError;

        currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * errorChange;
        previousFilterEstimate = currentFilterEstimate;

        derivative = currentFilterEstimate / timer.seconds();
        integralSum = integralSum + (error * timer.seconds());

        pFactor = kp * error;
        iFactor = ki * integralSum;
        dFactor = kd * derivative;
        result = pFactor + iFactor + dFactor;
        lastError = error;
        timer.reset();

        return result;
    }

    /**
     * Reset the PID for a new target
     */
    public void reset() {
        integralSum = 0.0;
        lastError = 0.0;
        previousFilterEstimate = 0.0;
        timer.reset();
    }
}
