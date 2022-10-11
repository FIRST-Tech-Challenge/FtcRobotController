package org.firstinspires.ftc.teamcode.PIDs;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    double kp;
    double ki;
    double kd;
    double derivative;
    ElapsedTime timer;

    double lastError = 0;
    double integralSum = 0;
    double integralSumLimit = 0.25; // Recommended amount

    double a = 0.8; // a can be anything from 0 < a < 1
    double previousFilterEstimate = 0;
    double currentFilterEstimate = 0;

    /**
     * construct PID controller
     * @param Kp Proportional coefficient
     * @param Ki Integral coefficient
     * @param Kd Derivative coefficient
     */
    public PIDController(double Kp, double Ki, double Kd, ElapsedTime timer) {
        this.kp = Kp;
        this.ki = Ki;
        this.kd = Kd;
        this.timer = timer;
    }

    /**
     * update the PID controller output
     * @param target where we would like to be, also called the reference
     * @param state where we currently are, I.E. motor position
     * @return the command to our motor, I.E. motor power
     */
    public double update(double target, double state) {
        // PID logic and then return the output
        double error = target - state;

        double errorChange = (error - lastError);

        // filter out high frequency noise to increase derivative performance
        /**
         * This low-pass filter works because the sum of
         * (a * previousFilterEstimate) + (1-a) * errorChange <-- current measurement
         * always equals 1 (like 100%).
         *
         * By changing values of a, you alter how much of 100% each variable takes on.
         * For instance, if a was really small, previousFilterEstimate would be really small,
         * and errorChange would be much larger, and they would both add up to 1
         */
        currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * errorChange;
        previousFilterEstimate = currentFilterEstimate;

        // rate of change of the error
        derivative = currentFilterEstimate / timer.seconds();

        integralSum += error * timer.seconds();

        // The following code sets hard limits on how large our integralSum can become
        if (integralSum > integralSumLimit) {
            integralSum = integralSumLimit;
        }
        if (integralSum < -integralSumLimit) {
            integralSum = -integralSumLimit;
        }

        derivative = (error - lastError) / timer.seconds();
        lastError = error;

        double outputPower = (error * kp) + (derivative * kd) + (integralSum * ki);
        return outputPower;
    }
}