package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SimplePIDControl {
    private final ElapsedTime period = new ElapsedTime();
    public double targetValue = 0;
    public double maxOutput = 0.2;
    // Alpha is the extinction coefficient.  It must be between (0.0, 1.0]
    //   * 0 means that new values are ignored
    //   * 1 means that the previous values don't matter at all (no averaging).
    public double alpha = 0.5;

    // These follow the standard definitions
    public double p;
    public double i;
    public double d;
    public double chillFactor = 1;

    private double lastError = 0;
    private double integratedError = 0.0;
    private double averagedDerivative = 0.0;

    /**
     * This PID Controller drives a motor using PID control as specified.  The function is
     * implemented using the following equation (approximately):
     * <p>
     * TargetPower = pValue * err() + iValue * integratedError + dValue * averagedDerivative
     * <p>
     * Notes:
     *  * integratedError is calculated as a proper integral (error * delta_time)
     *  * averagedDerivative is an exponential weighted moving average (EWMA) of the derivative of error
     *    See <a href="https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average">Wikipedia for EWMA definitions</a>
     * <p>
     * See <a href="https://en.wikipedia.org/wiki/PID_controller">Wikipedia on PID Controllers</a>.
     *
     * @param pValue The value for p
     * @param iValue The value for i
     * @param dValue The value for d
     */
    public SimplePIDControl(double pValue, double iValue, double dValue) {
        this.p = pValue;
        this.d = dValue;
        this.i = iValue;
    }

    /**
     * Clamp a float value between upper and lower bounds.
     *
     * @param val the value
     * @param min lower bound
     * @param max upper bound
     * @return val clamped between min and max.
     */
    public static float clamp(float val, float min, float max) {
        return Math.max(min, Math.min(max, val));
    }

    /**
     * Clamp a double value between upper and lower bounds.
     *
     * @param val the value
     * @param min lower bound
     * @param max upper bound
     * @return val clamped between min and max.
     */
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    /**
     * Call this function inside the loop to update the PID calculations and set the optimal
     * motor power.
     */
    public double update(double measurement, double feedForward) {
        // Calculate how long since the last update and reset the timer.
        double timeStep = period.seconds();
        period.reset();

        // Store thisError because we'll use it several times.
        double thisError = measuredError(measurement);
        // Store thisDerivative because it makes the later equations easier to understand
        double thisDerivative = (thisError - lastError) / timeStep;

        // Only start integrating when we get close so large moves don't over-weight the error.
        if (Math.abs(thisError) < 5) {
            // Use weighted averaging to smooth out the derivative and integral terms.
            integratedError = integratedError + thisError * timeStep;
        } else {
            integratedError = thisError * timeStep;
        }
        averagedDerivative = thisDerivative * alpha + averagedDerivative * (1 - alpha);

        lastError = thisError;

        double powerCalc = p * thisError + i * integratedError + d * averagedDerivative + feedForward;
        return powerCalc;
        /** Calculations for our custom PID control **/
    }

    /**
     * Update the measurement with no feedforward input.
     *
     * @param measurement
     * @return drive power value.
     */
    public double update(double measurement) {
        return update(measurement, 0);
    }

        /**
         * Calculates the encoder angular error.  This is the difference between the current position and
         * the target encoder position divided by the TICKS_PER_DEGREE
         *
         * @return the angle in degrees
         */
    public double measuredError(double measurement) {
        return (targetValue - measurement);
    }

    /**
     * Sets the target angle of the pid controller.  Note: Also resets the integratedError
     *
     * @param targetValue Target angle in degrees
     */
    public void setTargetValue(double targetValue) {
        this.targetValue = targetValue;
        integratedError = 0.0;
    }

    public double getTargetValue() {
        return targetValue;
    }





    /**
     * From <a href="https://en.wikipedia.org/wiki/PID_controller#Standard_versus_parallel_(ideal)_form">Standard vs Parallel form</a>
     * on wikipedia, the integral time is defined as:
     * <p>
     * Kp/Ti = Ki
     *
     * <h3>From Wikipedia for reference:</h3>
     * In this standard form, the parameters have a clear physical meaning. In particular, the inner
     * summation produces a new single error value which is compensated for future and past errors.
     * The proportional error term is the current error. The derivative components term attempts to
     * predict the error value at T_{d} seconds (or samples) in the future, assuming that the loop
     * control remains unchanged. The integral component adjusts the error value to compensate for
     * the sum of all past errors, with the intention of completely eliminating them in T_{i}
     * seconds (or samples). The resulting compensated single error value is then scaled by the
     * single gain K_{p} to compute the control variable.
     *
     * @param time
     */
    public void setIntegralTime(double time) {
        this.i = this.p / time;
    }

    /**
     * From <a href="https://en.wikipedia.org/wiki/PID_controller#Standard_versus_parallel_(ideal)_form">Standard vs Parallel form</a>
     * on wikipedia, the derivative time is defined as:
     * <p>
     * Kp * Td = Kd
     *
     * <h3>From Wikipedia for reference:</h3>
     * In this standard form, the parameters have a clear physical meaning. In particular, the inner
     * summation produces a new single error value which is compensated for future and past errors.
     * The proportional error term is the current error. The derivative components term attempts to
     * predict the error value at T_{d} seconds (or samples) in the future, assuming that the loop
     * control remains unchanged. The integral component adjusts the error value to compensate for
     * the sum of all past errors, with the intention of completely eliminating them in T_{i}
     * seconds (or samples). The resulting compensated single error value is then scaled by the
     * single gain K_{p} to compute the control variable.
     *
     * @param time
     */
    public void setDerivativeTime(double time) {
        this.d = this.p * time;
    }

}
/**yaeoaeaoeoeaeoaeoaeoaoeaeoeet*/