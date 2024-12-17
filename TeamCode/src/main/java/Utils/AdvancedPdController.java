package org.firstinspires.ftc.teamcode.Utils;

import android.util.Log;

public class AdvancedPdController {

    private double P;
    private double D;
    private double F;

    private double maxInput = Double.MAX_VALUE;
    private double minInput = -Double.MAX_VALUE;

    private double maxOutput = Double.MAX_VALUE;
    private double minOutput = -Double.MAX_VALUE;

    private double setPoint = 0;

    private double lastInput = 0;
    private long lastCalculationStartTime = 0;

    private boolean firstRun = true;
    private boolean reversed = false;

    private double outputRampRate = 0;
    private double lastDOutPut = 0;
    private double lastOutput = 0;

    private double outputFilter = 0;

    private boolean isContinuous = false;

    private boolean errorIsPositive = true;
    private String pidName;

    /**
     * Create a AdvancedPidController class object.
     * See setP, setI, setD methods for more detailed parameters.
     *
     * @param p Proportional gain. Large if large difference between setpoint and target.
     * @param d Derivative gain. Responds quickly to large changes in error. Small values prevents P and I terms from causing overshoot.
     */
    public AdvancedPdController(double p, double d, String pidName) {
        P = p;
        D = d;
        this.pidName = pidName;

        checkSigns();
    }

    /**
     * Configure the Proportional gain parameter. <br>
     * This responds quickly to changes in setpoint, and provides most of the initial driving force
     * to make corrections. <br>
     * Some systems can be used with only a P gain, and many can be operated with only PI.<br>
     * For position based controllers, this is the first parameter to tune, with I second. <br>
     * For rate controlled systems, this is often the second after F.
     *
     * @param p Proportional gain. Affects output according to: output+=P*(setpoint-current_value)
     */
    public void setP(double p) {
        P = p;
        checkSigns();
    }

    /**
     * Changes the D parameter <br>
     * This has two primary effects:
     * <list>
     * <li> Adds a "startup kick" and speeds up system response during setpoint changes
     * <li> Adds "drag" and slows the system when moving toward the target
     * </list>
     * A small D value can be useful for both improving response times, and preventing overshoot.
     * However, in many systems a large D value will cause significant instability, particularly
     * for large setpoint changes.
     * <br>
     * Affects output through <b>output += -D*(current_input_value - last_input_value)</b>
     *
     * @param d New gain value for the Derivative term
     */
    public void setD(double d) {
        D = d;
        checkSigns();
    }

    /**
     * Configure the PID object.
     * See setP, setD methods for more detailed parameters.
     *
     * @param p Proportional gain. Large if large difference between setpoint and target.
     * @param d Derivative gain. Responds quickly to large changes in error. Small values prevents P and I terms from causing overshoot.
     */
    public void setPD(double p, double d) {
        P = p;
        D = d;
        //Note: the I term has additional calculations, so we need to use it's
        //specific method for setting it.
        checkSigns();
    }

    /**
     * Configure the Feedforward parameter. <br>
     * This responds whatever the set point is. <br>
     * For position based controllers, this can be tuned with P. <br>
     * For rate controlled systems, this is often the second after F. <br>
     *
     * @param f Feedforward. Affects output according to: output+=f
     */
    public void setF(double f) {
        F = f;
    }

    /**
     * Specify a maximum output range. <br>
     * When one input is specified, output range is configured to
     * <b>[-output, output]</b>
     *
     * @param output
     */
    public void setOutputLimits(double output) {
        setOutputLimits(-output, output);
    }

    /**
     * Specify a  maximum output.
     * When two inputs specified, output range is configured to
     * <b>[minimum, maximum]</b>
     *
     * @param minimum possible output value
     * @param maximum possible output value
     */
    public void setOutputLimits(double minimum, double maximum) {
        if (maximum < minimum) return;
        maxOutput = maximum;
        minOutput = minimum;
    }

    /**
     * Set the operating direction of the PID controller
     *
     * @param reversed Set true to reverse PID output
     */
    public void setDirection(boolean reversed) {
        this.reversed = reversed;
    }

    /**
     * Configure setPoint for the PID calculations<br>
     * This represents the target for the PID system's, such as a
     * position, velocity, or angle. <br>
     *
     * @param setPoint
     */
    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }

    public void setContinuous(double minInput, double maxInput) {
        this.isContinuous = true;

        this.minInput = minInput;
        this.maxInput = maxInput;
    }

    /**
     * Calculate the output value for the current PID cycle.
     *
     * @param input    The monitored value, typically as a sensor input.
     * @param setPoint The target value for the system. This will be stored for future calculations.
     * @return calculated output value for driving the system
     */
    public double calculate(double input, double setPoint) {
        double pOutput;
        double dOutput;
        double fOutput;
        double output;

        this.setPoint = setPoint;

        long currentTime = System.currentTimeMillis();
        long calculationInterval = currentTime - lastCalculationStartTime;

        // Do the simple parts of the calculations
        double error = setPoint - input;

        // Fix the value if it is continuous
        if (isContinuous) {
            if (Math.abs(error) > (maxInput - minInput) / 2) {
                if (error > 0) {
                    error = error - maxInput + minInput;
                } else {
                    error = error
                            + maxInput - minInput;
                }
            }
        }

        // Calculate P term
        pOutput = P * error;

        // If this is our first time running this, we don't actually _have_ a previous input or output.
        // For sensor, sanely assume it was exactly where it is now.
        // For last output, we can assume it's the current time-independent outputs.
        if (firstRun) {
            lastInput = input;
            lastCalculationStartTime = Long.MIN_VALUE;
            lastOutput = pOutput;
            firstRun = false;
        }

        // Calculate D Term
        // Note, this is negative. This actually "slows" the system if it's doing
        // the correct thing, and small values helps prevent output spikes and overshoot
        if (calculationInterval == 0) {
            return lastOutput;
        } else {
            dOutput = -D * (input - lastInput) / calculationInterval;
        }

        Log.d(pidName, "Input: " + input + "; Last input: " + lastInput);

        lastInput = input;

        //Calculate F Term
        if(pOutput + dOutput > 0){
            fOutput = F;
        } else {
            fOutput = -F;
        }

        // And, finally, we can just add the terms up
        output = pOutput + dOutput + fOutput;

        output = constrain(output, minOutput, maxOutput);

        if (outputFilter != 0) {
            Log.d(pidName, String.format("Original output before filtered: %.4f;", output));
            output = lastOutput * outputFilter + output * (1 - outputFilter);
        }

        Log.d(pidName, String.format("Output: %.4f; P: %.4f; D: %.4f;", output, pOutput, dOutput));

        lastCalculationStartTime = currentTime;
        lastDOutPut = dOutput;
        lastOutput = output;
        return output;
    }

    public double getAbsOutPutConstrain(double output, long calculationInterval) {
        return calculationInterval * outputRampRate + Math.abs(output);
    }

    /**
     * Calculate the output value for the current PID cycle.<br>
     * In one parameter mode, the last configured setpoint will be used.<br>
     *
     * @param actual The monitored value, typically as a sensor input.
     * @return calculated output value for driving the system
     */
    public double calculate(double actual) {
        return calculate(actual, setPoint);
    }

    /**
     * Resets the controller. This erases the I term buildup, and removes
     * D gain on the next loop.<br>
     * This should be used any time the PID is disabled or inactive for extended
     * duration, and the controlled portion of the system may have changed due to
     * external forces.
     */
    public void reset() {
        firstRun = true;
    }

    /**
     * Set the maximum rate the output can increase per cycle.<br>
     * This can prevent sharp jumps in output when changing setpoints or
     * enabling a PID system, which might cause stress on physical or electrical
     * systems.  <br>
     * Can be very useful for fast-reacting control loops, such as ones
     * with large P or D values and feed-forward systems.
     *
     * @param rate, with units being the same as the output
     */
    public void setOutputRampRate(double rate) {
        outputRampRate = rate;
    }

    /**
     * Set a filter on the output to reduce sharp oscillations. <br>
     * 0.1 is likely a sane starting value. Larger values use historical data
     * more heavily, with low values weigh newer data. 0 will disable, filtering, and use
     * only the most recent value. <br>
     * Increasing the filter strength will P and D oscillations, but force larger I
     * values and increase I term overshoot.<br>
     * Uses an exponential weighted rolling sum filter, according to a simple <br>
     * <pre>output*(1-strength)*sum(0..n){output*strength^n}</pre> algorithm.
     *
     * @param strength valid between [0..1), meaning [current output only.. historical output only)
     */
    public void setOutputFilter(double strength) {
        if (strength == 0 || bounded(strength, 0, 1)) {
            outputFilter = strength;
        }
    }

    /**
     * Forces a value into a specific range
     *
     * @param value input value
     * @param min   maximum returned value
     * @param max   minimum value in range
     * @return Value if it's within provided range, min or max otherwise
     */
    private double constrain(double value, double min, double max) {
        if (value > max) {
            return max;
        }
        return Math.max(value, min);
    }

    /**
     * Test if the value is within the min and max, inclusive
     *
     * @param value to test
     * @param min   Minimum value of range
     * @param max   Maximum value of range
     * @return true if value is within range, false otherwise
     */
    private boolean bounded(double value, double min, double max) {
        // Note, this is an inclusive range. This is so tests like
        // `bounded(constrain(0,0,1),0,1)` will return false.
        // This is more helpful for determining edge-case behaviour
        // than <= is.
        return (min < value) && (value < max);
    }

    /**
     * To operate correctly, all PID parameters require the same sign
     * This should align with the {@literal}reversed value
     */
    private void checkSigns() {
        if (reversed) {  // all values should be below zero
            if (P > 0) P *= -1;
            if (D > 0) D *= -1;
        } else {  // all values should be above zero
            if (P < 0) P *= -1;
            if (D < 0) D *= -1;
        }
    }
}