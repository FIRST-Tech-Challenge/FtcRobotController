package org.firstinspires.ftc.teamcode.Utils;

import android.util.Log;

public class AdvancedPiController {
    public double I_OUTPUT = 0;

    private double P;
    private double I;
    private double F;

    private double maxIOutput = 0;
    private double maxErrorSum = 0;
    private double errorSum = 0;

    private double maxInput = Double.MAX_VALUE;
    private double minInput = -Double.MAX_VALUE;

    private double maxOutput = Double.MAX_VALUE;
    private double minOutput = -Double.MAX_VALUE;

    private double setPoint = 0;

    private long lastCalculationStartTime = 0;

    private boolean firstRun = true;
    private boolean reversed = false;

    private double outputRampRate = 0;

    private boolean isContinuous = false;

    private boolean errorIsPositive = true;
    private String pidName;

    /**
     * Create a AdvancedPidController class object.
     * See setP, setI, setD methods for more detailed parameters.
     *
     * @param p Proportional gain. Large if large difference between setpoint and target.
     * @param i Integral gain.  Becomes large if setpoint cannot reach target quickly.
     */
    public AdvancedPiController(double p, double i, double maxIOutput, String pidName) {
        P = p;
        I = i;
        this.pidName = pidName;

        setMaxIOutput(maxIOutput);

        checkSigns();
    }

    public AdvancedPiController(PidValues pidValues, String pidName) {
        P = pidValues.p;
        I = pidValues.i;
        this.pidName = pidName;

        setMaxIOutput(pidValues.maxIOutput);

        checkSigns();
    }

    /**
     * Set the maximum output value contributed by the I component of the system
     * This can be used to prevent large windup issues and make tuning simpler
     *
     * @param maximum Units are the same as the expected output value
     */
    public void setMaxIOutput(double maximum) {
        // Internally maxError and Izone are similar, but scaled for different purposes.
        // The maxError is generated for simplifying math, since calculations against
        // the max error are far more common than changing the I term or Izone.
        maxIOutput = maximum;
        if (I != 0) {
            maxErrorSum = maxIOutput / I;
        }
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
     * Changes the I parameter <br>
     * This is used for overcoming disturbances, and ensuring that the controller always gets to the control mode.
     * Typically tuned second for "Position" based modes, and third for "Rate" or continuous based modes. <br>
     * Affects output through <b>output+=previous_errors*Igain ;previous_errors+=current_error</b>
     *
     * @param i New gain value for the Integral term
     */
    public void setI(double i) {
        if (I != 0) {
            errorSum = errorSum * I / i;
        }
        if (maxIOutput != 0) {
            maxErrorSum = maxIOutput / i;
        }
        I = i;
        checkSigns();
        // Implementation note:
        // This Scales the accumulated error to avoid output errors.
        // As an example doubling the I term cuts the accumulated error in half, which results in the
        // output change due to the I term constant during the transition.
    }

    /**
     * Configure the PID object.
     * See setP, setI, setD methods for more detailed parameters.
     *
     * @param p Proportional gain. Large if large difference between setpoint and target.
     * @param i Integral gain.  Becomes large if setpoint cannot reach target quickly.
     */
    public void setPI(double p, double i) {
        P = p;
        //Note: the I term has additional calculations, so we need to use it's
        //specific method for setting it.
        setI(i);
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
     * @param output the absolute number of maximum output
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

        // Ensure the bounds of the I term are within the bounds of the allowable output swing
        if (maxIOutput == 0 || maxIOutput > (maximum - minimum)) {
            setMaxIOutput(maximum - minimum);
        }
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
        double iOutput;
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
            lastCalculationStartTime = Long.MIN_VALUE;
            firstRun = false;
        }

//        Log.d(pidName, "Input: " + input + "; Last input: " + lastInput);

        // The Iterm is more complex. There's several things to factor in to make it easier to deal with.
        // 1. maxIOutput restricts the amount of output contributed by the Iterm.
        // 2. prevent windup by not increasing errorSum if we're already running against our max Ioutput
        // 3. prevent windup by not increasing errorSum if output is output=maxOutput
        iOutput = I * errorSum;
        if (maxIOutput != 0) {
            iOutput = constrain(iOutput, -maxIOutput, maxIOutput);
        }

        // And, finally, we can just add the terms up
        output = pOutput + iOutput;

        //Calculate F Term
        if (pOutput + iOutput > 0) {
            output += F;
        } else {
            output -= F;
        }


        double IError, absOutPutConstrain;
        if (error > 0 == errorIsPositive) {
            IError = error * calculationInterval * 0.001;
            absOutPutConstrain = getAbsOutPutConstrain(output, calculationInterval);


            // Figure out what we're doing with the error.
            if (!bounded(output, minOutput, maxOutput)) {
                errorSum = IError;
                // reset the error sum to a sane level
                // Setting to current error ensures a smooth transition when the P term
                // decreases enough for the I term to start acting upon the controller
                // From that point the I term will build up as would be expected
            } else if (outputRampRate != 0 && !bounded(output, -absOutPutConstrain, absOutPutConstrain)) {
                errorSum = IError;
            } else if (maxIOutput != 0) {
                errorSum = constrain(errorSum + IError, -maxErrorSum, maxErrorSum);
                // In addition to output limiting directly, we also want to prevent I term
                // buildup, so restrict the error directly
            } else {
                errorSum += IError;
            }

            // Restrict output to our specified output and ramp limits
            if (outputRampRate != 0) {
                Log.d(pidName, "Output ramped;");
                output = constrain(
                        output,
                        -absOutPutConstrain,
                        +absOutPutConstrain
                );
            }
        } else {
            errorIsPositive = error > 0;
            errorSum = 0;
        }

        output = constrain(output, minOutput, maxOutput);

//        Log.d(pidName, String.format("Output: %.4f; P: %.4f; I: %.4f; D: %.4f;", output, pOutput, Ioutput, DOutput));

        lastCalculationStartTime = currentTime;
        I_OUTPUT = iOutput;
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
        errorSum = 0;
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
            if (I > 0) I *= -1;
        } else {  // all values should be above zero
            if (P < 0) P *= -1;
            if (I < 0) I *= -1;
        }
    }
}