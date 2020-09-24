/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package com.hfrobots.tnt.corelib.drive;

import android.util.Log;

import com.hfrobots.tnt.corelib.Constants;

// A simplification of Titan Robotic's PID controller
public class PidController {

    private final String instanceName;
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double tolerance;
    private long settlingTimeMs;

    private boolean inverted = false;
    private boolean absSetPoint = false;
    private boolean noOscillation = false;
    private double minTarget = 0.0;
    private double maxTarget = 0.0;
    private double minOutput = -1.0;
    private double maxOutput = 1.0;

    private double prevTime = 0.0;
    private double currError = 0.0;
    private double totalError = 0.0;
    private double settlingStartTimeMs = 0.0;
    private double setPoint = 0.0;
    private double setPointSign = 1.0;
    private double output = 0.0;

    private double pTerm;
    private double iTerm;
    private double dTerm;
    private double fTerm;

    public static Builder builder() {
        return new Builder();
    }

    public static class Builder {
        private String       instanceName;
        private double       kP;
        private double       kI;
        private double       kD;
        private double       kF;
        private double       tolerance;
        private long         settlingTimeMs;
        private boolean      allowOscillation = false;

        /**
         * @param instanceName the name used in debug logging and telemetry output
         */
        public Builder setInstanceName(String instanceName) {
            this.instanceName = instanceName;

            return this;
        }

        /**
         * @param kP specifies the proportional constant.
         */
        public Builder setKp(double kP) {
            this.kP = kP;

            return this;
        }

        /**
         * @param kI specifies the integral constant.
         */
        public Builder setkI(double kI) {
            this.kI = kI;

            return this;
        }

        /**
         * @param kD specifies the differential constant.
         */
        public Builder setkD(double kD) {
            this.kD = kD;

            return this;
        }

        /**
         * @param kF specifies the feed forward constant.
         */
        public Builder setkF(double kF) {
            this.kF = kF;

            return this;
        }

        /**
         * @param tolerance specifies the target tolerance.
         */
        public Builder setTolerance(double tolerance) {
            this.tolerance = tolerance;

            return this;
        }

        /**
         * @param settlingTimeMs specifies the minimum on target settling time.
         */
        public Builder setSettlingTimeMs(long settlingTimeMs) {
            this.settlingTimeMs = settlingTimeMs;

            return this;
        }

        public Builder setAllowOscillation(boolean flag) {
            this.allowOscillation = flag;

            return this;
        }

        public PidController build() {
            if (instanceName == null) {
                instanceName = "unnamed";
            }

            PidController built = new PidController(instanceName);
            built.kP = Math.abs(kP);
            built.kI = Math.abs(kI);
            built.kD = Math.abs(kD);
            built.kF = Math.abs(kF);
            built.tolerance = Math.abs(tolerance);
            built.settlingTimeMs = Math.abs(settlingTimeMs);
            built.setNoOscillation(!allowOscillation);

            return built;
        }
    }

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    private PidController(final String instanceName) {
        this.instanceName = instanceName;
    }

    /**
     * This method calculates the PID output applying the PID equation to the given set point target and current
     * input value.
     *
     * @return PID output value.
     */
    public double getOutput(double input) {
        double prevError = currError;
        double currTime = System.currentTimeMillis();
        double deltaTime = currTime - prevTime;
        prevTime = currTime;

        currError = setPoint - input;

        if (inverted) {
            currError = -currError;
        }

        if (kI != 0.0) {
            //
            // Make sure the total error doesn't get wound up too much exceeding maxOutput.
            //
            double potentialGain = (totalError + currError * deltaTime) * kI;

            if (potentialGain >= maxOutput) {
                totalError = maxOutput / kI;
            } else if (potentialGain > minOutput) {
                totalError += currError * deltaTime;
            } else {
                totalError = minOutput / kI;
            }
        }

        pTerm = kP * currError;
        iTerm = kI * totalError;
        dTerm = deltaTime > 0.0 ? kD * (currError - prevError) / deltaTime: 0.0;
        fTerm = kF * setPoint;
        output = fTerm + pTerm + iTerm + dTerm;

        if (output > maxOutput) {
            output = maxOutput;
        } else if (output < minOutput) {
            output = minOutput;
        }

        Log.d(Constants.LOG_TAG, String.format(
                "PID %s: Target=%6.1f, Input=%6.1f, Error=%6.1f, dT=%6.3f, PIDTerms=%6.3f/%6.3f/%6.3f/%6.3f, Output=%6.3f(%6.3f/%5.3f)",
                instanceName, setPoint, input, currError, deltaTime, pTerm, iTerm, dTerm, fTerm, output, minOutput, maxOutput));

        return output;
    }

    /**
     * This method inverts the sign of the calculated error. Normally, the calculated error starts with a large
     * positive number and goes down. However, in some sensors such as the ultrasonic sensor, the target is a small
     * number and the error starts with a negative value and increases. In order to calculate a correct output which
     * will go towards the target, the error sign must be inverted.
     *
     * @param inverted specifies true to invert the sign of the calculated error, false otherwise.
     */
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    /**
     * This method sets the set point mode to be absolute. PID controller always calculates the output with an
     * absolute set point comparing to a sensor value representing an absolute input. But by default, it will
     * treat the set point as a value relative to its current input. So it will add the relative set point value
     * to the current input as the absolute set point in its calculation. This method allows the caller to treat
     * the set point as absolute set point.
     *
     * @param absolute specifies true if set point is absolute, false otherwise.
     */
    public void setAbsoluteSetPoint(boolean absolute) {
        this.absSetPoint = absolute;
    }

    /**
     * This method enables/disables NoOscillation mode. In PID control, if the PID constants are not tuned quite
     * correctly, it may cause oscillation that could waste a lot of time. In some scenarios, passing the target
     * beyond the tolerance may be acceptable. This method allows the PID controller to declare "On Target" even
     * though it passes the target beyond tolerance so it doesn't oscillate.
     *
     * @param noOscillation specifies true to enable no oscillation, false to disable.
     */
    public void setNoOscillation(boolean noOscillation) {
        this.noOscillation = noOscillation;
    }

    /**
     * This method sets a new target tolerance.
     *
     * @param tolerance specifies the new target tolerance.
     */
    public void setTargetTolerance(double tolerance)
    {
        this.tolerance = tolerance;
    }   //setTargetTolerance

    /**
     * This method sets a range limit on the target set point.
     *
     * @param minTarget specifies the target set point lower range limit.
     * @param maxTarget specifies the target set point higher range limit.
     */
    public void setTargetRange(double minTarget, double maxTarget) {
        this.minTarget = minTarget;
        this.maxTarget = maxTarget;
    }

    /**
     * This method sets a range limit on the calculated output. It is very useful to limit the output range to
     * less than full power for scenarios such as using mecanum wheels on a drive train to prevent wheel slipping
     * or slow down a PID drive in order to detect a line etc.
     *
     * @param minOutput specifies the PID output lower range limit.
     * @param maxOutput specifies the PID output higher range limit.
     */
    public void setOutputRange(double minOutput, double maxOutput) {
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }

    /**
     * This method returns the current set point value.
     *
     * @return current set point.
     */
    public double getTarget() {
        return setPoint;
    }

    /**
     * This methods sets the target set point.
     *
     * @param target specifies the target set point.
     */
    public void setTarget(double target, double input) {
        if (!absSetPoint) {
            //
            // Set point is relative, add target to current input to get absolute set point.
            //
            setPoint = input + target;
            currError = target;
        } else {
            //
            // Set point is absolute, use as is.
            //
            setPoint = target;
            currError = setPoint - input;
        }

        if (inverted) {
            currError = -currError;
        }

        setPointSign = Math.signum(currError);

        //
        // If there is a valid target range, limit the set point to this range.
        //
        if (maxTarget > minTarget) {
            if (setPoint > maxTarget) {
                setPoint = maxTarget;
            } else if (setPoint < minTarget) {
                setPoint = minTarget;
            }
        }

        totalError = 0.0;
        prevTime = settlingStartTimeMs = System.currentTimeMillis();
    }

    /**
     * This method returns the error of a previous output calculation.
     *
     * @return previous error.
     */
    public double getError() {
        return currError;
    }

    /**
     * This method resets the PID controller clearing the set point, error, total error and output.
     */
    public void reset() {
        currError = 0.0;
        prevTime = 0.0;
        totalError = 0.0;
        setPoint = 0.0;
        setPointSign = 1.0;
        output = 0.0;
    }

    /**
     * This method determines if we have reached the set point target. It is considered on target if the previous
     * error is smaller than the tolerance and is maintained for at least settling time. If NoOscillation mode is
     * set, it is considered on target if we are within tolerance or pass target regardless of setting time.
     *
     * @return true if we reached target, false otherwise.
     */
    public boolean isOnTarget() {
        boolean onTarget = false;

        if (noOscillation) {
            //
            // Don't allow oscillation, so if we are within tolerance or we pass target, just quit.
            //
            if (currError * setPointSign <= tolerance) {
                onTarget = true;
            }
        } else if (Math.abs(currError) > tolerance) {
            settlingStartTimeMs = System.currentTimeMillis();
        } else if (System.currentTimeMillis() >= settlingStartTimeMs + settlingTimeMs) {
            onTarget = true;
        }

        return onTarget;
    }
}
